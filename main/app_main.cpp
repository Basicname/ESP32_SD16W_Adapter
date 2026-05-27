#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "iray_detect.hpp"
#include "nvs_flash.h"
#include "lwip/ip4_addr.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "iray_scanner";

#define WIFI_SSID "iray_scanner_ap"
#define WIFI_PASS "iray1234"
#define WIFI_MAX_RETRY 10
#define WIFI_STATIC_IP_0 192
#define WIFI_STATIC_IP_1 168
#define WIFI_STATIC_IP_2 88
#define WIFI_STATIC_IP_3 50
#define WIFI_GATEWAY_0 192
#define WIFI_GATEWAY_1 168
#define WIFI_GATEWAY_2 88
#define WIFI_GATEWAY_3 1

#define PIN_UART_TX 17
#define PIN_UART_RX 16
#define PIN_SPI_SCK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_INT 4

#define SD16W_UART_NUM UART_NUM_1
#define SD16W_UART_BAUD 115200

#define FRAME_WIDTH 160
#define FRAME_HEIGHT 120
#define FRAME_PIXELS (FRAME_WIDTH * FRAME_HEIGHT)
#define PACKET_BYTES 333
#define PACKETS_PER_FRAME 121
#define SPI_BURST_BYTES (PACKET_BYTES * PACKETS_PER_FRAME)
#define SPI_HOST_DEV SPI2_HOST
#define SPI_CLOCK_HZ (40 * 1000 * 1000)
#define INT_DEBOUNCE_US 5000
#define SPI_DMA_CHUNK 4092
#define DETECT_EVERY_N_FRAMES 1
#define FRAME_WAIT_TIMEOUT_MS 200

#define MAX_BOXES 10
#define BOX_BYTES 9
// +4 bytes for appended raw min/max, +4 bytes frame id for verification.
#define WS_BUF_SIZE (FRAME_PIXELS + 1 + MAX_BOXES * BOX_BYTES + 8)

static uint8_t s_spi_buf[SPI_BURST_BYTES] __attribute__((aligned(4)))
EXT_RAM_BSS_ATTR;
static uint16_t s_temp_raw[FRAME_PIXELS];
static uint8_t s_index_frame[FRAME_PIXELS];
// dual buffer: safe to write while async send is in progress
static uint8_t s_ws_bufs[2][WS_BUF_SIZE];
static int s_ws_buf_idx = 0;

static volatile uint32_t s_last_int_us = 0, s_irq_count = 0;
static TaskHandle_t s_scanner_task = NULL;
static float s_min_c = 0, s_max_c = 0;
static uint32_t s_frame_count = 0;
static uint16_t s_raw_min = 0, s_raw_max = 0;
static uint32_t s_capture_us = 0, s_process_us = 0, s_detect_us = 0;
static uint32_t s_ws_queued = 0, s_ws_skipped = 0;
static uint32_t s_wait_timeouts = 0, s_wait_level_hits = 0;
static uint32_t s_drop_count = 0, s_wait_low_timeouts = 0;
static volatile uint32_t s_irq_period_us = 0;
static volatile uint32_t s_irq_period_min_us = UINT32_MAX;
static volatile uint32_t s_irq_period_max_us = 0;

static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_wifi_retry = 0;
static bool s_wifi_ok = false;
static esp_netif_t *s_wifi_netif = NULL;

static httpd_handle_t s_server = NULL;

#define MAX_WS_CLIENTS 4
static int s_ws_fds[MAX_WS_CLIENTS];
static int s_ws_count = 0;
static SemaphoreHandle_t s_ws_mux = NULL;
static SemaphoreHandle_t s_box_mux = NULL;

static IRayDetect *s_detector = nullptr;
static TaskHandle_t s_detect_task = NULL;

typedef struct {
  int16_t x1;
  int16_t y1;
  int16_t x2;
  int16_t y2;
  uint8_t score;
} box_cache_t;

static box_cache_t s_last_boxes[MAX_BOXES];
static uint8_t s_last_box_count = 0;
static uint32_t s_last_detect_frame = 0;
static uint32_t s_detect_runs = 0, s_detect_skipped = 0;
static volatile bool s_detect_busy = false;
static uint8_t s_detect_frame[FRAME_PIXELS] EXT_RAM_BSS_ATTR;

static void IRAM_ATTR spi_int_isr(void *arg) {
  (void)arg;
  uint32_t now = (uint32_t)esp_timer_get_time();
  uint32_t delta = now - s_last_int_us;
  if (s_last_int_us != 0 && delta < INT_DEBOUNCE_US)
    return;
  if (s_last_int_us != 0) {
    s_irq_period_us = delta;
    if (delta < s_irq_period_min_us)
      s_irq_period_min_us = delta;
    if (delta > s_irq_period_max_us)
      s_irq_period_max_us = delta;
  }
  s_last_int_us = now;
  s_irq_count = s_irq_count + 1;
  BaseType_t woken = pdFALSE;
  if (s_scanner_task) {
    vTaskNotifyGiveFromISR(s_scanner_task, &woken);
    if (woken)
      portYIELD_FROM_ISR();
  }
}

static uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
  }
  return crc;
}

static void sd16w_send_cmd(uint8_t cmd, const uint8_t *payload, uint16_t plen) {
  uint8_t pkt[72];
  int n = 0;
  pkt[n++] = 0xAA;
  pkt[n++] = 0x55;
  pkt[n++] = 0x00;
  pkt[n++] = cmd;
  pkt[n++] = (uint8_t)(plen >> 8);
  pkt[n++] = (uint8_t)(plen & 0xFF);
  uint16_t c1 = crc16_ccitt(pkt, n);
  pkt[n++] = (uint8_t)(c1 >> 8);
  pkt[n++] = (uint8_t)(c1 & 0xFF);
  for (int i = 0; i < (int)plen; i++)
    pkt[n++] = payload[i];
  uint16_t c2 = crc16_ccitt(pkt, n);
  pkt[n++] = (uint8_t)(c2 >> 8);
  pkt[n++] = (uint8_t)(c2 & 0xFF);
  pkt[n++] = 0x0D;
  pkt[n++] = 0x0A;
  uart_write_bytes(SD16W_UART_NUM, (const char *)pkt, n);
  uart_wait_tx_done(SD16W_UART_NUM, pdMS_TO_TICKS(100));
}

static void sd16w_init(void) {
  uart_config_t u = {};
  u.baud_rate = SD16W_UART_BAUD;
  u.data_bits = UART_DATA_8_BITS;
  u.parity = UART_PARITY_DISABLE;
  u.stop_bits = UART_STOP_BITS_1;
  u.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  u.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_param_config(SD16W_UART_NUM, &u));
  ESP_ERROR_CHECK(uart_set_pin(SD16W_UART_NUM, PIN_UART_TX, PIN_UART_RX,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(SD16W_UART_NUM, 256, 0, 0, NULL, 0));
  const uint8_t z = 0x00;
  sd16w_send_cmd(0xF1, &z, 1);
  vTaskDelay(pdMS_TO_TICKS(1100));
  sd16w_send_cmd(0x00, &z, 1);
  ESP_LOGI(TAG, "SD16W init done");
}

static spi_device_handle_t s_spi_dev;
static int s_spi_actual_khz = 0;
static void spi_init(void) {
  spi_bus_config_t b = {};
  b.miso_io_num = PIN_SPI_MISO;
  b.mosi_io_num = -1;
  b.sclk_io_num = PIN_SPI_SCK;
  b.quadwp_io_num = -1;
  b.quadhd_io_num = -1;
  b.max_transfer_sz = SPI_BURST_BYTES;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_DEV, &b, SPI_DMA_CH_AUTO));
  spi_device_interface_config_t d = {};
  d.mode = 0;
  d.clock_speed_hz = SPI_CLOCK_HZ;
  d.spics_io_num = -1;
  d.queue_size = 1;
  d.flags = 0;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_DEV, &d, &s_spi_dev));
  ESP_ERROR_CHECK(spi_device_get_actual_freq(s_spi_dev, &s_spi_actual_khz));
  ESP_LOGI(TAG, "SPI request=%d MHz actual=%.3f MHz", SPI_CLOCK_HZ / 1000000,
           (double)s_spi_actual_khz / 1000.0);
}

static spi_transaction_t s_spi_t;

static bool wait_for_frame_ready(void) {
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(FRAME_WAIT_TIMEOUT_MS)) > 0)
    return true;
  s_wait_timeouts++;
  return false;
}

static void wait_for_frame_not_ready(void) {
  (void)0;
}

static bool capture_frame(void) {
  int64_t t0 = esp_timer_get_time();
  size_t received = 0;
  while (received < SPI_BURST_BYTES) {
    size_t chunk = SPI_BURST_BYTES - received;
    if (chunk > SPI_DMA_CHUNK)
      chunk = SPI_DMA_CHUNK;
    memset(&s_spi_t, 0, sizeof(s_spi_t));
    s_spi_t.length = chunk * 8;
    s_spi_t.rx_buffer = s_spi_buf + received;
    if (spi_device_transmit(s_spi_dev, &s_spi_t) != ESP_OK) {
      return false;
    }
    received += chunk;
    esp_task_wdt_reset();
  }
  s_capture_us = (uint32_t)(esp_timer_get_time() - t0);
  int64_t t1 = esp_timer_get_time();
  taskYIELD();
  uint8_t seen[FRAME_HEIGHT] = {0};
  uint16_t valid = 0;
  for (size_t o = 0; o + PACKET_BYTES <= SPI_BURST_BYTES; o++) {
    const uint8_t *p = &s_spi_buf[o];
    if (p[0] != 0xAA || p[1] != 0x55 || p[2] != 0x00)
      continue;
    if (p[PACKET_BYTES - 2] != 0x0D || p[PACKET_BYTES - 1] != 0x0A)
      continue;
    if (p[3] != 0x25 || ((uint16_t)(p[4] << 8) | p[5]) != 0x0141)
      continue;
    uint8_t line = p[8];
    if (line >= FRAME_HEIGHT || seen[line])
      continue;
    seen[line] = 1;
    uint16_t *dst = &s_temp_raw[(size_t)line * FRAME_WIDTH];
    for (int x = 0; x < FRAME_WIDTH; x++)
      dst[x] =
          (uint16_t)(((uint16_t)p[9 + x * 2] << 8) | p[10 + x * 2]) & 0x3FFF;
    valid++;
    if (valid >= FRAME_HEIGHT)
      break;
    o += PACKET_BYTES - 1;
  }
  if (valid < 115)
    return false;
  uint64_t sum = 0, sumSq = 0;
  uint16_t minV = 0x3FFF, maxV = 0;
  for (int i = 0; i < FRAME_PIXELS; i++) {
    uint16_t v = s_temp_raw[i];
    if (v < minV)
      minV = v;
    if (v > maxV)
      maxV = v;
    sum += v;
    sumSq += (uint64_t)v * v;
  }
  float fn = (float)FRAME_PIXELS, mean = (float)sum / fn;
  float var = (float)sumSq / fn - mean * mean;
  if (var < 0)
    var = 0;
  float std = sqrtf(var);
  int rMin = (int)(mean - 2.5f * std);
  if (rMin < (int)minV)
    rMin = (int)minV;
  int rMax = (int)(mean + 2.5f * std);
  if (rMax > (int)maxV)
    rMax = (int)maxV;
  if (rMax <= rMin + 2) {
    rMin = minV;
    rMax = maxV;
  }
  uint32_t range = (uint32_t)(rMax - rMin);
  for (int i = 0; i < FRAME_PIXELS; i++) {
    int v = (int)s_temp_raw[i];
    if (v < rMin)
      v = rMin;
    if (v > rMax)
      v = rMax;
    uint32_t sc = ((uint32_t)(v - rMin) * 255U) / range;
    s_index_frame[i] = (uint8_t)(sc > 255 ? 255 : sc);
  }
  // Save true raw min/max for WS payload and stats
  s_raw_min = minV;
  s_raw_max = maxV;
  s_min_c = (float)minV / 10.0f - 273.0f;
  s_max_c = (float)maxV / 10.0f - 273.0f;
  s_process_us = (uint32_t)(esp_timer_get_time() - t1);
  return true;
}

static void index_to_rgb565be(const uint8_t *idx, uint16_t *out, int n) {
  for (int i = 0; i < n; i++) {
    uint8_t v = idx[i];
    uint8_t r, g, b;
    if (v < 64) {
      r = 0;
      g = (uint8_t)(v * 4);
      b = 255;
    } else if (v < 128) {
      r = 0;
      g = 255;
      b = (uint8_t)(255 - (v - 64) * 4);
    } else if (v < 192) {
      r = (uint8_t)((v - 128) * 4);
      g = 255;
      b = 0;
    } else {
      r = 255;
      g = (uint8_t)(255 - (v - 192) * 4);
      b = 0;
    }
    uint16_t px = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    out[i] = (uint16_t)((px >> 8) | (px << 8));
  }
}

static size_t build_ws_payload(void) {
  uint8_t *buf = s_ws_bufs[s_ws_buf_idx];
  memcpy(buf, s_index_frame, FRAME_PIXELS);

  box_cache_t boxes[MAX_BOXES];
  uint8_t cnt = 0;
  if (s_box_mux && xSemaphoreTake(s_box_mux, pdMS_TO_TICKS(1)) == pdTRUE) {
    cnt = s_last_box_count;
    memcpy(boxes, s_last_boxes, cnt * sizeof(box_cache_t));
    xSemaphoreGive(s_box_mux);
  }
  buf[FRAME_PIXELS] = cnt;
  uint8_t *p = buf + FRAME_PIXELS + 1;
  for (int i = 0; i < cnt; i++) {
    int16_t bx1 = boxes[i].x1, by1 = boxes[i].y1;
    int16_t bx2 = boxes[i].x2, by2 = boxes[i].y2;
    uint8_t sc = boxes[i].score;
    p[0] = (uint8_t)(bx1 >> 8);
    p[1] = (uint8_t)(bx1 & 0xFF);
    p[2] = (uint8_t)(by1 >> 8);
    p[3] = (uint8_t)(by1 & 0xFF);
    p[4] = (uint8_t)(bx2 >> 8);
    p[5] = (uint8_t)(bx2 & 0xFF);
    p[6] = (uint8_t)(by2 >> 8);
    p[7] = (uint8_t)(by2 & 0xFF);
    p[8] = sc;
    p += BOX_BYTES;
  }
  // append raw min/max (big-endian uint16) for JS click-to-temp
  p[0] = (uint8_t)(s_raw_min >> 8);
  p[1] = (uint8_t)(s_raw_min & 0xFF);
  p[2] = (uint8_t)(s_raw_max >> 8);
  p[3] = (uint8_t)(s_raw_max & 0xFF);
  p[4] = (uint8_t)(s_frame_count >> 24);
  p[5] = (uint8_t)(s_frame_count >> 16);
  p[6] = (uint8_t)(s_frame_count >> 8);
  p[7] = (uint8_t)(s_frame_count & 0xFF);
  return (size_t)(FRAME_PIXELS + 1 + cnt * BOX_BYTES + 8);
}

static void submit_detection_frame(void) {
  if (!s_detector || !s_detect_task)
    return;
  if ((s_frame_count % DETECT_EVERY_N_FRAMES) != 0)
    return;
  if (s_detect_busy) {
    s_detect_skipped++;
    return;
  }
  memcpy(s_detect_frame, s_index_frame, FRAME_PIXELS);
  s_detect_busy = true;
  xTaskNotifyGive(s_detect_task);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id,
                               void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    s_wifi_ok = false;
    xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    if (s_wifi_retry < WIFI_MAX_RETRY) {
      esp_wifi_connect();
      s_wifi_retry++;
      ESP_LOGW(TAG, "WiFi retry %d", s_wifi_retry);
    } else {
      xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
    }
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
    s_wifi_retry = 0;
    s_wifi_ok = true;
    xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
  }
}

static void wifi_init(void) {
  s_wifi_eg = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  s_wifi_netif = esp_netif_create_default_wifi_sta();
  ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_wifi_netif));
  esp_netif_ip_info_t ip = {};
  IP4_ADDR(&ip.ip, WIFI_STATIC_IP_0, WIFI_STATIC_IP_1, WIFI_STATIC_IP_2,
           WIFI_STATIC_IP_3);
  IP4_ADDR(&ip.gw, WIFI_GATEWAY_0, WIFI_GATEWAY_1, WIFI_GATEWAY_2,
           WIFI_GATEWAY_3);
  IP4_ADDR(&ip.netmask, 255, 255, 255, 0);
  ESP_ERROR_CHECK(esp_netif_set_ip_info(s_wifi_netif, &ip));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  esp_event_handler_instance_t h1, h2;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &h1));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &h2));
  wifi_config_t wc = {};
  strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
  strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);
  wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_ps(WIFI_PS_NONE);
  EventBits_t bits =
      xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                          pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
  if (!(bits & WIFI_CONNECTED_BIT))
    ESP_LOGE(TAG, "WiFi connect failed");
}

static void ws_add_client(int fd) {
  xSemaphoreTake(s_ws_mux, portMAX_DELAY);
  s_ws_fds[0] = fd;
  s_ws_count = 1;
  xSemaphoreGive(s_ws_mux);
  ESP_LOGI(TAG, "WS client fd=%d", fd);
}

static volatile bool s_ws_sending = false;
static int s_ws_send_fd = -1;
static size_t s_ws_send_len = 0;
static uint8_t *s_ws_send_buf = NULL;

static void ws_send_cb(void *arg) {
  (void)arg;
  if (s_ws_send_buf == NULL) {
    s_ws_sending = false;
    return;
  }
  httpd_ws_frame_t pkt = {};
  pkt.type = HTTPD_WS_TYPE_BINARY;
  pkt.payload = s_ws_send_buf;
  pkt.len = s_ws_send_len;
  esp_err_t r = httpd_ws_send_frame_async(s_server, s_ws_send_fd, &pkt);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "WS send fail fd=%d err=%d", s_ws_send_fd, r);
    xSemaphoreTake(s_ws_mux, portMAX_DELAY);
    s_ws_count = 0;
    xSemaphoreGive(s_ws_mux);
  }
  s_ws_sending = false;
}

static void ws_broadcast(size_t payload_len) {
  if (!s_server || s_ws_count == 0)
    return;
  if (s_ws_sending) {
    s_ws_skipped++;
    return;
  }
  int fd;
  xSemaphoreTake(s_ws_mux, portMAX_DELAY);
  fd = (s_ws_count > 0) ? s_ws_fds[0] : -1;
  xSemaphoreGive(s_ws_mux);
  if (fd < 0)
    return;
  uint8_t *buf = s_ws_bufs[s_ws_buf_idx];
  s_ws_buf_idx ^= 1;
  s_ws_send_fd = fd;
  s_ws_send_len = payload_len;
  s_ws_send_buf = buf;
  s_ws_sending = true;
  esp_err_t r = httpd_queue_work(s_server, ws_send_cb, NULL);
  if (r != ESP_OK) {
    s_ws_sending = false;
    s_ws_skipped++;
    ESP_LOGW(TAG, "queue_work fail err=%d", r);
  } else {
    s_ws_queued++;
  }
}

static esp_err_t ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) {
    ws_add_client(httpd_req_to_sockfd(req));
    return ESP_OK;
  }
  httpd_ws_frame_t pkt = {};
  pkt.type = HTTPD_WS_TYPE_TEXT;
  httpd_ws_recv_frame(req, &pkt, 0);
  return ESP_OK;
}

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static esp_err_t http_root_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  const char *html = (const char *)index_html_start;
  size_t html_len = (size_t)(index_html_end - index_html_start);
  if (html_len > 0 && html[html_len - 1] == '\0')
    html_len--;
  return httpd_resp_send(req, html, html_len);
}

static esp_err_t http_stats_handler(httpd_req_t *req) {
  char buf[512];
  uint32_t irq_min =
      (s_irq_period_min_us == UINT32_MAX) ? 0 : s_irq_period_min_us;
  snprintf(buf, sizeof(buf),
           "{\"frames\":%lu,\"min_c\":%.1f,\"max_c\":%.1f,"
           "\"capture_us\":%lu,\"process_us\":%lu,\"detect_us\":%lu,"
           "\"detect_runs\":%lu,\"detect_skipped\":%lu,"
           "\"ws_queued\":%lu,\"ws_skipped\":%lu,\"detect_interval\":%d,"
           "\"irq\":%lu,\"irq_period_us\":%lu,"
           "\"irq_period_min_us\":%lu,\"irq_period_max_us\":%lu,"
           "\"wait_timeouts\":%lu,\"wait_level_hits\":%lu,"
           "\"drops\":%lu,\"wait_low_timeouts\":%lu,"
           "\"spi_requested_hz\":%d,\"spi_actual_khz\":%d}",
           (unsigned long)s_frame_count, s_min_c, s_max_c,
           (unsigned long)s_capture_us, (unsigned long)s_process_us,
           (unsigned long)s_detect_us, (unsigned long)s_detect_runs,
           (unsigned long)s_detect_skipped, (unsigned long)s_ws_queued,
           (unsigned long)s_ws_skipped, DETECT_EVERY_N_FRAMES,
           (unsigned long)s_irq_count, (unsigned long)s_irq_period_us,
           (unsigned long)irq_min, (unsigned long)s_irq_period_max_us,
           (unsigned long)s_wait_timeouts, (unsigned long)s_wait_level_hits,
           (unsigned long)s_drop_count,
           (unsigned long)s_wait_low_timeouts, SPI_CLOCK_HZ,
           s_spi_actual_khz);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static void start_server(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  cfg.max_open_sockets = 7;
  cfg.lru_purge_enable = true;
  if (httpd_start(&s_server, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Server start failed");
    return;
  }
  static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = http_root_handler,
    .user_ctx = NULL,
#if CONFIG_HTTPD_WS_SUPPORT
    .is_websocket = false,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL,
#endif
  };
  static const httpd_uri_t stat = {
    .uri = "/stats",
    .method = HTTP_GET,
    .handler = http_stats_handler,
    .user_ctx = NULL,
#if CONFIG_HTTPD_WS_SUPPORT
    .is_websocket = false,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL,
#endif
  };
  static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL,
#if CONFIG_HTTPD_WS_SUPPORT
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL,
#endif
  };
  httpd_register_uri_handler(s_server, &root);
  httpd_register_uri_handler(s_server, &stat);
  httpd_register_uri_handler(s_server, &ws);
  ESP_LOGI(TAG, "Server started port 80 (HTTP+WS)");
}

static void scanner_task(void *arg) {
  (void)arg;
  esp_task_wdt_add(NULL);
  s_scanner_task = xTaskGetCurrentTaskHandle();
  uint32_t drop = 0, wait_miss = 0, last_log = 0;
  while (1) {
    esp_task_wdt_reset();
    if (!wait_for_frame_ready()) {
      wait_miss++;
      continue;
    }
    if (!capture_frame()) {
      drop++;
      s_drop_count = drop;
      wait_for_frame_not_ready();
      continue;
    }
    s_frame_count++;
    wait_for_frame_not_ready();
    ulTaskNotifyTake(pdTRUE, 0);
    submit_detection_frame();
    if (s_wifi_ok) {
      size_t plen = build_ws_payload();
      ws_broadcast(plen);
    }
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - last_log >= 5000) {
      last_log = now;
      ESP_LOGI(
          TAG,
          "[Status] frames=%lu drop=%lu wait=%lu irq=%lu T=%.1f~%.1fC ws=%d heap=%lu",
          (unsigned long)s_frame_count, (unsigned long)drop,
          (unsigned long)wait_miss, (unsigned long)s_irq_count, s_min_c,
          s_max_c, s_ws_count,
          (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
    }
  }
}

static void detect_task(void *arg) {
  (void)arg;
  static uint16_t rgb_buf[FRAME_PIXELS];
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t td0 = esp_timer_get_time();
    index_to_rgb565be(s_detect_frame, rgb_buf, FRAME_PIXELS);
    dl::image::img_t img;
    img.data = rgb_buf;
    img.width = FRAME_WIDTH;
    img.height = FRAME_HEIGHT;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;
    auto &results = s_detector->run(img);
    box_cache_t boxes[MAX_BOXES];
    uint8_t cnt =
        (uint8_t)(results.size() > MAX_BOXES ? MAX_BOXES : results.size());
    auto it = results.begin();
    for (int i = 0; i < cnt; i++, ++it) {
      const auto &res = *it;
      boxes[i].x1 = (int16_t)res.box[0];
      boxes[i].y1 = (int16_t)res.box[1];
      boxes[i].x2 = (int16_t)res.box[2];
      boxes[i].y2 = (int16_t)res.box[3];
      boxes[i].score = (uint8_t)(res.score * 255.0f);
    }
    if (s_box_mux && xSemaphoreTake(s_box_mux, portMAX_DELAY) == pdTRUE) {
      memcpy(s_last_boxes, boxes, cnt * sizeof(box_cache_t));
      s_last_box_count = cnt;
      s_last_detect_frame = s_frame_count;
      xSemaphoreGive(s_box_mux);
    }
    s_detect_runs++;
    s_detect_us = (uint32_t)(esp_timer_get_time() - td0);
    s_detect_busy = false;
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "iray_scanner starting - ESP32-S3-N16R8");
  ESP_ERROR_CHECK(nvs_flash_init());
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << PIN_SPI_INT);
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_ENABLE;
  io.intr_type = GPIO_INTR_POSEDGE;
  ESP_ERROR_CHECK(gpio_config(&io));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(
      gpio_isr_handler_add((gpio_num_t)PIN_SPI_INT, spi_int_isr, NULL));
  spi_init();
  sd16w_init();
  wifi_init();
  s_ws_mux = xSemaphoreCreateMutex();
  s_box_mux = xSemaphoreCreateMutex();
  start_server();
  s_detector = new IRayDetect(IRayDetect::IRAY_PICO_120_160, false);
  xTaskCreatePinnedToCore(detect_task, "detect", 8192, NULL, 1, &s_detect_task,
                          0);
  xTaskCreatePinnedToCore(scanner_task, "scanner", 8192, NULL, 5, NULL, 1);
  ESP_LOGI(TAG, "Scanner task started");
}
