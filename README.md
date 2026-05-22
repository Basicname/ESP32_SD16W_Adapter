# ESP32 SD16W Adapter Firmware

Firmware for the open-source DALI adapter hardware published at <https://oshwhub.com/basicname/dali_adapter>.

This project connects a DALI SD16W thermal imaging module to a custom ESP32-S3 adapter board. The ESP32-S3 captures the SD16W thermal stream, serves a browser-based viewer over Wi-Fi, and overlays local ESP-DL detection results on the live thermal image.

## Hardware

- **Adapter board:** custom ESP32-S3 board for the DALI SD16W adapter, open-sourced on OSHWHub.
- **Thermal module:** DALI SD16W. Public product listings describe it as an uncooled FPA thermal imaging module with 160 × 120 / 120 × 120 resolution options, 25 µm pixel pitch, 8–14 µm spectral range, SPI digital video output, and 5 V power input.
- **Default target:** ESP32-S3-N16R8, configured for 16 MB flash and 8 MB Octal PSRAM.

## Firmware Features

- Captures 160 × 120 thermal frames from the SD16W over SPI.
- Initializes and controls the SD16W over UART.
- Runs an ESP-DL detection model locally on the ESP32-S3.
- Hosts an HTTP/WebSocket thermal viewer from the ESP32-S3.
- Streams pseudo-color thermal frames, detection boxes, and min/max raw temperature values to the browser.

## Pinout

| Signal | ESP32-S3 GPIO |
| --- | ---: |
| SD16W UART TX | GPIO17 |
| SD16W UART RX | GPIO16 |
| SD16W SPI SCK | GPIO18 |
| SD16W SPI MISO | GPIO19 |
| SD16W frame interrupt | GPIO4 |

## Private Model and Dataset

The trained face-detection model and training dataset are intentionally excluded from this repository because the model was trained on private face data.

Local builds expect the model file at:

```text
components/iray_model/models/s3/espdet_pico_120_160_inface.espdl
```

Keep that file locally when building, but do not commit or publish it. The `.gitignore` file excludes `*.espdl` and `dataset/` for this reason.

## Build

Activate ESP-IDF, then build:

```bash
esp
idf.py build
```

The `esp` shell alias is expected to source the local ESP-IDF environment on the development machine.

## Flash and Run

```bash
esp
idf.py -p /dev/ttyUSB0 flash monitor
```

After boot, connect to the configured Wi-Fi network and open the device address in a browser to view the live thermal stream.

## Repository Layout

- `main/` — application entry point, SD16W capture, Wi-Fi, HTTP/WebSocket server, and browser UI asset.
- `components/iray_model/` — ESP-DL wrapper for the local thermal face-detection model.
- `partitions.csv` — ESP32-S3 partition table for the firmware image and optional model partition.
- `sdkconfig.defaults` — default ESP32-S3-N16R8 build configuration.

## References

- Adapter hardware: <https://oshwhub.com/basicname/dali_adapter>
- DALI SD16W public product information: <https://dali-tech2021.en.made-in-china.com/product/IZCfDRthHmUs/China-Dali-SD16W-Cheap-Non-Contact-IR-Human-Body-Temperature-Measurement-Infrared-Imaging-Module.html>
