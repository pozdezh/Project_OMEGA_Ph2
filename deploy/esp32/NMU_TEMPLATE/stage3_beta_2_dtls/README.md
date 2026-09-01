# NMU template

This folder is the complete latest NMU firmware package. Make a copy of the folder for each physical board.

## Before compiling

1. Copy `config.h.example` to `config.h`.
2. Replace `YOUR_WLAN_SSID` and `YOUR_WLAN_PASSWORD` in `config.h`.
3. Generate the device certificate on the server:

```bash
python3 /home/smart/omega_brick4/omega_pki.py arduino \
  /home/smart/omega_brick4/pki_provisioning NMU_17 \
  /tmp/omega_certs_NMU_17.h
```

Replace `NMU_17` with the board name. Copy the generated file into this folder and call it `omega_certs.h`.

Do not edit the generated certificate file. It contains the private key and must be flashed only to that named board.

## Arduino IDE

- Board: Adafruit ESP32-S3 Feather (no PSRAM)
- USB CDC On Boot: Enabled
- Port: select the connected board
- Do not change or remove `partitions.csv`
- Click Verify, then Upload
- Serial Monitor: 115200 baud

The brick4 server port is already set to `11400`, and the empty server address enables discovery.

## Command-line compile

```bash
arduino-cli compile \
  --fqbn esp32:esp32:adafruit_feather_esp32s3_nopsram:CDCOnBoot=cdc \
  --output-dir build .
```

Keep one board connected at a time. The generated `omega_certs.h`, `config.h`, and `build/` directory are local files and must not be committed.
