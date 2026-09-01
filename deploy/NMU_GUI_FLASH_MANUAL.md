# NMU quick flash instructions

Use one board at a time.

## 1. Make a copy

Copy this complete folder and rename the copy, for example:

`brick4_dtls13/deploy/esp32/NMU_TEMPLATE`

Open the inner folder in Arduino IDE:

`NMU_TEMPLATE/stage3_beta_2_dtls`

Do not remove `partitions.csv`.

## 2. Prepare the two files

Inside the inner `stage3_beta_2_dtls` folder:

1. Copy `config.h.example` to `config.h`.
2. Replace `YOUR_WLAN_SSID` and `YOUR_WLAN_PASSWORD`.
3. On the server, create the certificate for this board:

```bash
python3 /home/smart/omega_brick4/omega_pki.py arduino \
  /home/smart/omega_brick4/pki_provisioning NMU_17 \
  /tmp/omega_certs_NMU_17.h
```

Replace `NMU_17` everywhere with the real board name. Copy the generated file into the inner folder and name it exactly `omega_certs.h`.

The certificate file contains the private key. Flash it only to that named board.

## 3. Arduino IDE

Select:

- Board: `Adafruit ESP32-S3 Feather (no PSRAM)`
- USB CDC On Boot: `Enabled`
- Port: the connected ESP32-S3

Click **Verify**, then **Upload**.

If upload cannot find the board, put it into flashing mode by hand. Leave
the cable plugged in: hold **BOOT**, press **RESET** once and release RESET,
then release **BOOT**. Upload again.

## 4. Check it

Open Serial Monitor at `115200` baud. The board should connect to WiFi and then to the server.

The template already uses the correct brick4 settings:

- server discovery enabled
- server port `11400`

## Important

Never use the same generated `omega_certs.h` on two boards. For `NMU_18`, generate a new file with `NMU_18` and replace the old one before compiling.

The generated `omega_certs.h`, local `config.h`, and build output must not be committed.
