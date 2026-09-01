Omega Brick 4 NMU (ESP32-S3) - DTLS 1.3 firmware notes
=======================================================

Files in this folder (all must travel together - see PARTITION TABLE below):
  stage3_beta_2_dtls.ino          setup()/loop() only; the two FreeRTOS tasks
                                  live in omega_tasks.cpp.
  omega_dtls.h/.cpp               wolfSSL DTLS 1.3 client.
  omega_net.h/.cpp                WiFi + session lifecycle + discovery glue.
  omega_audio.h/.cpp              ADC mic sampling, dB math.
  omega_buffer.h/.cpp             offline ring buffer (RTC_DATA_ATTR).
  omega_tasks.h/.cpp              SentryTask + NetworkTask bodies.
  omega_discovery.h/.cpp          mDNS + UDP-probe server discovery.
  omega_config.h                  every tunable constant, no logic.
  partitions.csv                  REQUIRED exact filename - see below.
  config.h.example                copy to config.h, fill in WLAN + server.
  omega_certs.h                   generate per unit (see below), place here.

Per-unit certificate header (run on the server):
  python3 provisioning/omega_pki.py arduino ./pki NMU_01 omega_certs.h
Copy the resulting omega_certs.h next to this sketch. It contains the device
certificate and PRIVATE KEY - never commit it, flash it only to that unit.

BOARD / TOOLCHAIN (verified 2026-08-16 against real hardware):
  - Board: Adafruit Feather ESP32-S3, NO PSRAM, 8 MB flash.
  - FQBN:  esp32:esp32:adafruit_feather_esp32s3_nopsram
  - Core:  esp32:esp32 (Arduino-ESP32 3.x). Confirm with:
             arduino-cli core list
  - Libraries: ArduinoJson, wolfssl (arduino-cli lib list to confirm).

TWO SETTINGS THE BOARD DOES NOT DEFAULT TO CORRECTLY:

  1. USB CDC On Boot MUST be "Enabled" (FQBN option CDCOnBoot=cdc).
     Board default is OFF. With it off, Serial silently goes to the UART0
     pins instead of the USB port - "no serial output" with no error, easy
     to mistake for a hardware fault. See FINDINGS #13.

  2. partitions.csv MUST be the exact filename in this folder.
     Arduino-ESP32 only auto-detects a sketch-folder partition file named
     EXACTLY "partitions.csv" (confirmed against the official docs:
     https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/partition_table.html).
     Any other name is silently ignored and the board-menu default is used
     instead - which for this board is TinyUF2 8MB (2MB APP/3.7MB FATFS),
     an entirely different, smaller, non-OTA layout. This project's table
     puts app0 at 0x20000 with two OTA-sized app slots; the board default
     puts app0 at 0x10000 with none. Both boot and run FINE on their own,
     which is exactly what makes the mismatch dangerous: nothing fails
     until the day OTA or a partition-address assumption is needed. Verify
     what actually shipped by reading the compiled table itself, not by
     trusting the console summary line:
       python3 -c "
       import struct
       data = open('<build-dir>/stage3_beta_2_dtls.ino.partitions.bin','rb').read()
       for i in range(0, len(data), 32):
           e = data[i:i+32]
           if e[0:2] != b'\xaa\x50': break
           off, size = struct.unpack('<II', e[4:12])
           print(e[12:28].rstrip(b'\x00').decode(), hex(off), hex(size))
       "
     Expect: nvs@0x9000, otadata@0xe000, phy_init@0x10000, app0@0x20000
     (size 0x300000), app1@0x320000, spiffs@0x620000, coredump@0x7f0000.

VERIFIED COMMAND-LINE COMPILE (arduino-cli, no manual file juggling needed
now that partitions.csv is correctly named):
  arduino-cli compile \
    --fqbn esp32:esp32:adafruit_feather_esp32s3_nopsram:CDCOnBoot=cdc \
    --output-dir <out-dir> <this-folder>

FLASHING WITHOUT THE ARDUINO IDE (esptool, bundled with the IDE install at
Arduino15\packages\esp32\tools\esptool_py\<ver>\esptool.exe):
  esptool --chip esp32s3 --port <COMx> --baud 460800 \
    --before default-reset --after hard-reset write-flash -z \
    0x0     <out-dir>\stage3_beta_2_dtls.ino.bootloader.bin \
    0x8000  <out-dir>\stage3_beta_2_dtls.ino.partitions.bin \
    0xe000  <Arduino15 core>\tools\partitions\boot_app0.bin \
    0x20000 <out-dir>\stage3_beta_2_dtls.ino.bin
  Note the 0x20000 app offset matches THIS project's partition table only
  because of the partitions.csv fix above - if that file is ever missing,
  this address is wrong and the board will enumerate but never run.

  This board's native USB port renumbers (COM7/8/9...) across every reset
  and every bootloader entry/exit - detect the port fresh before each
  esptool call rather than hardcoding it.

  TWO DIFFERENT USB IDENTITIES, DO NOT CONFUSE THEM (see FINDINGS #17):
  esptool always talks to VID_303A PID_1001 (the chip's built-in ROM
  download/debug interface - never carries app Serial output). The app's
  own console (CDCOnBoot=cdc) is a SEPARATE identity, VID_239A PID_8113,
  that only appears once the app is actually running. Zero bytes on a
  VID_303A port is expected, not evidence of a dead device.

  USE esptool 5.3.1 AND --after watchdog-reset. On this board the
  USB-Serial/JTAG peripheral can only do a CORE reset, which does not
  re-sample the boot strapping pin - so after a normal --after hard_reset
  the chip STAYS IN DOWNLOAD MODE and needs a physical replug.
  watchdog-reset does a full system reset, re-samples the pin, and the app
  boots on its own. esptool 4.5.1 does NOT have this option; 5.3.1 does,
  and 5.x spells options with hyphens (write-flash, not write_flash).

FLASHING WITH THE ARDUINO IDE (the normal path - the user does this):
  - Tools menu: confirm the two settings above (USB CDC On Boot: Enabled;
    the Partition Scheme menu is IGNORED as long as partitions.csv sits in
    this folder, so it does not need to match).
  - Use a USB 2.0 port and a good data cable.
  - If "Checksum error"/"chip stopped responding": try a slower upload
    speed, or manual bootloader entry (hold BOOT, tap RESET, release BOOT).
  - Serial Monitor at 115200. If it shows nothing at all: wait 2 full
    seconds after the port connects before assuming there is no output -
    this board's native USB CDC drops the first burst of prints if the
    host has not finished enumerating yet (this firmware now pads that gap
    on its own; see FINDINGS #13).

RECOVERY (board stuck / shows up as a USB drive instead of a serial port):
  That is the factory bootloader's safe recovery mode, not damage. Unplug
  and replug the USB cable; if that does not clear it, see Adafruit's
  guide: https://learn.adafruit.com/adafruit-esp32-s3-feather/factory-reset
