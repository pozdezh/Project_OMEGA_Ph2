MAKING NEW NMU UNITS
====================

WHAT YOU NEED
-------------
  - This pendrive (it holds the key that gives each unit its identity)
  - The server PC, switched on
  - One or more ESP32 boards
  - A USB cable

WHAT TO DO
----------
  1. Plug this pendrive into the server PC.

  2. Open the pendrive in the file manager (it appears in the sidebar).

  3. Double-click "Make NMU Units".

     If double-clicking does nothing, instead:
     right-click empty space in this folder -> "Open in Terminal",
     then type:   ./make-units

     Nothing is copied by hand. The tool runs from the drive and puts
     whatever the server needs where it belongs, by itself.

  4. Answer three questions:
       - your WiFi network name
       - your WiFi password
       - how many units you want, and what to call each one
         (press Enter to accept the suggested name)

  5. The tool then handles one unit at a time. For each one it will:
       - build that unit's own firmware
       - ask you to plug in a board
       - flash it
       - tell you when to unplug it and plug in the next one

  6. When it finishes, each unit joins the server on its own within
     about a minute. Check the dashboard to see them appear.

IF A BOARD IS NOT FOUND
-----------------------
  Put it into flashing mode by hand. Leave the cable plugged in:

    1. Hold down the small BOOT button and keep holding it.
    2. Press the RESET button once and let go of RESET.
    3. Now let go of BOOT.

  Then run the tool again and give that unit the same name as before.

IMPORTANT
---------
  Keep this pendrive safe and do not leave it plugged in.

  The key on it is what lets new units be trusted by the system.
  While it is unplugged, nobody can add a unit to your fleet -
  not even from the server itself. That is the point of it being
  on a pendrive rather than on the server's hard disk.
