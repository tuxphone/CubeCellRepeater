# CubeCellRepeater

This is the code for a simple repeater node for the meshtastic project: https://github.com/meshtastic

The actual node can be one of the LoRa CubeCell nodes by Heltec Automation: https://github.com/HelTecAutomation/ASR650x-Arduino/

Notes:

Intended for use with the platform.io IDE. Depends on the NanoPB Lib. Serial output speed is 115200.
See the provided platformio.ini for built-in environments. Default is cubecell_board.

Will repeat packets ONCE. To prevent flooding the last repeated packet ID will not be repeated again.

Keep in mind that re-sending packets will cause the initial sender to assume that the packet is "received" or at least in the mesh.
If no other meshtastic node is in range of either the node or the repeater, the message will still be shown as received. 
You can use this for range tests.

Will work with most packets meeting the radio settings, but the serial output is based on the assumption that the node receives meshtastic packets.
Minimum size for none-Meshtastic packets is 14 bytes.

"#define SILENT" to stop serial output.

"#define NOBLINK" to NOT getting a red blink from the RGB LED for the duration of sending a packet (can be quite long at speed setting 3).

Modify radio settings for your own channels:

Edit the CONFIGURATION block in config.h

e.g.
#define REGION  RegionCode_EU865  -  defines your region (to EU865). For US, use RegionCode_US, for CN use RegionCode_CN etc. See config.h for more supported regions.

MESHTASTIC_SPEED    3   - defines your speed to "very long range". Values are:  0 = short range, 1 = medium range, 2 = long range, 3 = very long range

MESHTASTIC_NAME[12] = {"Default"} - sets your Channel Name, but without "-Xy" suffix , e.g. use "Test" instead of "Test-A"

TX_MAX_POWER     14  -  sets output power to 14 dB. This value will also be used, wenn output power is set to Zero in your RegionCode (0 = max. power). TX_MAX_POWER will be ignored, when higher than RegionCode maximum.
