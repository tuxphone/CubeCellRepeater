# CubeCellRepeater

This is the code for a simple repeater node for the meshtastic project: https://github.com/meshtastic

The actual node can be one of the LoRa CubeCell nodes by Heltec Automation: https://github.com/HelTecAutomation/ASR650x-Arduino/

Notes:

Intended for use with the platform.io IDE. Depends on the NanoPB Lib and Base64 Lib (see platformio.ini). Serial output speed is 115200.
See the provided platformio.ini for built-in environments. Default is cubecell_board.

Will repeat packets ONCE with HopLimit reduced by one. To prevent flooding the last repeated packet ID will not be repeated again.
If the HopLimit was already 0, the packet will not be repeated.

Keep in mind that re-sending packets will cause the initial sender to assume that the packet is "received" or at least in the mesh.
If no other meshtastic node is in range of either the node or the repeater, the message will still be shown as received. 

Will work with most packets meeting the radio settings, but the serial output is based on the assumption that the node receives meshtastic packets.
Minimum size for none-Meshtastic packets is 14 bytes.

"#define SILENT" in main.h to stop serial output.

Modifying radio settings for your own radio settings:  edit the main.h
