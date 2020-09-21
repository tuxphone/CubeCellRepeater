# CubeCellRepeater

This is the code for a simple repeater node for the meshtastic project: https://github.com/meshtastic

The actual node can be one of the LoRa CubeCell nodes by Heltec Automation: https://github.com/HelTecAutomation/ASR650x-Arduino/

Notes:

Will repeat packets ONCE. To prevent flooding a short list of received packet IDs is checked against the ID of the current packet.
Keep in mind that re-sending packets will cause the initial sender to assume that the packet is "received" or at least in the mesh.
If no other meshtastic node is in range of either the node or the repeater, the message will still be shown as received. 
You can use this for range tests.

Will work with most packets meeting the radio settings, but the serial output is based on the assumption that the node receives meshtastic packets.
Minimum size for none-Meshtastic packets is 14 bytes.

"#define SILENT" to stop serial output.

Modify radio settings for your own channels:

Edit the CONFIGURATION block in MeshRadio.h

e.g.
HW_VERSION_EU865  -  defines your region (to EU). For US, use HW_VERSION_US, for CN use HW_VERSION_CN etc.

MESHTASTIC_SPEED    3   - defines your speed to "very long range". Other values are:  0 = short range, 1 = medium range, 2 = long range, 3 = very long range

MESHTASTIC_NAME[12] = {"Default"} - sets your Channel Name, but without "-Xy" suffix , e.g. use "Test" instead of "Test-A"

TX_OUTPUT_POWER     22  -  sets output power to 22 dB. Keep in mind the maximums set by law and the hardware
