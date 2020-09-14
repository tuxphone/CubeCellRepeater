# CubeCellRepeater

This is the code for a simple repeater node for the meshtastic project: https://github.com/meshtastic

The actual node can be one of the LoRa CubeCell nodes by Heltec Automation: https://github.com/HelTecAutomation/ASR650x-Arduino/

Notes:

Will repeat packets ONCE. To prevent flooding a short list of received packet IDs is checked against the ID of the current packet.
Keep in mind that re-sending packets will cause the initial sender to assume that the packet is "received" or at least in the mesh.
If no other meshtastic node is in range of either the node or the repeater, the message will still be shown as received. 
You can use this for range tests.

Will work only for the channel "Default" with setting "Very long range (but slow)"!
Modify radio settings for your own channels.

Will work with any packet meeting the radio settings, but the serial output is based on the assumption that the node receives meshtastic packets.

Will only send serial data, if "#define SILENT" is deleted, commented or set to another value (e.g. VERBOSE).

Comment "#include <Arduino.h>" when using the Arduino IDE. The include is needed when using platform.io.
