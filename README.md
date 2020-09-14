# CubeCellRepeater

This is the code for a simple repeater node for the meshtastic project: https://github.com/meshtastic

The actual node can be one of the LoRa CubeCell nodes by Heltec Automation: https://github.com/HelTecAutomation/ASR650x-Arduino/

Notes:

Will work only for the channel "Default" with setting "Very long range (but slow)"!
Modify radio settings for your own channels.

Will work with any packet meeting the radio settings, but the serial output is based on the assumption that the node receives meshtastic packets.

Will only send serial data, if "#define SILENT" is deleted, commented or set to another value (e.g. VERBOSE).
