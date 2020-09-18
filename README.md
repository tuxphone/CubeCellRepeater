# CubeCellRepeater

This is the code for a simple repeater node for the meshtastic project: https://github.com/meshtastic

The actual node can be one of the LoRa CubeCell nodes by Heltec Automation: https://github.com/HelTecAutomation/ASR650x-Arduino/

Notes:

Will repeat packets ONCE. To prevent flooding a short list of received packet IDs is checked against the ID of the current packet.
Keep in mind that re-sending packets will cause the initial sender to assume that the packet is "received" or at least in the mesh.
If no other meshtastic node is in range of either the node or the repeater, the message will still be shown as received. 
You can use this for range tests.

Will work with any packet meeting the radio settings, but the serial output is based on the assumption that the node receives meshtastic packets.
"#define SILENT" to stop serial output.

Modify radio settings for your own channels:

Look for the following defines and change to your needs. The settings will be stored in "CanSet" and used to configure the radio.

#define LORA_SPREADING_FACTOR       12          // [SF5..SF12]

#define LORA_CODINGRATE             4           // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]

#define LORA_CHAN_NUM               0           // Channel Number (meshtastic specific)


Note: with "HW_VERSION" you select your country/continent settings. For US use HW_VERSION_US, for CN use HW_VERSION_CN. See Meshradio.h

#define HW_VERSION_EU865 // define your region _before_ including MeshRadio.h !

#include "MeshRadio.h"


ChannelSettings ChanSet={ TX_OUTPUT_POWER, ChannelSettings_ModemConfig_Bw125Cr48Sf4096, mPSK, "Default", 
                        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_CHAN_NUM };
