# CubeCellRepeater

This is the code for a simple repeater node for the meshtastic project: https://github.com/meshtastic (https://www.meshtastic.org)
The Meshtastic project is in no way affiliated with or respnsible for the CubeCellRepeater project.

To be compatible with meshtastic project, portions of their code was used.
(c) Copyright Meshtastic Project (and others).

Hardware:
The actual node can be one of the LoRa CubeCell nodes by Heltec Automation: https://github.com/HelTecAutomation/ASR650x-Arduino/

Notes:

Intended for use with the platform.io IDE. Serial output speed is 115200.
See the provided platformio.ini for built-in environments. Default is cubecell_board.

Will repeat packets ONCE with HopLimit reduced by one. To prevent flooding the last 64 repeated packets will not be repeated again.
If the HopLimit was already 0, the packet will not be repeated.

Keep in mind that re-sending packets will cause the initial sender to assume that the packet is "received" or at least in the mesh.
If no other meshtastic node is in range of either the node or the repeater, the message will still be shown as received. 

Will work with most packets meeting the radio settings, but the serial output is based on the assumption that the node receives meshtastic packets.
Minimum size for none-Meshtastic packets is 16 bytes.

The node can decode meshtastic packets meeting some requirements:
- they key is the default meshtastic key or provided by you (see main.h)
- the packets are sent over channel 0, primary channel. This should already be true for all telemetry, node info, traces and DM.

## main.h
"#define SILENT" in main.h to stop serial output.

"#define CC_MONITOR_ONLY true" to stop repeating packets and just monitor the traffic via serial output.

You can modify the radio settiungs in main.h (e.g using a preset, but selecting an alternate frequency slot. Or using your own freq/sf/cr settings).

List of meshtastic-supported regions:

- United States meshtastic_Config_LoRaConfig_RegionCode_US
- European Union 433mhz meshtastic_Config_LoRaConfig_RegionCode_EU_433
- European Union 868mhz (default) meshtastic_Config_LoRaConfig_RegionCode_EU_868
- China meshtastic_Config_LoRaConfig_RegionCode_CN
- Japan meshtastic_Config_LoRaConfig_RegionCode_JP
- Australia / New Zealand meshtastic_Config_LoRaConfig_RegionCode_ANZ
- Korea meshtastic_Config_LoRaConfig_RegionCode_KR
- Taiwan meshtastic_Config_LoRaConfig_RegionCode_TW
- Russia meshtastic_Config_LoRaConfig_RegionCode_RU
- India meshtastic_Config_LoRaConfig_RegionCode_IN
- New Zealand 865mhz meshtastic_Config_LoRaConfig_RegionCode_NZ_865
- Thailand meshtastic_Config_LoRaConfig_RegionCode_TH
- WLAN Band meshtastic_Config_LoRaConfig_RegionCode_LORA_24
- Ukraine 433mhz meshtastic_Config_LoRaConfig_RegionCode_UA_433
- Ukraine 868mhz meshtastic_Config_LoRaConfig_RegionCode_UA_868
- Malaysia 433mhz meshtastic_Config_LoRaConfig_RegionCode_MY_433
- Malaysia 919mhz meshtastic_Config_LoRaConfig_RegionCode_MY_919
- Singapore 923mhz meshtastic_Config_LoRaConfig_RegionCode_SG_923

For lora modem settings you should always try the default (LongFast), it's proven to be THE setting to go for.
If you want to try other presets, here is the list:

- meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST
- meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW
- meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST
- meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW
- meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST      (default)
- meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE
- meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW
- meshtastic_Config_LoRaConfig_ModemPreset_VERY_LONG_SLOW
