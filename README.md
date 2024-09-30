# Intum_CO_detector
CO detectors source code. 

If need SDK download and install nececary files from here: 
https://github.com/Ai-Thinker-Open/Ai-Thinker-LoRaWAN-Ra-08 

Need to add AS923.h in Ai-Thinker-LoRaWAN-Ra-08/lora/mac/region folder
  Default channel frequency is changed 
  
  /*!
 * LoRaMac default channel 1
 * Channel = { Frequency [Hz], RX1 Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
 */
#define AS923_LC1                                   { 920200000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }

/*!
 * LoRaMac default channel 2
 * Channel = { Frequency [Hz], RX1 Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
 */
#define AS923_LC2                                   { 920400000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }


# For compile 

make clean 

make 

