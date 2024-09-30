# Intum_CO_detector
CO detectors source code. 

If need SDK download and install nececary files from here: 
https://github.com/Ai-Thinker-Open/Ai-Thinker-LoRaWAN-Ra-08 

Need to add AS923.h in Ai-Thinker-LoRaWAN-Ra-08/lora/mac/region folder
* Default channel frequency is changed 
   
      #define AS923_LC1                                   { 920200000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
      #define AS923_LC2                                   { 920400000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }

# For compile 

make clean 

make 

