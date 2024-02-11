# **STM32-WiFi-Streaming-Microphone**


- [**STM32-WiFi-Streaming-Microphone**](#stm32-wifi-streaming-microphone)
- [**1. Global Configurations**](#1-global-configurations)
  - [**1.1 RCC Clock**](#11-rcc-clock)
- [**2. Configuring Microphones**](#2-configuring-microphones)
  - [**2.1 Microphone Connections**](#21-microphone-connections)
  - [**2.2 Microphone Channels**](#22-microphone-channels)
    - [**2.2.1 Single Microphone Channel Configuration**](#221-single-microphone-channel-configuration)
    - [**2.2.2 Dual Microphone Channel Configuration**](#222-dual-microphone-channel-configuration)
  - [**2.3 DFSDM Clock Output**](#23-dfsdm-clock-output)
  - [**2.4 DFSDM Output Frequency**](#24-dfsdm-output-frequency)
  - [**2.5 DFSDM Output Resolution**](#25-dfsdm-output-resolution)
- [3. WiFi Module](#3-wifi-module)
- [**References**](#references)


# **1. Global Configurations**

## **1.1 RCC Clock**




# **2. Configuring Microphones**

## **2.1 Microphone Connections**

The STM32L475E-IOT01A board features a stereo microphone configuration with two microphones connected to the DFSDM (Digital Filter for Sigma-Delta Modulators) peripheral via channel 2. These microphones utilize the DFSDM1 pins to receive clock signals for data sampling and output. The schematic image below illustrates the connection setup between the two microphones and the DFSDM1 pins on the board:

<div style="text-align:center">
  <img src="./img/Microphones_Schematic.png">
</div>


## **2.2 Microphone Channels**

There are two microphones both connected to Input Channel 2 of the DFSDM1, as illustrated in Section 2.1. It's important to note that one channel of the DFSDM1 peripheral can process data from only one microphone because each microphone sends data on either the rising or falling edge, and the channel needs to filter data on one type of edge. See the following image for the data organization of two microphones. (see document[2] for more details).

So there is a need for two Channels to process data from two microphones; the configuration is explained in Section 2.3.2.

<div style="text-align:center">
  <img src="./img/Stereo_Channel_MIC_data.png" width="600" height="300">
</div>


### **2.2.1 Single Microphone Channel Configuration**

In this project, I am using only one microphone and, therefore, using one channel with any type of clock edges, as shown in the following image:


<div style="text-align:center">
  <img src="./img/DFSDM1_Channels_for_one_MIC.png" width="600" height="600">
</div>


### **2.2.2 Dual Microphone Channel Configuration**

I figured out that in the DFSDM peripheral, each channel can redirect data to its preceding channel. So, Channel 1 can take the input data of Channel 2, and we configure each one with different types of edges so that one could acquire the left channel data, and the other could acquire the right channel data. See the configuration in the following image:

<div style="text-align:center">
  <img src="./img/DFSDM1_Channels_for_two_MICs.png" width="600" height="600">
</div>


## **2.3 DFSDM Clock Output**

From the MP34DT01 datasheet, we can observe the permissible range of clock frequencies as illustrated in the following figure that can be found on this link:
https://www.st.com/resource/en/datasheet/mp34dt01-m.pdf

<div style="text-align:center">
  <img src="./img/MP34DT01-m_characteristics.png" width="600" height="600">
</div>

So, I configured the output clock from the DFSDM peripheral to 2 MHz by selecting the system clock as the clock source and applying a divider of 40 (80 MHz / 40 = 2 MHz).

<div style="text-align:center">
  <img src="./img/DFSDM1_Clock_Output.png">
</div>


## **2.4 DFSDM Output Frequency**

To achieve the desired sampling frequency, four filters can be associated with any channel, but only to one. Then, there is the FOSR value that needs to be modified according to this equation:

    Output Frequency = Output Clock / FOSR.
​
In my case, I have configured the output clock to 2 MHz, and I chose the value 64 for FOSR so that I can achieve a frequency of 31,250 Hz. See the configuration in the following image:

<div style="text-align:center">
  <img src="./img/DFSDM1_Filters.png">
</div>

## **2.5 DFSDM Output Resolution**

The output resolution of samples is determined by the filter type and the FOSR value, as illustrated in the following table, which was extracted from the microcontroller datasheet [3].

<div style="text-align:center">
  <img src="./img/DFSDM1_Filter_Maximum_Output_Resolution.png" width="600" >
</div>

So, in my case, I chose the filter type Sinc3 and an FOSR value of 64, resulting in an output of signed 19 bits: +-262144.

**!!! Nota Bene !!!**

The filter's maximum resolution is 24 bits; exceeding it will loose the higher bits of data. 


# 3. WiFi Module

WiFi Driver:
https://github.com/STMicroelectronics/STM32CubeL4/tree/master/Projects/B-L475E-IOT01A/Applications/WiFi


# **References**




- **Microcontroller:**

[3] STM32L47xxx [https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf]

- **DFSDM:**

[1] STM32L4 Training Hands on DFSDM : https://www.youtube.com/watch?v=MdDqVeIGhec 

[2] Take a look at this Application Note: !(DM00380469)[https://www.st.com/resource/en/application_note/dm00380469-interfacing-pdm-digital-microphones-using-stm32-mcus-and-mpus-stmicroelectronics.pdf]


- **SD Card:**

[4] Application notes from ANALOG about SD card protocol: https://www.analog.com/media/en/technical-documentation/application-notes/AN-1443.pdf

[5] Tutorial: An SD card over SPI using STM32CubeIDE and FatFS: https://01001000.xyz/2020-08-09-Tutorial-STM32CubeIDE-SD-card/
