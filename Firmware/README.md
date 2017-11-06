# B200 Demo software

## Program the device

Precompiled firmware is provided and can be used with B200 hardware. The .hex file is available in the ```/hex``` folder. 

## Compile the software

* Clone this repository  
```> git clone https://github.com/u-blox/blueprint-B200-NINA-B1```

* Download the source code for nrf5 sdk version 11.0.0 from [Nordic](https://www.nordicsemi.com)

* Update the SDK\_ROOT variable in the Makefiles (for the main project and the bootloader) according to your installation of the nrf5 SDK  
```For example: SDK_ROOT := C:/nRF5_SDK_11.0.0```  
If using the Eclipse IDE update the SDK\_ROOT for the Eclipse .project too

* Go to the bootloader directory (where the Makefile is located) and execute the ```make``` command

* Go to the main project directory (where the Makefile is located) and execute the ```make``` command

* Execute the ```make create_demo_product``` command to create a combined .hex file including SoftDevice, bootloader and application

* The generated software (B200-3D_Demo.hex) is located in the ```/bin``` folder