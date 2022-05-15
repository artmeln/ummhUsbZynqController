# ummhUsbZynqController

This project provides an example of a UniversalMMHubUsb controller. Using a computer running [Micro-Manager](https://micro-manager.org/), you can utilize the UniversalMMHubUsb device adapter to communicate with this controller. The controller contains simulations of several devices, most notably two cameras that pass data to the computer via bulk USB2 transfers.

(There is a counterpart to this project that uses Arduino Mega and communication over a UART port. This type of communication is much easier to understand and implement, and you may want to look at [ummhSerial_controller_example1](https://github.com/artmeln/ummhSerial_controller_examples/tree/main/ummhSerial_controller_example1) first and test it using [UniversalSerialHub](https://micro-manager.org/universalserialhub) adapter. However, if the device you are planning to implement requires large data transfers, you’ll have to use hardware that is capable, at the very least, of high speed USB2 transfers, similar to the one described below).

The hardware used in this project was the Z-turn board (Xilinx Zynq-7010). It should be easy, however, to convert this project to run on a different Zynq family board, as long as USB device mode is realized on your specific hardware. Start with a ‘Hello world’ Vivado example for the board that you have and make sure that USB0 is enabled among the peripherals. Follow through the example and build the hardware specification (.xsa) file. Then open Xilinx Vitis, create a new application using your XSA file, and then choose the ‘Hello world’ project as your template. Then replace the source files with the source files from ummhUsbZynqController project.

Once you have built your application, you can copy the generated BOOT.BIN file to your SD card (make sure that SD card is the boot mode for your board). Also include devices.txt file from this project as it contains descriptions of implemented devices and, after booting, your application will attempt to read this file.

Connect your board’s USB OTG port to a computer and run Micro-Manager-2.0. Go through the usual hardware configuration and choose UniversalMMHubUsb as the device, then enable devices you want to test among the peripherals. This project does not implement control of any physical devices, it only illustrates principals of communication used by UniversalMMHubUsb adapter.

For troubleshooting purposes, you can use the UART port on your board. All communications happening over USB2 will be duplicated there along with any error messages.
