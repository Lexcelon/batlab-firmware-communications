Batlab V1.0 Communications Processor Firmware

This firmware was developed using the MPLABX IDE and the free version of the XC8 compiler. The target device is a PIC16F1454 microcontroller.

Description:

The Communications PIC is the liaison between the main processor on the Batlab
and the PC. It handles the routing of command messages from the PC to the Batlab Measurement Processor, as well as forwarding responses from the Measurment Processor to the PC. It uses a system of circular buffers to route messages between the USB and UART interfaces.

This processor also controls a STATUS LED for each of the 4 cells, and the detection of an external 5V power supply

More information, along with a complete programming user's guide, can be found at http://www.lexcelon.com


