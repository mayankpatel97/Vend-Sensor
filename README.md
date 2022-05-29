
Vend sensor is used to detect an item has vend in Vending Machines. It sense the product and send a signal to 
the main motherboard, indicating item has vend. Responding to this signal motherboard will stop rotating motor

Every vending machine has a rectangular bucket from where user collects the purchased items. This sensor will 
be installed there, so it can detect and count each and every product dropped by machine in to the bucket.

A vend sensor must be sensitive and fast enough to detect even tiny products. 

The product has two pcbs 
1) Transmitter : An array IR LEDs transmitting a 56Khz modulated signal. This pcb is connected with Receiver board through a cable
2) Receiver : This board consist of IR demodulators connected to an STM32F0 microcontroller.


************************** PCB ****************************************
Receiver PCB 
![image](https://user-images.githubusercontent.com/92679540/170858481-0a065fb2-5f77-4b04-9530-9d008d23d54d.png)



![image](https://user-images.githubusercontent.com/92679540/170858890-867f650d-6bde-43d9-8d59-21c74072518f.png)


Transmitter PCB 

![image](https://user-images.githubusercontent.com/92679540/170858941-45cc4502-9380-4bf2-a421-94ee7c18bb6d.png)

************************* Software *************************************

As I mentioned above the circuit is built around STM32F0 microcontroller, it uses HAL drivers.
Follow below steps to make the code working 

1) Download the code and open STM32 Cube file and regenerate the code.
2) Once the code has generated, open the project in keil MDK ARM, and build the code.
3) Then upload the code in the microcontroller. 

for further details contact me at 
mayankpatel468@gmail.com 
or send a whatsapp message on 
+918802846486







