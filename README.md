# ps2kbd
An Arduino PS/2 keyboard controller

The PS/2 keyboard predates USB, but the interface is pretty simple and easy to build an interface for. 
There are only two non-power wires: a clock line and data line. Both are open-collector and bidirectional.
Mostly the keyboard sends scan codes representing key presses and releases, but there are commands
that the host can send to it, the most interesting of which are reset and turning the LED's on and off.

The interface I built has two pullup resistors to deal with the fact that the inputs are open-collector
(i.e. zero is ground, but one is floating), and two transistors to take over the bus from the 
microcontroller side.

The sketch is more of a demo right now than anything. It decodes keypresses assuming a standard 101-key
US layout, and ignores function keys, arrow keys, the numeric keypad, etc. and sends the characters back
over the Arduino's USB/serial interface so they can be seen in the serial monitor. They could just as
easily be clocked out on an 8-bit parallel interface to another system. The shift keys and caps lock
key are supported, and caps lock will correctly toggle the state of the caps lock LED.

For more information on the low level protocol, see http://www.burtonsys.com/ps2_chapweske.htm.

For information on scan codes and keyboard commands, see https://wiki.osdev.org/PS/2_Keyboard.
