# Test Harness for debugging I2C communication issues between 2 Raspberry Pi Pico's (RP2040)

### This was created to debug I2C communication issues that i encountered between 2 Raspberry Pi Pico's (RP2040) connected together using i2c

### This is based on the 256 byte buffer transfer idea from the SPI master-slave example from the Pico examples Repo:
https://github.com/raspberrypi/pico-examples/tree/master/spi/spi_master_slave

It  uses this i2c slave library:
https://github.com/vmilea/pico_i2c_slave


### I have modified the way the example operates to first send a separate, single byte, data transfer before the buffer is transferred. The output and input buffers have also been reduced from 256 bytes to 255.
I increased the send rate from 1 per second to 10 per second.

There is also a lot of extra serial output and error capturing and reporting has been added.

In order to properly view the serial output you will need to use a proper terminal emulator, ie. one that supports ANSI control characters. I use iTerm2 on MacOS with a command to launch screen against the usb tty.

### What it does
The Sender Pico sends a single value to the Receiver Pico via the I2C, in this case the length of the buffer to be sent next.
Immediately after this it sends the 255 byte buffer, (this was 256 bytes in the original SPI master-slave example).

The Sender, Master Pico then sends an I2C "Read" request to the Slave Pico to ask for its own buffer, like a register read for a normal I2C slave device. The Slave Pico then responds with its own buffer.
### The issue

Intermittent missing data was observed on the receiving Pico during the i2c communications, the error rate seems to depend on the i2c clock speed and other factors such as how much serial communications is taking place and when these communications take place relative to the i2c communications.

When the errors are observed, all of the the data has been confirmed to be correctly sent on the wire by the transmitting Pico, this has been confirmed via an oscilloscope and by the use of it's protocol decoder function.

A lower clock speed reduces the error rate.

Errors still occur if there is no USB serial usage during data reception and even when the USB cable is unplugged and the boards are running on batteries.

pull-up resistors have been changed from 4.7k to 1k, this made no difference.

Serial communications occurring during reception of i2c data increased the error rate.

Serial communications just before to the reception of i2c data also increased the error rate, but less so.

### To fix the issue

Go to line 61 in the i2c_slave library:

pico_i2c_slave/i2c_slave/i2c_slave.c

and uncomment the improved version of the i2c_slave_irq_handler function, it is not clear why this seems to fix the issue, the re-ordering of the IRQ flag checks seems to be the main thing that reduces the error rate.
