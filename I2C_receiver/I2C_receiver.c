// I2C Test Harness by Chris Claxton
// Based on the SPI master-slave and i2c examples from the Pico examples repo
// I2C_Receiver

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

// https://github.com/vmilea/pico_i2c_slave
#include <i2c_fifo.h>
#include <i2c_slave.h>

// Debug Signal outputs
#define LED_BUILTIN (25u)
#define DEBUG_PIN2 (6u)
#define DEBUG_PIN3 (7u)
#define DEBUG_PIN4 (8u)
#define DEBUG_PIN5 (9u)
#define DEBUG_PIN_INITIAL_STATE (1)

// Serial data output and debugging options
#define DEBUG_SERIAL_OUTPUT_SCROLLING (false) // If not scrolling the terminal position is reset using escape sequences, proper terminal emulator required
#define DEBUG_SERIAL_OUTPUT_PAGE_LIMIT (0) // Set to zero to show all pages

#define I2C_INSTANCE i2c0 // Valid pins below must be used for each i2c instance
#define I2C_SLAVE_SDA_PIN (4u)
#define I2C_SLAVE_SCL_PIN (5u)
#define I2C_SLAVE_ADDRESS (0x30)
//#define I2C_BAUDRATE      (400000u)  // 400 kHz
#define I2C_BAUDRATE      (1000000u) // 1 MHz

#define BUF_LEN         0xFF // 255 byte buffer
uint8_t bufferLength = BUF_LEN;
uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

bool ledState = false;
unsigned int seconds = 0, lastSeconds = 0;
unsigned int receiveCounter = 0, lastReceiveCount = 0, receiveRate = 0, receiveErrorCount = 0, sendCounter = 0, sendErrorCount = 0;
unsigned int receivedBytesErrorCount = 0;

volatile unsigned int _startByte = 0; // A pre-agreed start byte between the sender and receiver, not implemented yet.
volatile uint8_t _byte = 0;
volatile unsigned int _byteIndex = 0, _expectedByteCount = 0, _bytesReceived = 0;
volatile bool i2cDataReady = false, i2cDataRequested = false;
volatile unsigned int  bytesAvailable = 0, bytesExpected = 0;
unsigned int lastBytesAvailable = 0, lastBytesExpected = 0;

void printBuffer(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02X \r\n", buf[i]);
        else
            printf("%02X ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        printf("   \r\n");
    }
}

bool verifyInBuffer(unsigned int page, bool printOnlyFirstError) {
    bool success = true;
    for (uint8_t i = 0; i < BUF_LEN; ++i) {
        if (in_buf[i] != i + 1) {
            receivedBytesErrorCount++;
            if (success && printOnlyFirstError) {
                printf("ERROR! page: %07u First Error at index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i + 1, in_buf[i]);
            } else if (!printOnlyFirstError) {
                printf("ERROR! page: %07u index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i + 1, in_buf[i]);
            }
            success = false;
        }
    }
    return success;
}

void clearBuffer(uint8_t buf[], size_t len) {
    for (int i = 0; i < len; ++i) {
        buf[i] = 0;
    }
}

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        gpio_put(DEBUG_PIN2, 0); // signal the start of the interrupt
        _byte = i2c_read_byte(i2c);
        in_buf[_byteIndex] = _byte;
        _byteIndex++;
        gpio_put(DEBUG_PIN2, 1); // signal the end of the interrupt
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        gpio_put(DEBUG_PIN4, 0); // signal the start of the interrupt
        //i2c_write_byte(i2c, 0xff); // dummy data
        i2cDataRequested = true;
        gpio_put(DEBUG_PIN4, 1); // signal the end of the interrupt
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        gpio_put(DEBUG_PIN3, 0); // signal the start of the interrupt
        // if we only received a single byte, this is the prefix byte indicating the size of the buffer to be transferred next
        if (_byteIndex == 1) {
            bytesExpected = _byte;
            in_buf[_byteIndex] = 0; // erase the buffer at the first location
            bytesAvailable = 0;
        } else {
            bytesAvailable = _byteIndex;
            i2cDataReady = true;
        }
        _byteIndex = 0;
        gpio_put(DEBUG_PIN3, 1); // signal the end of the interrupt
        break;
    default:
        break;
    }
}

// Uses the i2c_write_byte function defined in the i2c slave library:
// https://github.com/vmilea/pico_i2c_slave
// the i2c_write_byte function is used in the example provided in the library
void sendBufferToMasterUsingFifo(uint8_t length) {
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        printf("I2C Receiver says: Sending Response Output buffer...  (page %u, buffer size: %03u) \r\n", sendCounter, length);
    }
    gpio_put(DEBUG_PIN3, 0);
    // First send the data length of the buffer so the other side knows what to expect next
    i2c_write_byte(I2C_INSTANCE, length);

    // Write the output buffer to the UART
    for (uint8_t x = 0; x < length; ++x) {
        sleep_us(100); // delay so we don't overflow the buffer, crude but ok for now
        i2c_write_byte(I2C_INSTANCE,out_buf[x]); // dummy data
    }
    gpio_put(DEBUG_PIN3, 1);
    sendCounter++;
}


// This method just hangs on i2c_write_blocking, maybe this function is not supported in slave mode?
int sendBufferToMaster(uint8_t length) {
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        printf("I2C Receiver says: Sending Response Output buffer...  (page %u, buffer size: %03u) \r\n", sendCounter, length);
    }
    gpio_put(DEBUG_PIN3, 0);
    // First send the data length of the buffer so the other side knows what to expect next
    int byteCount = i2c_write_blocking(I2C_INSTANCE, I2C_SLAVE_ADDRESS, &length, 1, false);
    if (byteCount < 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            printf("I2C Receiver says: ERROR!!! Could not send Output buffer size for page %u, Couldn't write to slave, please check your wiring! \r\n", sendCounter);
        }
        gpio_put(DEBUG_PIN3, 1);
        return 0;
    }
    // Now send the buffer
    byteCount = i2c_write_blocking(I2C_INSTANCE, I2C_SLAVE_ADDRESS, out_buf, length, false);
    if (byteCount < 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            printf("I2C Receiver says: ERROR!!! Could not send Output buffer page %u, Couldn't write to slave, please check your wiring! \r\n", sendCounter);
        }
        gpio_put(DEBUG_PIN3, 1);
        return 0;
    }
    gpio_put(DEBUG_PIN3, 1);
    sendCounter++;
    return byteCount;
}

// uses https://github.com/vmilea/pico_i2c_slave
static void setupSlave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(I2C_INSTANCE, I2C_BAUDRATE);
    // configure I2C_INSTANCE for slave mode
    i2c_slave_init(I2C_INSTANCE, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main() {
    // Enable UART so we can print
    stdio_init_all();

    int startupDelay = 9;
    for (int i = 1; i <= startupDelay; ++i) {
        printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    printf("\e[2J\e[H"); // clear screen and go to home position

    printf("I2C receiver example using i2c baud rate: %d \r\n", I2C_BAUDRATE);
    printf("rp2040_chip_version: %u \r\n", rp2040_chip_version());
    printf("rp2040_rom_version: %u \r\n", rp2040_rom_version());
    printf("get_core_num: %u \r\n\r\n", get_core_num());

    // Init the onboard LED
    gpio_set_function(LED_BUILTIN, GPIO_FUNC_SIO);
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, GPIO_OUT);
    gpio_put(LED_BUILTIN, ledState);

    // Init the debug pins
    gpio_set_function(DEBUG_PIN2, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN2);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    gpio_put(DEBUG_PIN2, DEBUG_PIN_INITIAL_STATE);

    gpio_set_function(DEBUG_PIN3, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN3);
    gpio_set_dir(DEBUG_PIN3, GPIO_OUT);
    gpio_put(DEBUG_PIN3, DEBUG_PIN_INITIAL_STATE);

    gpio_set_function(DEBUG_PIN4, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN4);
    gpio_set_dir(DEBUG_PIN4, GPIO_OUT);
    gpio_put(DEBUG_PIN4, DEBUG_PIN_INITIAL_STATE);

    sleep_us(10); // delay so we can easily see the debug pulse
    gpio_put(DEBUG_PIN3, 0); // signal the start of I2C config

    // Setup the I2C hardware
    setupSlave();
    gpio_put(DEBUG_PIN3, 1); // signal the end of I2C config

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // bit-inverted from i. The values should be: {0xff, 0xfe, 0xfd...}
        out_buf[i] = ~i;
    }
    clearBuffer(in_buf, BUF_LEN);

    printf("I2C Receiver says: After reading I2C data from RX, the value: 0x%02X (%u) (buffer size) and then the following buffer will be written to the sender:\r\n", BUF_LEN, BUF_LEN);
    printBuffer(out_buf, BUF_LEN);
    printf("\r\n");

    unsigned long startMillis = to_ms_since_boot(get_absolute_time());
    unsigned long currentMillis = 0;
    int bytesSent = 0;

    // Loop for ever...
    for (size_t i = 0; ; ++i) {
        // Check if we have all the expected data gathered by the receive interrupts
        if (i2cDataReady) {
            gpio_put(LED_BUILTIN, 1); // turn on the LED
            if (bytesAvailable == 0) {
                printf("ERROR!!! Received empty transmission from the Sender (master) page: %u bytesAvailable: %03u                              \r\n", receiveCounter, bytesAvailable);
            } else {
                receiveCounter++;
                // Keep track of seconds since start
                currentMillis = to_ms_since_boot(get_absolute_time());
                seconds = (currentMillis - startMillis) / 1000;
                if (seconds - lastSeconds > 0) {
                    lastSeconds = seconds;
                    //calculate the receive rate, per second
                    receiveRate = receiveCounter - lastReceiveCount;
                    lastReceiveCount = receiveCounter;
                }

                gpio_put(DEBUG_PIN3, 0);
                if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                    // Reset the previous terminal position if we are not scrolling the output
                    if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                        printf("\e[H"); // move to the home position, at the upper left of the screen
                        printf("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
                    }
                    // Print the header info
                    printf("\r\nSeconds: %07u.%03u       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
                    printf("receiveCounter: %07u         \r\n", receiveCounter);
                    printf("receiveRate: %07u            \r\n", receiveRate);
                    printf("Receive errorCount: %03u         \r\n", receiveErrorCount);
                    printf("Receive FailureRate: %11.7f percent  \r\n", 100.0f * receiveErrorCount / (receiveCounter > 0 ? receiveCounter : 1));
                    printf("receivedBytesErrorCount: %03u         \r\n", receivedBytesErrorCount);
                    printf("sendCounter: %07u             \r\n", sendCounter);
                    printf("Send errorCount: %03u             \r\n", sendErrorCount);
                    printf("Send FailureRate: %11.7f percent  \r\n", 100.0f * sendErrorCount / (sendCounter > 0 ? sendCounter : 1));
                    printf("Data Received...                                                                \r\n");

                    // print data to the serial port
                    printf("I2C Receiver says: read page %u from the sender, received page size: %03u expected: %03u lastExpected: %03u \r\n", receiveCounter, bytesAvailable, bytesExpected, lastBytesExpected);
                    // Write the input buffer out to the USB serial port
                    printBuffer(in_buf, BUF_LEN);

                    printf("I2C Receiver says: Verifying received data...                           \r\n");
                    if (bytesExpected != BUF_LEN) {
                        receiveErrorCount++;
                        printf("ERROR!!! page: %u bytesExpected: %03u should equal the Buffer Length: %03u                               \r\n", receiveCounter, bytesExpected, BUF_LEN);
                    }
                    bool verifySuccess = verifyInBuffer(receiveCounter, false);
                    // Check that we only record the error once for each receive cycle
                    if (bytesExpected == BUF_LEN && !verifySuccess) receiveErrorCount++;
                }
                clearBuffer(in_buf, BUF_LEN);
                lastBytesExpected = bytesExpected;
                i2cDataReady = false;
                bytesAvailable = 0;
                gpio_put(DEBUG_PIN3, 1);
            }
            gpio_put(LED_BUILTIN, 0); // turn off the LED
        }
        if (i2cDataRequested) {
            sleep_us(10); // delay so we can easily see the debug pulse
            gpio_put(DEBUG_PIN3, 0);
            // Send data back to the sender...
            // Having issues getting this to work...

            // bytesSent = sendBufferToMaster(BUF_LEN);

            // alternative method
            //sendBufferToMasterUsingFifo(BUF_LEN);

            i2cDataRequested = false;
            gpio_put(DEBUG_PIN3, 1);

            // if (bytesSent == BUF_LEN) {
            //     printf("I2C Receiver says: Responded with Output buffer page %u, buffer size: %03u \r\n", receiveCounter, BUF_LEN);
            // } else if (bytesSent == 0) {
            //     sendErrorCount++;
            //     printf("I2C Receiver says: ERROR!!! Could not Respond with Output buffer page %u,  Couldn't write to slave, please check your wiring! \r\n", receiveCounter);
            // } else {
            //     sendErrorCount++;
            //     printf("I2C Receiver says: ERROR!!! Output buffer page %u, was not fully sent, bytesSent: %d \r\n", receiveCounter, bytesSent);
            // }
        }
    }
}
