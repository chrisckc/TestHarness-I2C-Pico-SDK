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
#define DEBUG_SERIAL_DURING_I2C_RECEIVE (true)
#define DEBUG_SERIAL_DURING_I2C_RESPONSE (true)

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

// underscore vars only to be modified by the i2c_slave_handler ISR
volatile uint8_t _byte = 0, _prevByte = 0;;
volatile unsigned int _byteIndex = 0, _expectedByteCount = 0, _bytesReceived = 0;
volatile unsigned int _byteRequestedIndex = 0;
volatile bool _i2cDataReceiveInProgress = false, _i2cDataRequestInProgress = false;

volatile bool i2cDataReady = false, i2cDataRequested = false, i2cDataRequestCompleted = false;
volatile unsigned int  bytesAvailable = 0, bytesExpectedOrRequested = 0, bytesSent = 0;
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

unsigned int verifyInBuffer(unsigned int page, bool printOnlyFirstError) {
    int errorCount = 0;
    for (uint8_t i = 0; i < BUF_LEN; ++i) {
        if (in_buf[i] != i + 1) {
            if (errorCount == 0 && printOnlyFirstError) {
                printf("ERROR! page: %07u First Error at index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i + 1, in_buf[i]);
            } else if (!printOnlyFirstError) {
                printf("ERROR! page: %07u index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i + 1, in_buf[i]);
            }
            errorCount++;
        }
    }
    return errorCount;
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
        _i2cDataReceiveInProgress = true;
        if (_byteIndex == 1) {
            in_buf[0] = _prevByte; // backfill the first buffer byte now that we have more than 1 byte received
        }
        if (_byteIndex > 0) {
            in_buf[_byteIndex] = _byte; // fill the buffer
        }
        _prevByte = _byte;
        _byteIndex++;
        gpio_put(DEBUG_PIN2, 1); // signal the end of the interrupt
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        gpio_put(DEBUG_PIN4, 0); // signal the start of the interrupt
        i2c_write_byte(i2c, out_buf[_byteRequestedIndex]); // send the requested data, the i2c_write_byte function is used in the example provided in the library
        _i2cDataRequestInProgress = true;
        if (_byteRequestedIndex == 0) i2cDataRequested = true; // only set this once at the start
        _byteRequestedIndex++;
        if (_byteRequestedIndex >= BUF_LEN) {
            bytesSent += _byteRequestedIndex; // just roll-over the buffer
            _byteRequestedIndex = 0;
        }
        gpio_put(DEBUG_PIN4, 1); // signal the end of the interrupt
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        gpio_put(DEBUG_PIN3, 0); // signal the start of the interrupt
        // check if this is a finish event due to a read request from the master
        if (_i2cDataRequestInProgress) {
            bytesSent += _byteRequestedIndex;
            _byteRequestedIndex = 0;
            _i2cDataRequestInProgress = false;
            i2cDataRequestCompleted = true;
        }
        // check if this is a finish event due to a receive from the master
        if (_i2cDataReceiveInProgress) {
            // if we only received a single byte, this is the prefix or command byte indicating the size of the buffer to be transferred next from the master
            // It can also be a command to indicate the size of the buffer the master wants to read back, serviced by the I2C_SLAVE_REQUEST case
            if (_byteIndex == 1) {
                bytesExpectedOrRequested = _byte; // this can also indicate the number of bytes requested
                bytesSent = 0;
            } else {
                bytesAvailable = _byteIndex;
                i2cDataReady = true;
            }
            _byteIndex = 0;
            _i2cDataReceiveInProgress = false;
        }
        gpio_put(DEBUG_PIN3, 1); // signal the end of the interrupt
        break;
    default:
        break;
    }
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

    printf("I2C receiver Pico-SDK example using i2c baud rate: %d \r\n", I2C_BAUDRATE);
    printf("rp2040_chip_version: %u \r\n", rp2040_chip_version());
    printf("rp2040_rom_version: %u \r\n", rp2040_rom_version());
    printf("get_core_num: %u \r\n", get_core_num());
    printf("DEBUG_SERIAL_DURING_I2C_RECEIVE: %s \r\n", DEBUG_SERIAL_DURING_I2C_RECEIVE ? "true" : "false");
    printf("DEBUG_SERIAL_DURING_I2C_RESPONSE: %s \r\n\r\n", DEBUG_SERIAL_DURING_I2C_RESPONSE ? "true" : "false");

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

    gpio_set_function(DEBUG_PIN5, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN5);
    gpio_set_dir(DEBUG_PIN5, GPIO_OUT);
    gpio_put(DEBUG_PIN5, DEBUG_PIN_INITIAL_STATE);

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
    unsigned int verifyErrorCount = 0;

    // Loop for ever...
    for (size_t i = 0; ; ++i) {
        // Check if we have all the expected data gathered by the receive interrupts
        if (i2cDataReady && (i2cDataRequestCompleted || DEBUG_SERIAL_DURING_I2C_RESPONSE)) {
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

                gpio_put(DEBUG_PIN5, 0);
                if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                    // Reset the previous terminal position if we are not scrolling the output
                    if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                        printf("\e[H"); // move to the home position, at the upper left of the screen
                        printf("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
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
                    printf("I2C Receiver says: read page %u from the sender, received page size: %03u expected: %03u lastExpected: %03u \r\n", receiveCounter, bytesAvailable, bytesExpectedOrRequested, lastBytesExpected);
                    // Write the input buffer out to the USB serial port
                    printBuffer(in_buf, BUF_LEN);

                    printf("I2C Receiver says: Verifying received data...                           \r\n");
                    if (bytesExpectedOrRequested != BUF_LEN) {
                        receiveErrorCount++;
                        printf("ERROR!!! page: %u bytesExpected: %03u should equal the Buffer Length: %03u                               \r\n", receiveCounter, bytesExpectedOrRequested, BUF_LEN);
                    }
                    verifyErrorCount = verifyInBuffer(receiveCounter, false);
                    receivedBytesErrorCount += verifyErrorCount;
                    // Check that we only record the error once for each receive cycle
                    if (bytesExpectedOrRequested == BUF_LEN && verifyErrorCount > 0) receiveErrorCount++;
                }
                clearBuffer(in_buf, BUF_LEN);
                lastBytesExpected = bytesExpectedOrRequested;
                bytesAvailable = 0;
                gpio_put(DEBUG_PIN5, 1);
            }
            // Reset the flag
            i2cDataReady = false;
            gpio_put(LED_BUILTIN, 0); // turn off the LED
        }
        if (!i2cDataReady && i2cDataRequested && (i2cDataRequestCompleted || DEBUG_SERIAL_DURING_I2C_RESPONSE)) {
            sleep_us(10); // delay so we can easily see the debug pulse
            gpio_put(DEBUG_PIN5, 0);
            // Leave room for up to 4 previous error report lines not to be overwritten
            for (int v = 1; v <= 4; ++v) {
                if (v > verifyErrorCount) printf("\n");
            }
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                printf("I2C Receiver says: Responding to Request from the Sender (master) for the Output buffer... (page %u, bytes requested: %03u)                         \r\n", receiveCounter, bytesExpectedOrRequested);
            }
            if (bytesExpectedOrRequested == BUF_LEN) {
                printf("\r\n");
            } else {
                sendErrorCount++;
                if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                    printf("ERROR!!! Unexpected number of bytes requested by the Sender (master) for the Output buffer...  (page %u, bytes requested: %03u) \r\n", receiveCounter, bytesExpectedOrRequested);
                }
            }
            // Reset the flag
            i2cDataRequested = false;
        }
        if (!i2cDataReady && !i2cDataRequested && i2cDataRequestCompleted) {
            sendCounter++;
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                printf("I2C Receiver says: Responded to Request from the Sender (master) for the Output buffer  (page %u, bytesSent: %03u)          \r\n", sendCounter, bytesSent);
            }
            if (bytesSent == bytesExpectedOrRequested) {
                printf("\r\n");
            } else {
                sendErrorCount++;
                if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                    printf("ERROR!!! Responding to Request from the Sender (master) for the Output buffer, buffer not completely sent  (page %u, bytesSent: %03u) \r\n", sendCounter, bytesSent);
                }
            }
            // Reset the flag
            i2cDataRequestCompleted = false;
            //printf("                                                                                                                              \r\n");
            gpio_put(DEBUG_PIN5, 1);
        }
        if (DEBUG_SERIAL_DURING_I2C_RECEIVE && receiveCounter > 0) {
            // Just keep printing something out of USB Serial every 100 uS, this will overlap with i2c receive.
            // 100 uS is around 10 bytes received from i2c at 1MHz, so while receiving 255 bytes we will have decent overlap for testing
            sleep_us(100);
            printf(".");
        }
    }
}
