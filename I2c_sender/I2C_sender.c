// I2C Test Harness by Chris Claxton
// Based on the SPI master-slave and i2c examples from the Pico examples repo
// I2C_Sender

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

// https://github.com/vmilea/pico_i2c_slave
// #include <i2c_fifo.h>
// #include <i2c_slave.h>

// Debug Signal outputs
#define LED_BUILTIN (25u)
#define DEBUG_PIN2 (2u)
#define DEBUG_PIN3 (3u)
#define DEBUG_PIN4 (4u)
#define DEBUG_PIN5 (5u)
#define DEBUG_PIN_INITIAL_STATE (1)

// Serial data output and debugging options
#define DEBUG_SERIAL_OUTPUT_SCROLLING (false) // If not scrolling the terminal position is reset using escape sequences, proper terminal emulator required
#define DEBUG_SERIAL_OUTPUT_PAGE_LIMIT (0) // Set to zero to show all pages
#define DEBUG_SERIAL_OUTPUT_DURING_I2C_READ (false) // Set to false to prevent USB Serial debug output during I2C data reception

#define I2C_INSTANCE i2c1 // Valid pins below must be used for each i2c instance
#define I2C_SLAVE_SDA_PIN (6u)
#define I2C_SLAVE_SCL_PIN (7u)
#define I2C_SLAVE_ADDRESS (0x30)
//#define I2C_BAUDRATE      (400000u)  // 400 kHz
#define I2C_BAUDRATE      (1000000u) // 1 MHz

#define BUF_LEN         0xFF // 255 byte buffer
uint8_t bufferLength = BUF_LEN;
uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

bool ledState = false;
unsigned int sendCounter = 0, lastSendCount = 0, sendRate = 0;
unsigned int lastSendMillis = 0, sendInterval = 100; // send every 100 milliseconds

unsigned int seconds = 0, lastSeconds = 0;
unsigned int loopCounter = 0, lastLoopCounter = 0;
unsigned int receiveCounter = 0, lastReceiveCount = 0, receiveRate = 0, receiveErrorCount = 0, incompleteReceiveCount = 0, sendErrorCount = 0;
unsigned int receivedBytesErrorCount = 0;

volatile bool i2cDataReady = false;
volatile unsigned int  bytesAvailable = 0, bytesRequested = 0;

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
        uint8_t inverted = (uint8_t)~i;
        if (in_buf[i] != inverted) {
            receivedBytesErrorCount++;
            if (success && printOnlyFirstError) {
                printf("ERROR! page: %07u First Error at index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, inverted, in_buf[i]);
            } else if (!printOnlyFirstError) {
                printf("ERROR! page: %07u index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, inverted, in_buf[i]);
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

int sendBufferToSlave(uint8_t length) {
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        printf("I2C Sender says: Sending Output buffer to Receiver (slave Pico)...  (page %u, buffer size: %03u) \r\n", sendCounter, length);
    }
    gpio_put(DEBUG_PIN3, 0);
    // First send the data length of the buffer so the i2c slave knows what to expect next
    // This seems to just hang after a failed i2c_read_timeout_us during readBufferFromSlave()
    int byteCount = i2c_write_blocking(I2C_INSTANCE, I2C_SLAVE_ADDRESS, &length, 1, false);
    if (byteCount < 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            printf("I2C Sender says: ERROR!!! Could not send Output buffer page %u, return value: %d Couldn't write to i2c slave, please check your wiring! \r\n", sendCounter, byteCount);
        }
        gpio_put(DEBUG_PIN3, 1);
        return 0;
    }
    // Now send the buffer
    byteCount = i2c_write_blocking(I2C_INSTANCE, I2C_SLAVE_ADDRESS, out_buf, length, false);
    if (byteCount < 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            printf("I2C Sender says: ERROR!!! Could not send Output buffer page %u, return value: %d Couldn't write to i2c slave, please check your wiring! \r\n", sendCounter, byteCount);
        }
        gpio_put(DEBUG_PIN3, 1);
        return 0;
    }
    gpio_put(DEBUG_PIN3, 1);
    sendCounter++;
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        if (DEBUG_SERIAL_OUTPUT_DURING_I2C_READ) {
            // If USB Serial is sent, it may still be on its way out of a FIFO while the code continues from from here onto the I2C read operation
            printf("I2C Sender says: Output buffer page %u sent, buffer size: %03u \r\n", sendCounter, length);
        }
    }
    return byteCount;
}

int readBufferFromSlave(uint8_t length) {
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        printf("I2C Sender says: Requesting buffer page %u from Receiver (slave Pico) ...  (requesting buffer size: %03u) \r\n", sendCounter, length);
    }
    bytesAvailable = 0;
    gpio_put(DEBUG_PIN2, 0);
    // First write the command to the Receiver indicating the size of the buffer we want to read (length), set timeout to 10 ms
    int byteCount = i2c_write_timeout_us(I2C_INSTANCE, I2C_SLAVE_ADDRESS, &length, 1, true, 1000 * 10); // Important that "nostop" is set to true here for a Request operation
    // A negative return value from the above function indicates an error:
    // Return values, from SDK: PICO_OK = 0, PICO_ERROR_NONE = 0, PICO_ERROR_TIMEOUT = -1 , PICO_ERROR_GENERIC = -2 , PICO_ERROR_NO_DATA = -3 , PICO_ERROR_NOT_PERMITTED = -4 , PICO_ERROR_INVALID_ARG = -5 , PICO_ERROR_IO = -6
    if (byteCount < 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            printf("I2C Sender says: ERROR!!! Could not write the command to Request buffer page %u, return value: %d (Couldn't read from i2c slave, please check your wiring!) \r\n", sendCounter, byteCount);
        }
        gpio_put(DEBUG_PIN2, 1);
        return 0;
    }

    // Now read the buffer that the Receiver (slave) should be sending back in response to the read Request
    byteCount = i2c_read_timeout_us(I2C_INSTANCE, I2C_SLAVE_ADDRESS, in_buf, length, false, 1000 * 10); // set timeout to 10 ms
    if (byteCount < 0) {
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            printf("I2C Sender says: ERROR!!! Could not read the Request for buffer page %u, return value: %d (Couldn't read from i2c slave, please check your wiring!) \r\n", sendCounter, byteCount);
        }
        gpio_put(DEBUG_PIN2, 1);
        return 0;
    }
    gpio_put(DEBUG_PIN2, 1);
    bytesAvailable = byteCount;
    i2cDataReady = true;

    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        printf("I2C Sender says: Buffer page %u read from the Receiver (slave Pico), received buffer size: %03u expected: %03u  \r\n", sendCounter, byteCount, length);
    }
    return byteCount;
}

void resetAllRxVars() {
    i2cDataReady = false;
    bytesAvailable = 0;
}

static void setupMaster() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    // pull-ups are already active on slave side, this is just a fail-safe in case the wiring is faulty
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(I2C_INSTANCE, I2C_BAUDRATE);
}

int main() {
    // Enable UART so we can print
    stdio_init_all();

    int startupDelay = 10;
    for (int i = 1; i <= startupDelay; ++i) {
        printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    printf("\e[2J\e[H"); // clear screen and go to home position

    printf("I2C Sender example using i2c baud rate: %d \r\n", I2C_BAUDRATE);
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
    setupMaster();
    gpio_put(DEBUG_PIN3, 1); // signal the end of I2C config

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // The values should be: {0x01, 0x02, 0x03...}
        out_buf[i] = i + 1;
    }
    clearBuffer(in_buf, BUF_LEN);

    printf("I2C Sender says: The value: 0x%02X (%u) (buffer size) followed immediately by the buffer printed below will be sent to the receiver every: %u ms\r\n", BUF_LEN, BUF_LEN, sendInterval);
    printBuffer(out_buf, BUF_LEN);
    printf("\r\n");
    printf("The value 0x%02X (%u) is expected to be returned followed by a reversed version of the above buffer\r\n\r\n", BUF_LEN, BUF_LEN);

    unsigned long startMillis = to_ms_since_boot(get_absolute_time());
    unsigned long currentMillis = 0;

    // Loop for ever...
    for (size_t i = 0; ; ++i) {
        loopCounter++;
        // Keep track of seconds since start
        currentMillis = to_ms_since_boot(get_absolute_time());
        seconds = (currentMillis - startMillis) / 1000;
        if (seconds - lastSeconds > 0) {
            lastSeconds = seconds;
            // calculate the loop rate, per second
            lastLoopCounter = loopCounter;
            loopCounter = 0;
            // calculate the send rate, per second
            sendRate = sendCounter - lastSendCount;
            lastSendCount = sendCounter;
            //calculate the receive rate, per second
            receiveRate = receiveCounter - lastReceiveCount;
            lastReceiveCount = receiveCounter;
        }

        // Send data every sendInterval
        if (currentMillis >= lastSendMillis + sendInterval) {
            lastSendMillis = currentMillis;
            gpio_put(LED_BUILTIN, 1); // turn on the LED
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                if ((receiveCounter + incompleteReceiveCount) < sendCounter) {
                    incompleteReceiveCount++;
                    printf("ERROR!!! The page %u response was incomplete!!! bytesRequested: %03u and bytesAvailable: %03u should equal the Buffer Length: %03u\r\n", sendCounter, bytesRequested, bytesAvailable, BUF_LEN);
                    printBuffer(in_buf, BUF_LEN);
                    printf("I2C Sender says: ERROR!!! Received incomplete buffer!!! (printed above) \r\n");
                    bool verifySuccess = verifyInBuffer(sendCounter, true);
                    if (!verifySuccess) receiveErrorCount++;
                    clearBuffer(in_buf, BUF_LEN);
                    resetAllRxVars(); // recover from this anomaly
                }
                // Reset the previous terminal position if we are not scrolling the output
                if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                    printf("\e[H"); // move to the home position, at the upper left of the screen
                    printf("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
                }
                // Print the header info
                printf("\r\nSeconds: %07u.%03u       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
                printf("LoopRate: %07u            \r\n", lastLoopCounter); // how many loops per second
                printf("sendCounter: %07u         \r\n", sendCounter);
                printf("sendRate: %07u            \r\n", sendRate);
                printf("Send errorCount: %03u         \r\n", sendErrorCount);
                printf("Send FailureRate: %11.7f percent  \r\n", 100.0f * sendErrorCount / (sendCounter > 0 ? sendCounter : 1));
                printf("receiveCounter: %07u         \r\n", receiveCounter);
                printf("receiveRate: %07u            \r\n", receiveRate);
                printf("Receive errorCount: %03u             \r\n", receiveErrorCount);
                printf("Receive FailureRate: %11.7f percent  \r\n", 100.0f * receiveErrorCount / (sendCounter > 0 ? sendCounter : 1));
                printf("Receive incompleteReceiveCount: %03u         \r\n", incompleteReceiveCount);
                printf("receivedBytesErrorCount: %03u         \r\n", receivedBytesErrorCount);
            }
            // Send the Buffer to the Receiver
            int bytesSent = sendBufferToSlave(BUF_LEN);
            if (bytesSent < BUF_LEN) {
                sendErrorCount++;
            }

            // Request the buffer from the Receiver
            bytesRequested = BUF_LEN;
            int bytesReceived = readBufferFromSlave(bytesRequested);
            if (bytesReceived != bytesRequested) {
                //receiveErrorCount++; // do this below instead
            }
            // The results are checked below, although they could be checked here instead.

            gpio_put(LED_BUILTIN, 0); // turn off the LED
        }

        // Check if we have all the expected data from the i2c read request
        if (i2cDataReady) {
            receiveCounter++;
            gpio_put(DEBUG_PIN3, 0);
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                // Write the input buffer out to the USB serial port
                printBuffer(in_buf, BUF_LEN);
                printf("I2C Sender says: Verifying received data...                                         \r\n");
                if (bytesAvailable != BUF_LEN) {
                    receiveErrorCount++;
                    printf("ERROR!!! page: %u bytesAvailable: %03u should equal the Buffer Length: %03u\r\n", receiveCounter, bytesAvailable, BUF_LEN);
                }
                bool verifySuccess = verifyInBuffer(receiveCounter, false);
                // Check that we only record the error once for each receive cycle
                if (bytesAvailable == BUF_LEN && !verifySuccess) receiveErrorCount++;
            }
            clearBuffer(in_buf, BUF_LEN);
            i2cDataReady = false;
            bytesAvailable = 0;
            gpio_put(DEBUG_PIN3, 1);
        }
    }
}
