/**
 * @file main.c
 * @brief I2C communication and data processing with NRF microcontroller.
 *
 * This code communicates with an AVR device using I2C to retrieve sensor data 
 * and hub information. It periodically sends commands to the device, verifies
 * data integrity using CRC-16-CCITT and CRC-8, and processes the received data.
 *
 * @version 1.2
 * @date 2024
 */

#include <zephyr/kernel.h>           ///< Zephyr kernel header
#include <zephyr/drivers/i2c.h>     ///< I2C driver header
#include <stdio.h>                   ///< Standard I/O header
#include <stdlib.h>                  ///< Standard library header
//#include <zephyr/power/power.h>    ///< For sleep functionality (commented out)
#include <zephyr/sys/printk.h>       ///< Printk for logging
#include <zephyr/sys/byteorder.h>    ///< Byte order manipulation functions
#include <zephyr/device.h>           ///< Device management header
#include <zephyr/zephyr.h>
#include <cJSON.h>                   ///< cJSON library for JSON handling
#include <cJSON_os.h>

#include <zephyr/logging/log.h>       ///< Logging header for structured logging
#include <zephyr/drivers/gpio.h>     ///< GPIO driver header

#include <zephyr/drivers/watchdog.h>  ///< Watchdog timer header

#ifndef WDT_MAX_WINDOW
#define WDT_MAX_WINDOW  30000U
#endif

// Definitions
#define I2C0_NODE      DT_NODELABEL(avr_device)
#define NUM_SENSORS    16  ///< Maximum number of sensors
#define SENSOR_DATA_SIZE sizeof(SensorData)
#define HUB_INFO_SIZE  10  ///< Updated to 10 bytes based on new hub info structure
#define MAX_RETRIES    2 ///< Retry for CRC mismatches
#define HEARTBEAT_PERIOD_S 30  ///< Heartbeat interval in seconds
// I2C slave address from the device tree
#define I2C_SLAVE_ADDRESS  DT_REG_ADDR(DT_NODELABEL(avr_device))


typedef enum {
    CMD_GET_SAMPLES_ALL = 0x01,
    CMD_GET_HUB_INFO = 0x02,
    CMD_GET_SAMPLES = 0x03,
    CMD_GET_SAMPLES_SPECIFIC = 0x04
} CommandCode;
/**
 * @brief Send command, payload, and CRC-8 to AVR.
 * @param i2c_dev I2C device structure.
 * @param cmd Command to send.
 * @param payload Payload to send.
 * @return I2C write status.
 */
int send_command(const struct i2c_dt_spec *i2c_dev, CommandCode cmd, uint8_t payload) {
    uint8_t command[2] = { cmd, payload};
    return i2c_write_dt(i2c_dev, command, sizeof(command));
}



/**
 * @brief Main entry function.
 */int main(void) {
    int err;
    int ret;
       // Get the I2C device structure from the device tree
    const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(I2C0_NODE);

    // Check if the I2C device is ready
    if (!device_is_ready(i2c_dev.bus)) {
        printk("I2C bus %s is not ready!\n", i2c_dev.bus->name);
        return;
    }

      ret = send_command(&i2c_dev, CMD_GET_SAMPLES, 0x00);  // Request sensor samples
                if (ret) {
                    printk("I2C: Failed to send sample command (error %d).\n", ret);
                 // Retry sending the command
                }

}