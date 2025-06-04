#define _FDC2214_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "FDC2214.h"
#include <nrfx_twim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "twim_inst.h"
#include "twim2_inst.h"
#include "storage.h"
#include "versa_ble.h"
#include "versa_time.h"
#include "versa_config.h"
#include "app_data.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(FDC2214, LOG_LEVEL_INF);

// FDC2214 storage format header
#define FDC2214_STORAGE_HEADER 0x3333

#define FDC2214_PRINT_VAL

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Function to handle the FDC2214 thread
 * 
 * @param arg1 A pointer to the first argument passed to the thread.
 * @param arg2 A pointer to the second argument passed to the thread.
 * @param arg3 A pointer to the third argument passed to the thread.
 */
void FDC2214_thread_func(void *arg1, void *arg2, void *arg3);


/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! I2C Instance pointer*/
static nrfx_twim_t *I2cInstancePtr;
static nrfx_twim_t *I2c2InstancePtr;

// TX buffer
uint8_t tx_buffer_fdc[MAX_SIZE_TRANSFER + 1];

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(FDC2214_thread_stack, 1024);
struct k_thread FDC2214_thread;

/*! Structure to store capacitance data */
FDC2214_StorageFormat FDC2214_Storage;

// Initialization Variables
int sensor0_init_successful = 0;
int sensor1_init_successful = 0;
int sensor2_init_successful = 0;
int sensor3_init_successful = 0;


/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

uint16_t FDC2214_swap_endianness(uint16_t value) {
    return (value << 8) | (value >> 8);
}

int FDC2214_read_8bit(uint8_t addr, uint8_t *data, uint8_t sensor_addr, uint8_t i2c_bus_identifier){
    if (i2c_bus_identifier == 0){
        // Take the I2C semaphore
        k_sem_take(&I2C_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = addr;
        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(sensor_addr, tx_buffer_fdc, 1, data, 1);
        nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

        // Give back the I2C semaphore
        k_sem_give(&I2C_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    } else {
        // Take the I2C semaphore
        k_sem_take(&I2C2_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = addr;
        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(sensor_addr, tx_buffer_fdc, 1, data, 1);
        nrfx_err_t err = nrfx_twim_xfer(I2c2InstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2c2InstancePtr) == true){k_sleep(K_USEC(10));}

        // Give back the I2C semaphore
        k_sem_give(&I2C2_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    }
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_read_16bit(uint8_t start_address, uint16_t *data, uint8_t sensor_addr, uint8_t i2c_bus_identifier)
{
    if (i2c_bus_identifier == 0){
        // Take the I2C semaphore
        k_sem_take(&I2C_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = start_address;
        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(sensor_addr, tx_buffer_fdc, 1, (uint8_t *)data, 2);
        nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

        *data = FDC2214_swap_endianness(*data);

        // Give back the I2C semaphore
        k_sem_give(&I2C_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    } else {
        // Take the I2C semaphore
        k_sem_take(&I2C2_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = start_address;
        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(sensor_addr, tx_buffer_fdc, 1, (uint8_t *)data, 2);
        nrfx_err_t err = nrfx_twim_xfer(I2c2InstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2c2InstancePtr) == true){k_sleep(K_USEC(10));}

        *data = FDC2214_swap_endianness(*data);

        // Give back the I2C semaphore
        k_sem_give(&I2C2_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    }
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_write_8bit(uint8_t addr, uint8_t data, uint8_t sensor_addr, uint8_t i2c_bus_identifier){
    if (i2c_bus_identifier == 0){
        // Take the I2C semaphore
        k_sem_take(&I2C_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = addr;
        tx_buffer_fdc[1] = data;
        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(sensor_addr, tx_buffer_fdc, 2);
        nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

        // Give back the I2C semaphore
        k_sem_give(&I2C_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    } else {
        // Take the I2C semaphore
        k_sem_take(&I2C2_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = addr;
        tx_buffer_fdc[1] = data;
        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(sensor_addr, tx_buffer_fdc, 2);
        nrfx_err_t err = nrfx_twim_xfer(I2c2InstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2c2InstancePtr) == true){k_sleep(K_USEC(10));}

        // Give back the I2C semaphore
        k_sem_give(&I2C2_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    }
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_write_16bit(uint8_t start_address, uint16_t data, uint8_t sensor_addr, uint8_t i2c_bus_identifier)
{
    if (i2c_bus_identifier == 0){
        // Take the I2C semaphore
        k_sem_take(&I2C_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = start_address;

        // Fill the buffer with data, LSB first
        tx_buffer_fdc[1] = data >> 8;    // data[i] & 0xFF;  // LSB
        tx_buffer_fdc[2] = data & 0xFF;  // data[i] >> 8;    // MSB

        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(sensor_addr, tx_buffer_fdc, 2 + 1);
        nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

        // Give back the I2C semaphore
        k_sem_give(&I2C_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    } else {
        // Take the I2C semaphore
        k_sem_take(&I2C2_sem, K_FOREVER);

        // Perform the I2C transfer
        tx_buffer_fdc[0] = start_address;

        // Fill the buffer with data, LSB first
        tx_buffer_fdc[1] = data >> 8;    // data[i] & 0xFF;  // LSB
        tx_buffer_fdc[2] = data & 0xFF;  // data[i] >> 8;    // MSB

        nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(sensor_addr, tx_buffer_fdc, 2 + 1);
        nrfx_err_t err = nrfx_twim_xfer(I2c2InstancePtr, &xfer, 0);

        // Wait for the transfer to finish
        while(nrfx_twim_is_busy(I2c2InstancePtr) == true){k_sleep(K_USEC(10));}

        // Give back the I2C semaphore
        k_sem_give(&I2C2_sem);

        return err == NRFX_SUCCESS ? 0 : -1;
    }
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_init(void){
    // Configure GPIO Pins: Shutdown (SD) = LOW
    nrf_gpio_cfg_output(32);
    nrf_gpio_pin_clear(32); //set reset pin to low

    nrf_gpio_cfg_output(7);
    nrf_gpio_pin_clear(7); //set reset pin to low

    nrf_gpio_cfg_output(46);
    nrf_gpio_pin_clear(46); //set reset pin to low

    nrf_gpio_cfg_output(35);
    nrf_gpio_pin_clear(35); //set reset pin to low

    // Get the I2C instances
    nrfx_twim_t *I2cInstPtr = twim_get_instance();
    I2cInstancePtr=I2cInstPtr;

    nrfx_twim_t *I2c2InstPtr = twim2_get_instance();
    I2c2InstancePtr=I2c2InstPtr;

    // Instantiate the sensor
    FDC_2214 csb_sensor_0 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR1,
                            .sensor_id = 0,
                            .i2c_bus_id = 0};

    FDC_2214 csb_sensor_1 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR2,
                            .sensor_id = 1,
                            .i2c_bus_id = 0};
    
    FDC_2214 csb_sensor_2 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR1,
                            .sensor_id = 2,
                            .i2c_bus_id = 1};

    FDC_2214 csb_sensor_3 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR2,
                            .sensor_id = 3,
                            .i2c_bus_id = 1};

    k_msleep(4000);
    printk("Herenow\n");
    k_msleep(3000);

    uint16_t data_read = 0;

    // ----------------------------- INITIALIZE FDC CORE #0 ------------------------------
    int res = FDC2214_read_16bit(REG_FDC_DEVICE_ID, &data_read, csb_sensor_0.sensor_address, csb_sensor_0.i2c_bus_id);
    if (res != 0){
        printk("Error reading FDC Sensor 0\n");
    }

    if (data_read == FDC_DEVICE_ID){
        printk("FDC2214 #0 Initialized Successfully!\n");
        sensor0_init_successful = 1;
        // Configure the sensor
        FDC2214_configure(&csb_sensor_0, csb_sensor_0.i2c_bus_id);
    } else {
        printk("FDC2214 #0 Initialization Failed\n");
        //return -1;
    }

    // ----------------------------- INITIALIZE FDC CORE #1 ------------------------------
    res = FDC2214_read_16bit(REG_FDC_DEVICE_ID, &data_read, csb_sensor_1.sensor_address, csb_sensor_1.i2c_bus_id);
    if (res != 0){
        printk("Error reading FDC Sensor 1\n");
    }

    if (data_read == FDC_DEVICE_ID){
        printk("FDC2214 #1 Initialized Successfully!\n");
        sensor1_init_successful = 1;
        // Configure the sensor
        FDC2214_configure(&csb_sensor_1, csb_sensor_1.i2c_bus_id);
    } else {
        printk("FDC2214 #1 Initialization Failed\n");
        //return -1;
    }
    
    // ----------------------------- INITIALIZE FDC CORE #2 ------------------------------
    res = FDC2214_read_16bit(REG_FDC_DEVICE_ID, &data_read, csb_sensor_2.sensor_address, csb_sensor_2.i2c_bus_id);
    if (res != 0){
        printk("Error reading FDC Sensor 2\n");
    }

    if (data_read == FDC_DEVICE_ID){
        printk("FDC2214 #2 Initialized Successfully!\n");
        sensor2_init_successful = 1;
        // Configure the sensor
        FDC2214_configure(&csb_sensor_2, csb_sensor_2.i2c_bus_id);
    } else {
        printk("FDC2214 #2 Initialization Failed\n");
        //return -1;
    }

    // ----------------------------- INITIALIZE FDC CORE #3 ------------------------------
    res = FDC2214_read_16bit(REG_FDC_DEVICE_ID, &data_read, csb_sensor_3.sensor_address, csb_sensor_3.i2c_bus_id);
    if (res != 0){
        printk("Error reading FDC Sensor 3\n");
    }

    if (data_read == FDC_DEVICE_ID){
        printk("FDC2214 #3 Initialized Successfully!\n");
        sensor3_init_successful = 1;
        // Configure the sensor
        FDC2214_configure(&csb_sensor_3, csb_sensor_3.i2c_bus_id);
    } else {
        printk("FDC2214 #3 Initialization Failed\n");
        //return -1;
    }
    
    // ---------------------------- Start the FDC2214 thread ----------------------------
    k_msleep(200);
    k_thread_create(&FDC2214_thread, FDC2214_thread_stack, K_THREAD_STACK_SIZEOF(FDC2214_thread_stack),
                    FDC2214_thread_func, NULL, NULL, NULL, FDC2214_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&FDC2214_thread, "FDC2214_thread");

    LOG_INF("Versa API FDC2214 thread started\n");
    return 0;
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_configure(FDC_2214 *dev, uint8_t i2c_bus_identifier)
{
    uint16_t data2write;
    int status = 0;
    int config_successful = 0;

    // Configure CH0
    status = 0;
    if (dev->channel_mask & 0x01) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH0, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH0, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH0, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH0, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH0, data2write, dev->sensor_address, i2c_bus_identifier);
    }
    if (status != 0){
        printk("Error configuring CH%i\n", 0 + dev->sensor_id*4);
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 0 + dev->sensor_id*4);
    }

    // Configure CH1
    status = 0;
    if (dev->channel_mask & 0x02) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH1, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH1, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH1, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH1, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH1, data2write, dev->sensor_address, i2c_bus_identifier);
    }
    if (status != 0){
        printk("Error configuring CH%i\n", 1 + dev->sensor_id*4);
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 1 + dev->sensor_id*4);
    }

    // Configure CH2
    status = 0;
    if (dev->channel_mask & 0x04) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH2, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH2, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH2, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH2, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH2, data2write, dev->sensor_address, i2c_bus_identifier);
    }
    if (status != 0){
        printk("Error configuring CH%i\n", 2 + dev->sensor_id*4);
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 2 + dev->sensor_id*4);
    }

    // Configure CH3
    status = 0;
    if (dev->channel_mask & 0x08) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH3, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH3, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH3, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH3, data2write, dev->sensor_address, i2c_bus_identifier);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH3, data2write, dev->sensor_address, i2c_bus_identifier);
    }
    if (status != 0){
        printk("Error configuring CH%i\n", 3 + dev->sensor_id*4);
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 3 + dev->sensor_id*4);
    }

    // Configure ?????
    status = 0;
    uint8_t rr_se = 0x06;
    uint8_t glitch_filter = 0x05; // 10 MHz input deglitch filter
    uint16_t mux = 0x0208 | ((uint16_t)rr_se << 13) | glitch_filter;
    status |= FDC2214_write_16bit(REG_FDC_MUX_CONFIG, mux, dev->sensor_address, i2c_bus_identifier);
    if (status != 0){
        printk("Error configuring MUX!\n");
        config_successful |= -1;
    }

    // Use External Oscillator for Reference
    data2write = 0x1E81;
    status |= FDC2214_write_16bit(REG_FDC_CONFIG, data2write, dev->sensor_address, i2c_bus_identifier); 
    if (status != 0){
        printk("Error configuring CONFIG!\n");
        config_successful |= -1;
    }

    if (config_successful != 0){
        printk("FDC2214 Configuration Failed!\n");
        return -1;
    } else {
        printk("FDC2214 Configured Succesfully!\n");
        return 0;
    }
    
}

/*****************************************************************************
*****************************************************************************/

uint32_t FDC2214_get_values(FDC_2214 *dev, uint8_t channel_id, uint8_t i2c_bus_identifier){
    /*
    First 4 LSB bits of "STATUS" register at 0x18:
    Channel n Unread Conversion
    b0: No unread conversion is present for Channel n.
    b1: An unread conversion is present for Channel n.
    */
    uint8_t add_MSB, add_LSB;
    uint8_t unread_conv;
    uint32_t cap_value = 0;

    switch (channel_id) {
        case 0:
            add_MSB = REG_FDC_DATA_CH0_MSB;
            add_LSB = REG_FDC_DATA_CH0_LSB;
            unread_conv = 0x08;
            break;
        case 1:
            add_MSB = REG_FDC_DATA_CH1_MSB;
            add_LSB = REG_FDC_DATA_CH1_LSB;
            unread_conv = 0x04;
            break;
        case 2:
            add_MSB = REG_FDC_DATA_CH2_MSB;
            add_LSB = REG_FDC_DATA_CH2_LSB;
            unread_conv = 0x02;
            break;
        case 3:
            add_MSB = REG_FDC_DATA_CH3_MSB;
            add_LSB = REG_FDC_DATA_CH3_LSB;
            unread_conv = 0x01;
            break;
        default:
            return 0;
    }

    uint16_t conv_status = 0; //read_Cap(REG_FDC_STATUS, addr);
    int res = FDC2214_read_16bit(REG_FDC_STATUS, &conv_status, dev->sensor_address, i2c_bus_identifier);
    
    while (!(conv_status & unread_conv)) {
        if (res != 0){
            printk("An error occured while reading reg %x\n", REG_FDC_DEVICE_ID);
            return -1;
        }
        //printk("id: %i | CONV_STATUS: %x\n", channel_id, conv_status);
        FDC2214_read_16bit(REG_FDC_STATUS, &conv_status, dev->sensor_address, i2c_bus_identifier);
    }

    uint16_t msb_value = 0;
    uint16_t lsb_value = 0;
    res = 0;
    res |= FDC2214_read_16bit(add_MSB, &msb_value, dev->sensor_address, i2c_bus_identifier);
    cap_value = ((uint32_t)(msb_value & 0x0FFF) << 16);
    res |= FDC2214_read_16bit(add_LSB, &lsb_value, dev->sensor_address, i2c_bus_identifier);
    cap_value |= lsb_value;

    if (res != 0){
        printk("An error occured while reading reg %x\n", REG_FDC_DEVICE_ID);
        return -1;
    }

    return cap_value;
}

/*****************************************************************************
*****************************************************************************/

void FDC2214_thread_func(void *arg1, void *arg2, void *arg3)
{
    // Instantiate again the sensor
    // This is a workaround temporarily for the fact that I cannot pass pointer of pointer as an argument to the thread
    FDC_2214 csb_sensor_0 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR1,
                            .sensor_id = 0,
                            .i2c_bus_id = 0};

    FDC_2214 csb_sensor_1 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR2,
                            .sensor_id = 1,
                            .i2c_bus_id = 0};
    
    FDC_2214 csb_sensor_2 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR1,
                            .sensor_id = 2,
                            .i2c_bus_id = 1};
    
    FDC_2214 csb_sensor_3 = {.channel_mask = 0xF, 
                            .sampling_rate = 0x4FFF, // 30 Hertz
                            .sensor_address = FDC_DEVICE_ADDR2,
                            .sensor_id = 3,
                            .i2c_bus_id = 1};
    
    const int SENSOR_COUNT = 4;
    const int CHAN_COUNT = 4;
    uint32_t FDC_values[SENSOR_COUNT * CHAN_COUNT];
    memset(FDC_values, 0, SENSOR_COUNT * CHAN_COUNT);

    FDC2214_Storage.header = FDC2214_STORAGE_HEADER;       /*!< Storage header marker */
    uint8_t frame_index = 0;                            /*!< Frame sequence index */

    FDC_2214 sensor_array[4] = {csb_sensor_0, csb_sensor_1, csb_sensor_2, csb_sensor_3};
    int init_success_array[4] = {sensor0_init_successful, sensor1_init_successful, sensor2_init_successful, sensor3_init_successful};

    while (1){
        for (int i = 0; i < SENSOR_COUNT; ++i){
            struct time_values current_time = get_time_values();
            int16_t time_started = current_time.time_ms_bin;
            for (int j = 0; j < CHAN_COUNT; ++j) {
                if (init_success_array[i]){
                    FDC_values[i*CHAN_COUNT + j] = FDC2214_get_values(&sensor_array[i], j, sensor_array[i].i2c_bus_id);
                } else {
                    FDC_values[i*CHAN_COUNT + j] = 0;
                }

                #ifdef FDC2214_PRINT_VAL
                printk("%lu",FDC_values[i*CHAN_COUNT + j]);
                if (i*CHAN_COUNT + j < SENSOR_COUNT * CHAN_COUNT - 1){
                    printk(",");
                } else {
                    printk("\n");
                }
                #endif
            }
        }

        struct time_values current_time = get_time_values();
        FDC2214_Storage.rawtime_bin = current_time.rawtime_s_bin;
        FDC2214_Storage.time_ms_bin = current_time.time_ms_bin;

        /* Store encoded data in storage format structure */
        FDC2214_Storage.len = 4;
        FDC2214_Storage.index = frame_index++;
        FDC2214_Storage.CH0_val = FDC_values[0];
        FDC2214_Storage.CH1_val = FDC_values[1];
        FDC2214_Storage.CH2_val = FDC_values[2];
        FDC2214_Storage.CH3_val = FDC_values[3];
        FDC2214_Storage.CH4_val = FDC_values[4];
        FDC2214_Storage.CH5_val = FDC_values[5];
        FDC2214_Storage.CH6_val = FDC_values[6];
        FDC2214_Storage.CH7_val = FDC_values[7];
        FDC2214_Storage.CH8_val = FDC_values[8];
        FDC2214_Storage.CH9_val = FDC_values[9];
        FDC2214_Storage.CH10_val = FDC_values[10];
        FDC2214_Storage.CH11_val = FDC_values[11];
        FDC2214_Storage.CH12_val = FDC_values[12];
        FDC2214_Storage.CH13_val = FDC_values[13];
        FDC2214_Storage.CH14_val = FDC_values[14];
        FDC2214_Storage.CH15_val = FDC_values[15];

        storage_add_to_fifo((uint8_t *)&FDC2214_Storage, sizeof(FDC2214_Storage));
        ble_add_to_fifo((uint8_t *)&FDC2214_Storage, sizeof(FDC2214_Storage));
        printk("data sent\n");
        //k_sleep(K_MSEC(200));

    }
}