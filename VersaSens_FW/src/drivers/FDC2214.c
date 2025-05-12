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
#include "storage.h"
#include "versa_ble.h"
#include "versa_time.h"
#include "versa_config.h"
#include "app_data.h"

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! I2C Instance pointer*/
static nrfx_twim_t *I2cInstancePtr;

// TX buffer
uint8_t tx_buffer_fdc[MAX_SIZE_TRANSFER + 1];



/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

uint16_t FDC2214_swap_endianness(uint16_t value) {
    return (value << 8) | (value >> 8);
}

int FDC2214_read_8bit(uint8_t addr, uint8_t *data, uint8_t sensor_addr){
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
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_read_16bit(uint8_t start_address, uint16_t *data, size_t num_words, uint8_t sensor_addr)
{
    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_fdc[0] = start_address;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(sensor_addr, tx_buffer_fdc, 1, (uint8_t *)data, num_words*2);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);
    
    *data = FDC2214_swap_endianness(*data);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_write_8bit(uint8_t addr, uint8_t data, uint8_t sensor_addr){
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
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_write_16bit(uint8_t start_address, uint16_t *data, size_t num_words, uint8_t sensor_addr)
{
    if (num_words > MAX_SIZE_TRANSFER/2) {
        return -1;
    }

    // Take the I2C semaphore
    k_sem_take(&I2C_sem, K_FOREVER);

    // Perform the I2C transfer
    tx_buffer_fdc[0] = start_address;

    // Fill the buffer with data, LSB first
    for (size_t i = 0; i < num_words; i++) {
        tx_buffer_fdc[i*2 + 1] = data[i] & 0xFF;  // LSB
        tx_buffer_fdc[i*2 + 2] = data[i] >> 8;    // MSB
    }

    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(sensor_addr, tx_buffer_fdc, num_words*2 + 1);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    // Wait for the transfer to finish
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    // Give back the I2C semaphore
    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int FDC2214_configure(FDC_2214 *dev)
{
    uint16_t data2write;
    int status = 0;
    int config_successful = 0;

    // Use External Oscillator for Reference
    data2write = 0x1E81;
    status |= FDC2214_write_16bit(REG_FDC_CONFIG, &data2write, 1, dev->sensor_address); 
    if (status != 0){
        printk("Error writing to FDC2214!\n");
        config_successful |= -1;
    }

    // Configure CH0
    status = 0;
    if (dev->channel_mask & 0x01) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH0, &data2write, 1, dev->sensor_address);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH0, &data2write, 1, dev->sensor_address);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH0, &data2write, 1, dev->sensor_address);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH0, &data2write, 1, dev->sensor_address);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH0, &data2write, 1, dev->sensor_address);
    }
    if (status != 0){
        printk("Error configuring CH0!\n");
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 0);
    }

    // Configure CH1
    status = 0;
    if (dev->channel_mask & 0x02) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH1, &data2write, 1, dev->sensor_address);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH1, &data2write, 1, dev->sensor_address);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH1, &data2write, 1, dev->sensor_address);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH1, &data2write, 1, dev->sensor_address);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH1, &data2write, 1, dev->sensor_address);
    }
    if (status != 0){
        printk("Error configuring CH1!\n");
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 1);
    }

    // Configure CH2
    status = 0;
    if (dev->channel_mask & 0x04) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH2, &data2write, 1, dev->sensor_address);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH2, &data2write, 1, dev->sensor_address);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH2, &data2write, 1, dev->sensor_address);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH2, &data2write, 1, dev->sensor_address);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH2, &data2write, 1, dev->sensor_address);
    }
    if (status != 0){
        printk("Error configuring CH2!\n");
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 2);
    }

    // Configure CH3
    status = 0;
    if (dev->channel_mask & 0x08) {
        data2write = 0x012C;
        status |= FDC2214_write_16bit(REG_FDC_SETTLECOUNT_CH3, &data2write, 1, dev->sensor_address);
        data2write = dev->sampling_rate;
        status |= FDC2214_write_16bit(REG_FDC_RCOUNT_CH3, &data2write, 1, dev->sensor_address);
        data2write = 0x0000;
        status |= FDC2214_write_16bit(REG_FDC_OFFSET_CH3, &data2write, 1, dev->sensor_address);
        data2write = 0x2001;
        status |= FDC2214_write_16bit(REG_FDC_CLOCK_DIVIDERS_CH3, &data2write, 1, dev->sensor_address);
        data2write = 0xF800;
        status |= FDC2214_write_16bit(REG_FDC_DRIVE_CH3, &data2write, 1, dev->sensor_address);
    }
    if (status != 0){
        printk("Error configuring CH3!\n");
        config_successful |= -1;
    } else {
        printk("Configured CH%i\n", 3);
    }

    // Configure ?????
    status = 0;
    uint8_t rr_se = 0x06;
    uint8_t glitch_filter = 0x05;
    uint16_t mux = 0x0208 | ((uint16_t)rr_se << 13) | glitch_filter;
    status |= FDC2214_write_16bit(REG_FDC_MUX_CONFIG, &mux, 1, dev->sensor_address);
    if (status != 0){
        printk("Error configuring MUX!\n");
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

int FDC2214_init(void){
    // Get the I2C instance
    nrfx_twim_t *I2cInstPtr = twim_get_instance();
    I2cInstancePtr=I2cInstPtr;

    // Instantiate the sensor
    FDC_2214 csb_sensor_0;
    csb_sensor_0.sensor_address = FDC_DEVICE_ADDR;
    csb_sensor_0.channel_mask = 0xF;
    csb_sensor_0.sampling_rate = 10; // Hertz

    k_msleep(4000);
    printk("Herenow\n");
    k_msleep(3000);

    uint16_t data_read = 0;
    int res = FDC2214_read_16bit(REG_FDC_DEVICE_ID, &data_read, 1, csb_sensor_0.sensor_address);
    if (res != 0){
        printk("Error reading FDC Sensor\n");
    }
    //printk("dataread: %x, expected: %x", data_read[0], FDC_DEVICE_ID);

    if (data_read == FDC_DEVICE_ID){
        printk("FDC2214 Initialized Successfully!\n");
    } else {
        printk("FDC2214 Initialization Failed\n");
        return -1;
    }

    // Configure the sensor
    FDC2214_configure(&csb_sensor_0);

    FDC2214_main_loop(&csb_sensor_0);
    
    return 0;
}

int FDC2214_get_values(FDC_2214 *dev, uint8_t channel_id){
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
    int res = FDC2214_read_16bit(REG_FDC_STATUS, &conv_status, 1, dev->sensor_address);
    
    while (!(conv_status & unread_conv)) {
        if (res != 0){
            printk("An error occured while reading reg %x\n", REG_FDC_DEVICE_ID);
            return -1;
        }
        //printk("id: %i | CONV_STATUS: %x\n", channel_id, conv_status);
        res = FDC2214_read_16bit(REG_FDC_STATUS, &conv_status, 1, dev->sensor_address);
    }

    uint16_t msb_value = 0;
    uint16_t lsb_value = 0;
    res = 0;
    res |= FDC2214_read_16bit(add_MSB, &msb_value, 1, dev->sensor_address);
    cap_value = ((uint32_t)(msb_value & 0x0FFF) << 16);
    res |= FDC2214_read_16bit(add_LSB, &lsb_value, 1, dev->sensor_address);
    cap_value |= lsb_value;

    if (res != 0){
        printk("An error occured while reading reg %x\n", REG_FDC_DEVICE_ID);
        return -1;
    }

    return cap_value;
}

int FDC2214_main_loop(FDC_2214 *dev){
    const int CHAN_COUNT = 4;  // or whatever your channel count is
    long capa[CHAN_COUNT];     // Use 'long' in C++ (equivalent to 'signed long')

    for (int i = 0; i < CHAN_COUNT; ++i) {
        printk("Getting values for CH %i:\n", i);
        capa[i] = FDC2214_get_values(dev, i);
        //capa[i] = Cap.Read(i, 0x2A);  // Read from channel i with I2C address 0x2A
        printk(capa[i]);

        if (i < CHAN_COUNT - 1)
            printk(",");
        else
            printk("\n");  // change to Serial.println(""); for ending the line
    }
}