#ifndef _FDC2214_H
#define _FDC2214_H

#include <zephyr/types.h>
#include "twim_inst.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/*!Device Address*/
#define FDC_DEVICE_ADDR1                0x2A
#define FDC_DEVICE_ADDR2                0x2B

/*!I2C Configuration*/  
#define MAX_SIZE_TRANSFER               20  

/*!MAX77658 TWI pins*/  
#define MAX77658_SCL_PIN                47
#define MAX77658_SDA_PIN                33

/*!Register Values*/    
#define FDC_DEVICE_ID                   0x3055

/*!Global registers*/   
#define REG_FDC_DEVICE_ID               0x7F
#define REG_FDC_DATA_CH0_MSB            0x00
#define REG_FDC_DATA_CH0_LSB            0x01
#define REG_FDC_DATA_CH1_MSB            0x02
#define REG_FDC_DATA_CH1_LSB            0x03
#define REG_FDC_DATA_CH2_MSB            0x04
#define REG_FDC_DATA_CH2_LSB            0x05
#define REG_FDC_DATA_CH3_MSB            0x06
#define REG_FDC_DATA_CH3_LSB            0x07
#define REG_FDC_RCOUNT_CH0              0x08
#define REG_FDC_RCOUNT_CH1              0x09
#define REG_FDC_RCOUNT_CH2              0x0A
#define REG_FDC_RCOUNT_CH3              0x0B
#define REG_FDC_OFFSET_CH0              0x0C
#define REG_FDC_OFFSET_CH1              0x0D
#define REG_FDC_OFFSET_CH2              0x0E
#define REG_FDC_OFFSET_CH3              0x0F
#define REG_FDC_SETTLECOUNT_CH0         0x10
#define REG_FDC_SETTLECOUNT_CH1         0x11
#define REG_FDC_SETTLECOUNT_CH2         0x12
#define REG_FDC_SETTLECOUNT_CH3         0x13
#define REG_FDC_CLOCK_DIVIDERS_CH0      0x14
#define REG_FDC_CLOCK_DIVIDERS_CH1      0x15
#define REG_FDC_CLOCK_DIVIDERS_CH2      0x16
#define REG_FDC_CLOCK_DIVIDERS_CH3      0x17
#define REG_FDC_STATUS                  0x18
#define REG_FDC_CONFIG                  0x1A
#define REG_FDC_MUX_CONFIG              0x1B
#define REG_FDC_DRIVE_CH0               0x1E
#define REG_FDC_DRIVE_CH1               0x1F
#define REG_FDC_DRIVE_CH2               0x20
#define REG_FDC_DRIVE_CH3               0x21

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*! Format of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    uint8_t len;
    uint8_t index;
    uint32_t CH0_val;
    uint32_t CH1_val;
    uint32_t CH2_val;
    uint32_t CH3_val;
} __attribute__((packed)) FDC2214_StorageFormat;


typedef struct {
    uint8_t channel_mask;
    uint8_t sensor_address;
    uint16_t sampling_rate;
} FDC_2214;


/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

uint16_t FDC2214_swap_endianness(uint16_t value);

/**
 * @brief This function reads a register from the FDC2214 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading from a register
 *          on the FDC2214 device. The address of the register to read from is provided as a parameter,
 *          and the data read from the register is stored in a buffer also provided as a parameter.
 * 
 * @param[in]   addr      The address of the register to read from.
 * @param[out]  data      A pointer to the buffer where the read data should be stored.
 * @param[in]   reg_size  The size of the register to read from (not used in the current implementation).
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int FDC2214_read_8bit(uint8_t addr, uint8_t *data, uint8_t sensor_addr);

/**
 * @brief This function writes a byte of data to a register on the FDC2214 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a byte of data to a register
 *          on the FDC2214 device. The address of the register and the data to be written are provided as parameters.
 * 
 * @param[in]   addr  The address of the register to write to.
 * @param[in]   data  The data to write to the register.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int FDC2214_write_8bit(uint8_t addr, uint8_t data, uint8_t sensor_addr);

/**
 * @brief This function writes a sequence of 16-bit words to sequential registers on the FDC2214 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a sequence of 16-bit words to 
 *          sequential registers on the FDC2214 device. The start address of the registers and the data to be written 
 *          are provided as parameters.
 * 
 * @param[in]   I2cInstancePtr  A pointer to the TWIM instance to use for the I2C transfer.
 * @param[in]   start_address   The start address of the registers to write to.
 * @param[in]   data            A pointer to the data to be written.
 * @param[in]   num_words       The number of 16-bit words to write.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int FDC2214_write_16bit(uint8_t start_address, uint16_t data, uint8_t sensor_addr);

/**
 * @brief This function reads a sequence of 16-bit words from sequential registers on the FDC2214 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading a sequence of 16-bit words from 
 *          sequential registers on the FDC2214 device. The start address of the registers and the number of 16-bit words 
 *          to read are provided as parameters. The read data is stored in the buffer pointed to by the data parameter.
 * 
 * @param[in]   I2cInstancePtr  A pointer to the TWIM instance to use for the I2C transfer.
 * @param[in]   start_address   The start address of the registers to read from.
 * @param[out]  data            A pointer to a buffer where the read data will be stored.
 * @param[in]   num_words       The number of 16-bit words to read.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int FDC2214_read_16bit(uint8_t start_address, uint16_t *data, uint8_t sensor_addr);

int FDC2214_init(void);

int FDC2214_configure(FDC_2214 *dev);

uint32_t FDC2214_get_values(FDC_2214 *dev, uint8_t channel_id);


#endif /* _FDC2214_H */