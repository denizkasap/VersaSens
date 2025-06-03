/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : twim_inst.h                                                   **
** version  : 1                                                            **
** date     : DD/MM/21                                                     **
**                                                                         **
*****************************************************************************
**                                                                         **
** Copyright (c) EPFL                                                      **
** All rights reserved.                                                    **
**                                                                         **
*****************************************************************************
	 
VERSION HISTORY:
----------------
Version     : 1
Date        : DD/MM/YY
Revised by  : Benjamin Duc
Description : Original version.


*/

/***************************************************************************/
/***************************************************************************/

/**
* @file   twim_inst.h
* @date   DD/MM/YY
* @brief  This is the main header of twim_inst.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _TWIM2_INST_H
#define _TWIM2_INST_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

// #include "sdk_common.h"
#include <zephyr/types.h>
#include <nrfx_twim.h>
#include <zephyr/kernel.h>

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

#define TWIM2_SCL_PIN 29
#define TWIM2_SDA_PIN 30

#define TWIM2_INST_IDX 2

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

extern struct k_sem I2C2_sem;


/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _TWIM2_INST_C_SRC



#endif  /* _TWIM_INST_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief  This function returns the twim instance
 * 
 * @param  void
 * 
 * @return nrfx_twim_t* : twim instance 
 */
nrfx_twim_t * twim2_get_instance(void);


/**
 * @brief  This function initializes the twim instance
 * 
 * @param  void
 * 
 * @return int : 0 if successful, -1 otherwise
 */
int twim2_inst_init(void);

/**
 * @brief  This function waits for the twim transfer to finish
 * 
 * @param  void
 * 
 * @return void
 */
void wait_for_twim2_transfer(void);

/**
 * @brief  This function returns the status of the twim instance
 * 
 * @param  void
 * 
 * @return bool : true if busy, false otherwise
 */
bool twim2_is_busy(void);

/**
 * @brief  This function returns the status of the last twim transfer
 * 
 * @param  void
 * 
 * @return bool : true if successful, false otherwise
 */
bool twim2_transfer_succeeded(void);


/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _TWIM_INST_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/