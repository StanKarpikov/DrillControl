#ifndef _I2C_DRV_H
#define _I2C_DRV_H

#include <iostm8.h>
#include <intrinsics.h>

#define F_MASTER_MHZ    16UL
#define F_MASTER_HZ     16000000UL

//I2C operation result
typedef enum {
    I2C_SUCCESS = 0,
    I2C_TIMEOUT,
    I2C_ERROR
} t_i2c_status;

// I2C interface initialisation
extern void i2c_master_init(unsigned long f_master_hz, unsigned long f_i2c_hz);

// I2C register write
extern t_i2c_status  i2c_wr_reg(unsigned char address, 
                               unsigned char reg_addr,
                               char * data, 
                               unsigned char length);

// I2C regster read
extern t_i2c_status  i2c_rd_reg(unsigned char address, 
                               unsigned char reg_addr,
                               char * data, 
                               unsigned char length);
extern t_i2c_status i2c_wr_data(unsigned char address, char * data);
#endif