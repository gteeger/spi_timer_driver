#ifndef MY_SPI_MASTER_H
#define MY_SPI_MASTER_H

#include <linux/ioctl.h>
#include <linux/spi/spidev.h>


#define DEFAULT_MODE 0x00
#define BUTTON_MODE 0x01
#define SCRATCH_MODE 0x02

#define MY_MODE_WR _IOW(SPI_IOC_MAGIC, 6, __u8)
#define MY_MODE_RD _IOR(SPI_IOC_MAGIC, 6, __u8)

#endif
