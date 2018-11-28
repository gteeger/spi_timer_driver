#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <poll.h>
#include "my_spi_master.h"


#define TRUE (1)
#define FALSE (0)


#define WRITE_COMMAND (0)
#define READ_COMMAND (1)
#define MODE_CHANGE (2)
#define MAX_NUM_BITS 8
#define MAX_SPEED_HZ (500000)
#define TIMEOUT 5


static int keep_going;
static uint8_t bits = MAX_NUM_BITS;
static uint32_t mode;
static uint32_t speed = MAX_SPEED_HZ;
static uint8_t mode_choice = DEFAULT_MODE;



static void pabort(const char *s)
{
    perror(s);
    abort();
}

void my_function(int sig)
{
    keep_going = 0;
}


static void do_read(int fd)
{
    uint8_t buf;
    int status;
    status = read(fd, &buf, 1);
    if (status < 0) {
	perror("read");
	return;
    }

    printf("\n");
    printf("read(%2d): %02x,", status, buf);
    printf("\n");
}

static void do_write(int fd, uint8_t message)
{
    unsigned char buf;
    int status;
    buf = message;
    status = write(fd, &buf, 1);
    printf("\n");
    printf("write(%2d): %02x", status, buf);
    printf("\n");

    return;

}

static void dumpstat(const char *name, int fd)
{
    __u8 lsb, bits;
    __u32 mode, speed;

    if (ioctl(fd, SPI_IOC_RD_MODE32, &mode) < 0) {
	perror("SPI rd_mode");
	return;
    }
    if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0) {
	perror("SPI rd_lsb_fist");
	return;
    }
    if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
	perror("SPI bits_per_word");
	return;
    }
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
	perror("SPI max_speed_hz");
	return;
    }

    printf("%s: spi mode 0x%x, %d bits %sper word, %d Hz max\n",
	   name, mode, bits, lsb ? "(lsb first) " : "", speed);
}

int main(int argc, char **argv)
{
    int c;
    int fd;

    struct pollfd fds[1];


    int ret = 0;
    uint8_t write_msg = 0x00;

    int command_flag = DEFAULT_MODE;

    signal(SIGINT, my_function);
    mode = DEFAULT_MODE;
    const char *name = "/dev/SPI_DEV";
    const char *optstring = "w:rm:";
    while ((c = getopt(argc, argv, optstring)) != EOF) {

	switch (c) {

	case 'w':
	    command_flag = WRITE_COMMAND;
	    write_msg = (uint8_t) (atoi(optarg));
	    continue;
	case 'r':
	    command_flag = READ_COMMAND;
	    continue;
	case 'm':
	    command_flag = MODE_CHANGE;
	    mode_choice = (uint8_t) (atoi(optarg));
	    continue;
	case '?':
	  err:
	    fprintf(stderr,
		    "\nusage: %s [-w N] [-r] [-m N]\n-w: write [N: message]\n-r: read\n-m: mode change [N: MODES]\n",
		    argv[0]);
	    printf("\nMODES: 0 --> config_mode (default) \n");
	    printf("       1 --> button mode \n");
	    printf("       2 --> scratch mode \n\n");
	    return 1;
	}

    }


    fd = open(name, O_RDWR | O_NONBLOCK);

    /*
     * spi mode
     */
    ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
	pabort("can't set spi mode");

    ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
    if (ret == -1)
	pabort("can't get spi mode");




    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
	pabort("can't set max speed hz");

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
	pabort("can't get max speed hz");

    /*
     * bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
	pabort("can't set bits per word");

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
	pabort("can't get bits per word");



    printf("spi mode: 0x%x\n", mode);
    printf("bits per word: %d\n", bits);
    printf("max speed: %d Hz (%d KHz)\n", speed, speed / 1000);



    if (mode_choice == BUTTON_MODE) {
	keep_going = 1;
	while (keep_going) {
	    fds[0].fd = fd;
	    fds[0].events = POLLIN;

	    ret = poll(fds, 1, 5000);

	    if (ret == 0) {
		printf("timeout \n");
	    } else {

		printf("read: \n");
		do_read(fd);
	    }


	}


	return 0;
    }


    else if (command_flag == WRITE_COMMAND) {
	do_write(fd, write_msg);

    }

    else if (command_flag == READ_COMMAND) {
	do_read(fd);

    }

    else if (command_flag == MODE_CHANGE) {

	ret = ioctl(fd, MY_MODE_WR, &mode_choice);
	if (ret == -1)
	    pabort("can't set my mode");

	ret = ioctl(fd, MY_MODE_RD, &mode_choice);
	if (ret == -1)
	    pabort("can't get my mode");

	if (fd < 0) {
	    perror("open");
	    return 1;

	}
    }
    dumpstat(name, fd);




    close(fd);
    return 0;
}
