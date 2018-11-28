/*

Improving the kernel timers API:
https://lwn.net/Articles/735887/

    struct timer_list {
    unsigned long       expires;
    void            (*function)(unsigned long);
    unsigned long       data;
    }

The expires field contains the expiration time of the timer (in jiffies); on
expiration, function() will be called with the given data value. It is possible
to fill in a timer_list structure manually, but it is more common to use the
setup_timer() macro:
    void setup_timer(timer, function, data);

*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/poll.h>
#include <linux/sched/signal.h>

#include <linux/timer.h>

#include "spi_timer.h"

struct timer_list timer;



#define READ_CMD (0x00)
#define WRITE_CMD (0x01)
#define MAJOR_NUM (501)
#define BUTTON1_GPIO (115)
#define DEBOUNCE_TIME (200)
#define TRUE (1)
#define FALSE (0)
static int spidev_probe(struct spi_device *spi);
static int spidev_remove(struct spi_device *spi);
static inline ssize_t spi_dev_sync_write(size_t len);
static int is_data;
static u32 mode;

static int irqNumber;
static const struct of_device_id spidev_dt_ids[] = {
    {.compatible = "rohm,dh2228fv"},
    {.compatible = "lineartechnology,ltc2488"},
    {.compatible = "ge,achc"},
    {.compatible = "semtech,sx1301"},
    {.compatible = "spi_dev"},
    {},
};

MODULE_DEVICE_TABLE(of, spidev_dt_ids);



/* The next line is equivalent to: static wait_queue_head_t gpio_waitqueue;*/
static DECLARE_WAIT_QUEUE_HEAD(spi_waitqueue);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *  is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK       (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)


struct spi_data {

    dev_t devt;
    spinlock_t spi_lock;
    struct spi_device *spi;
    struct mutex buf_lock;
    unsigned users;
    u8 *tx_buffer;
    u8 *rx_buffer;
    u32 speed_hz;

};

static struct spi_driver spidev_spi_driver = {
    .driver = {
	       .name = "spi_dev",
	       .of_match_table = of_match_ptr(spidev_dt_ids),
	       },
    .probe = spidev_probe,
    .remove = spidev_remove,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

static unsigned bufsize = (4096);

struct spi_data *spi_dev;


static long spi_dev_ioctl(struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
    int retval = 0;
    struct spi_device *spi;
    u32 tmp;

    /* defined in spi.h */
    spi = spi_dev_get(spi_dev->spi);

    if (spi == NULL)
	return -ESHUTDOWN;

    switch (cmd) {

    case MY_MODE_WR:

	*spi_dev->tx_buffer = 0xBB;
	retval = spi_dev_sync_write(1);
	retval = get_user(tmp, (__u8 __user *) arg);
	mode = tmp;
	*spi_dev->tx_buffer = tmp;
	printk(KERN_ALERT "arg = %u", tmp);
	retval = spi_dev_sync_write(1);
	break;
    case MY_MODE_RD:
	retval = put_user(mode, (__u32 __user *) arg);
	break;

	/* read requests */
    case SPI_IOC_RD_MODE:
	retval = put_user(spi->mode & SPI_MODE_MASK, (__u8 __user *) arg);
	break;
    case SPI_IOC_RD_MODE32:
	retval = put_user(spi->mode & SPI_MODE_MASK, (__u32 __user *) arg);
	break;
    case SPI_IOC_RD_LSB_FIRST:
	retval = put_user((spi->mode & SPI_LSB_FIRST) ? 1 : 0,
			  (__u8 __user *) arg);
	break;
    case SPI_IOC_RD_BITS_PER_WORD:
	retval = put_user(spi->bits_per_word, (__u8 __user *) arg);
	break;
    case SPI_IOC_RD_MAX_SPEED_HZ:
	retval = put_user(spi_dev->speed_hz, (__u32 __user *) arg);
	break;

	/* write requests */
    case SPI_IOC_WR_MODE:
    case SPI_IOC_WR_MODE32:
	if (cmd == SPI_IOC_WR_MODE)
	    retval = get_user(tmp, (u8 __user *) arg);
	else
	    retval = get_user(tmp, (u32 __user *) arg);
	if (retval == 0) {
	    u32 save = spi->mode;

	    if (tmp & ~SPI_MODE_MASK) {
		retval = -EINVAL;
		break;
	    }

	    tmp |= spi->mode & ~SPI_MODE_MASK;
	    spi->mode = (u16) tmp;
	    retval = spi_setup(spi);
	    if (retval < 0)
		spi->mode = save;
	    else
		dev_dbg(&spi->dev, "spi mode %x\n", tmp);
	}
	break;
    case SPI_IOC_WR_LSB_FIRST:
	retval = get_user(tmp, (__u8 __user *) arg);
	if (retval == 0) {
	    u32 save = spi->mode;

	    if (tmp)
		spi->mode |= SPI_LSB_FIRST;
	    else
		spi->mode &= ~SPI_LSB_FIRST;
	    retval = spi_setup(spi);
	    if (retval < 0)
		spi->mode = save;
	    else
		dev_dbg(&spi->dev, "%csb first\n", tmp ? 'l' : 'm');
	}
	break;
    case SPI_IOC_WR_BITS_PER_WORD:
	retval = get_user(tmp, (__u8 __user *) arg);
	if (retval == 0) {
	    u8 save = spi->bits_per_word;

	    spi->bits_per_word = tmp;
	    retval = spi_setup(spi);
	    if (retval < 0)
		spi->bits_per_word = save;
	    else
		dev_dbg(&spi->dev, "%d bits per word\n", tmp);
	}
	break;
    case SPI_IOC_WR_MAX_SPEED_HZ:
	retval = get_user(tmp, (__u32 __user *) arg);
	if (retval == 0) {
	    u32 save = spi->max_speed_hz;

	    spi->max_speed_hz = tmp;
	    retval = spi_setup(spi);
	    if (retval >= 0)
		spi_dev->speed_hz = tmp;
	    else
		dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
	    spi->max_speed_hz = save;
	}
	break;

    default:

	break;
    }
    spi_dev_put(spi);
    return retval;

}


static ssize_t spi_dev_sync(struct spi_message *message)
{

    int status;
    struct spi_device *spi;
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);
    spi = spi_dev->spi;
    if (spi == NULL)
	status = -ESHUTDOWN;
    else
	status = spi_sync(spi, message);

    if (status == 0)
	status = message->actual_length;
    printk(KERN_ALERT "Leaving the %s function\n", __FUNCTION__);
    return status;
}


static inline ssize_t spi_dev_sync_write(size_t len)
{

    struct spi_transfer transfer = {

	.tx_buf = spi_dev->tx_buffer,
	.len = len,
	.speed_hz = spi_dev->speed_hz,
    };

    struct spi_message message;
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);
    spi_message_init(&message);
    spi_message_add_tail(&transfer, &message);

    return spi_dev_sync(&message);
}

static ssize_t spi_dev_write(struct file *filp, const char __user * buf,
			     size_t count, loff_t * f_pos)
{

    ssize_t status = 0;


    unsigned long missing;
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);

    /* specify write to arduino */
    *spi_dev->tx_buffer = WRITE_CMD;
    status = spi_dev_sync_write(1);

    if (count > bufsize)
	return -EMSGSIZE;

    missing = copy_from_user(spi_dev->tx_buffer, buf, count);

    if (missing == 0)
	status = spi_dev_sync_write(count);
    else
	status = -EFAULT;

    printk(KERN_ALERT "leaving the %s function\n", __FUNCTION__);

    return status;


}

static inline ssize_t spi_dev_sync_read(size_t len)
{

    struct spi_transfer transfer = {

	.rx_buf = spi_dev->rx_buffer,
	.len = len,
	.speed_hz = spi_dev->speed_hz,

    };

    struct spi_message message;
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);
    spi_message_init(&message);

    /*
     * Adds message to the back of the queue.
     * The queue is located in struct transfer -> transfer_list
     */

    spi_message_add_tail(&transfer, &message);

    printk(KERN_ALERT "leaving the %s function\n", __FUNCTION__);

    return spi_dev_sync(&message);

}


static ssize_t spi_dev_read(struct file *filp, char __user * buf,
			    size_t count, loff_t * f_pos)
{
    ssize_t status;
    DECLARE_WAITQUEUE(wait, current);

    /* specify read to arduino */
    *spi_dev->tx_buffer = READ_CMD;
    status = spi_dev_sync_write(1);

    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);
    add_wait_queue(&spi_waitqueue, &wait);


    do {

	set_current_state(TASK_INTERRUPTIBLE);
	status = spi_dev_sync_read(1);
	if (status != 0)
	    break;

	if (filp->f_flags & O_NONBLOCK) {
	    status = -EAGAIN;
	    goto out;
	}
	if (signal_pending(current)) {
	    status = -ERESTARTSYS;
	    goto out;
	}
	schedule();
    } while (1);

    /* spi_dev gets assigned the spi_data struct we declared in spi_dev_open() */

    if (status > 0) {
	status = copy_to_user(buf, spi_dev->rx_buffer, status);
    }

  out:
    set_current_state(TASK_RUNNING);
    remove_wait_queue(&spi_waitqueue, &wait);
    printk(KERN_ALERT "leaving the %s function\n", __FUNCTION__);
    is_data = FALSE;

    return status;
}


static int spi_dev_release(struct inode *inode, struct file *filp)
{
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);
    kfree(spi_dev->tx_buffer);
    kfree(spi_dev->rx_buffer);
    printk(KERN_ALERT "leaving the %s function\n", __FUNCTION__);


    return 0;
}


static int spi_dev_open(struct inode *inode, struct file *filp)
{
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);


    spi_dev->tx_buffer = kmalloc(bufsize, GFP_KERNEL);
    if (!spi_dev->tx_buffer) {
	dev_dbg(&spi_dev->spi->dev, "open/ENOMEM\n");
	return -ENOMEM;
    }

    spi_dev->rx_buffer = kmalloc(bufsize, GFP_KERNEL);
    if (!spi_dev->rx_buffer) {
	dev_dbg(&spi_dev->spi->dev, "open/ENOMEM\n");
	kfree(spi_dev->tx_buffer);
	return -ENOMEM;

    }

    printk(KERN_INFO "open complete\n");

    return 0;

}

static irqreturn_t irq_handler(unsigned int irq, void *dev_id,
			       struct pt_regs *regs)
{

    printk(KERN_INFO "Interrupt!\n");
    is_data = TRUE;
    /* Wake up processes in interruptible sleep */
    wake_up_interruptible(&spi_waitqueue);

    return IRQ_HANDLED;
}


static unsigned int spi_poll(struct file *file, poll_table * wait)
{
    int c;
    printk(KERN_INFO "File %p about to wait.\n", file);
    poll_wait(file, &spi_waitqueue, wait);
    printk(KERN_INFO "File %p done waiting.\n", file);
    c = (is_data != FALSE) ? (POLLIN | POLLRDNORM) : FALSE;
    return c;
}



static int spidev_probe(struct spi_device *spi)
{

    int status = 0;

    if (spi->dev.of_node && !of_match_device(spidev_dt_ids, &spi->dev)) {
	dev_err(&spi->dev, "buggy DT: spidev listed directly in DT\n");
	WARN_ON(spi->dev.of_node &&
		!of_match_device(spidev_dt_ids, &spi->dev));
    }

    printk(KERN_ALERT "sizeof= %d\n", sizeof(*spi_dev));
    spi_dev = kzalloc(sizeof(*spi_dev), GFP_KERNEL);
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);

    /* spidev is of type struct spi_data */
    /* spidev->spi is of type spi_device (from spi.h) */
    spi_dev->spi = spi;
    spi_dev->speed_hz = spi->max_speed_hz;
    spi_set_drvdata(spi, spi_dev);
    printk(KERN_ALERT "Leaving the %s function\n", __FUNCTION__);
    return status;
}

static int spidev_remove(struct spi_device *spi)
{
    printk(KERN_ALERT "Inside the %s function\n", __FUNCTION__);
    spi_unregister_driver(&spidev_spi_driver);
    kfree(spi_dev);
    printk(KERN_ALERT "Leaving the %s function\n", __FUNCTION__);
    return 0;
}



/*******************************************************/






static const struct file_operations spi_dev_fops = {
    .owner = THIS_MODULE,
    .write = spi_dev_write,
    .read = spi_dev_read,
    /*
     * unlocked_ioctl lets the writer chose what lock to use instead
     * of the BKL (big kernel lock )
     */
    .unlocked_ioctl = spi_dev_ioctl,
    .open = spi_dev_open,
    .poll = spi_poll,


    /* 
     * The release method is called when the last reference to the device is removed  
     * All device structures registed with the core must have a release method
     * LDD3-pg 382
     */
    .release = spi_dev_release,
};


static int __init spi_dev_init(void)
{

    int status;
    unsigned long IRQflags;
    IRQflags = IRQF_TRIGGER_RISING;
    mode = DEFAULT_MODE;
    is_data = FALSE;

    status = register_chrdev(MAJOR_NUM, "spi_dev", &spi_dev_fops);

    /* Probe is connected to internal spi overall probe */
    if (status < 0) {
	printk(KERN_ALERT "ERROR WITH INIT\n");
	unregister_chrdev(MAJOR_NUM, spidev_spi_driver.driver.name);
	return status;
    }
    if (gpio_request(BUTTON1_GPIO, "button1_gpio") ||
	gpio_set_debounce(BUTTON1_GPIO, DEBOUNCE_TIME) ||
	gpio_direction_input(BUTTON1_GPIO)) {
	return -1;
    }


    /* Interrupt used to indicate the presense of data */

    irqNumber = gpio_to_irq(BUTTON1_GPIO);
    status = request_irq(irqNumber,
			 (irq_handler_t) irq_handler,
			 IRQflags, "button_handler", NULL);

    status = spi_register_driver(&spidev_spi_driver);

    return status;
}

static void __exit spi_dev_exit(void)
{
    printk(KERN_ALERT "Inside the %s function!!\n", __FUNCTION__);
    free_irq(irqNumber, NULL);
    gpio_free(BUTTON1_GPIO);
    unregister_chrdev(MAJOR_NUM, "spi_dev");

    printk(KERN_INFO "Exited\n");
}

module_init(spi_dev_init);
module_exit(spi_dev_exit);


MODULE_AUTHOR("gt");
MODULE_DESCRIPTION
    ("General Purpose master/slave communication between beaglebone and arduino");
MODULE_LICENSE("GPL");
