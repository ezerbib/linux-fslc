/*
 * tc358743_h2c_irq_dev.h
 *
 *  Created on: Jul 24, 2016
 *      Author: ezerbib
 */

#ifndef DRIVERS_MEDIA_PLATFORM_MXC_CAPTURE_TC358743_H2C_IRQ_DEV_H_
#define DRIVERS_MEDIA_PLATFORM_MXC_CAPTURE_TC358743_H2C_IRQ_DEV_H_

///////////////////////////////////////////////////////////////////////////////////////////////////
#define TC358743_DEV_MINOR_A 0
#define TC358743_DEV_LAST 5

#define TC358743_DEV_IOCTYPE 44
/* supported ioctl _IOC_NR's */

#define IO_READBITS  0x1  /* read and return current port bits (obsolete) */
#define IO_SETBITS   0x2  /* set the bits marked by 1 in the argument */
#define IO_CLRBITS   0x3  /* clear the bits marked by 1 in the argument */

/* the alarm is waited for by select() */

#define IO_HIGHALARM 0x4  /* set alarm on high for bits marked by 1 */
#define IO_LOWALARM  0x5  /* set alarm on low for bits marked by 1 */
#define IO_CLRALARM  0x6  /* clear alarm for bits marked by 1 */


/* GPIO direction ioctl's */
#define IO_READDIR    0x8  /* Read direction 0=input 1=output  (obsolete) */
#define IO_SETINPUT   0x9  /* Set direction for bits set, 0=unchanged 1=input,
                              returns mask with current inputs (obsolete) */
#define IO_SETOUTPUT  0xA  /* Set direction for bits set, 0=unchanged 1=output,
                              returns mask with current outputs (obsolete)*/


/* SHUTDOWN ioctl */
#define IO_SHUTDOWN   0xD
#define IO_GET_PWR_BT 0xE

/* Bit toggling in driver settings */
/* bit set in low byte0 is CLK mask (0x00FF),
   bit set in byte1 is DATA mask    (0xFF00)
   msb, data_mask[7:0] , clk_mask[7:0]
 */
#define IO_CFG_WRITE_MODE 0xF
#define IO_CFG_WRITE_MODE_VALUE(msb, data_mask, clk_mask) \
	( (((msb)&1) << 16) | (((data_mask) &0xFF) << 8) | ((clk_mask) & 0xFF) )

/* The following 4 ioctl's take a pointer as argument and handles
 * 32 bit ports (port G) properly.
 * These replaces IO_READBITS,IO_SETINPUT AND IO_SETOUTPUT
 */
#define IO_READ_INBITS   0x10 /* *arg is result of reading the input pins */
#define IO_READ_OUTBITS  0x11 /* *arg is result of reading the output shadow */
#define IO_SETGET_INPUT  0x12 /* bits set in *arg is set to input,
                               * *arg updated with current input pins.
                               */
#define IO_SETGET_OUTPUT 0x13 /* bits set in *arg is set to output,
                               * *arg updated with current output pins.
                               */
typedef struct
{
	uint32_t Reg_8520;
} TC358743_INT_REG;

extern void tc358743_irq_setup(void);
//static int tc358743_irq_ioctl(struct inode *inode, struct file *file,
//	unsigned int cmd, unsigned long arg);

static long tc358743_irq_ioctl (struct file *, unsigned int cmd, unsigned long arg);
static ssize_t tc358743_irq_read(struct file *file, char *buf, size_t len, loff_t *ppos);
static ssize_t tc358743_irq_write(struct file *file, const char *buf, size_t count,
	loff_t *off);
static int tc358743_irq_open(struct inode *inode, struct file *filp);
static int tc358743_irq_release(struct inode *inode, struct file *filp);
static unsigned int tc358743_irq_poll(struct file *file, struct poll_table_struct *wait);

/* private data per open() of this driver */
struct tc358743_irq_private {
	struct tc358743_irq_private     *next;
	struct semaphore                sem;
	//char							*rbufp;		/* read buffer for I/O */
	int								rbufhead;
	int								rbufcnt;
	int								flags;
	TC358743_INT_REG 				intalarm;
	struct sensor_data              *sensor;
	/* These fields are generic */
	wait_queue_head_t               alarm_wq;
	atomic_t						event;
	s32 							event_count;
	int                             minor;
};

#define	READ_BUFFER_SIZE	0x10000		// 64k
#define	MAX_REQ_PACKET_SIZE	0x10000		// 64k
#define	RETRY_TIMEOUT		(HZ)
#define	MAX_WRITE_RETRY		5

#define TC358743_INT_FLAGS_DEV_OPEN		0x01
#define TC358743_INT_FLAGS_RX_BUSY			0x02
#define TC358743_INT_FLAGS_INTR_BUSY		0x04
#define TC358743_INT_FLAGS_DEV_ERROR		0x08
#define TC358743_INT_FLAGS_PENDING_CLEANUP	0x10
#define TC358743_INT_FLAGS_MASK			0x1f
/* linked list of alarms to check for */





#endif /* DRIVERS_MEDIA_PLATFORM_MXC_CAPTURE_TC358743_H2C_IRQ_DEV_H_ */
