/*
 * tc358743_h2c_irq_dev.c
 *
 *  Created on: Jul 24, 2016
 *      Author: ezerbib
 */

#include "mxc_v4l2_capture.h"
#include "tc358743_h2c.h"

#ifdef CONFIG_TC358743_DEV
#include "tc358743_h2c_irq_dev.h"
#endif

static const struct file_operations tc358743_irq_fops = {
	.owner       = THIS_MODULE,
	.poll        = tc358743_irq_poll,
	.unlocked_ioctl = tc358743_irq_ioctl,
	.read        = tc358743_irq_read,
	.write       = tc358743_irq_write,
	.open        = tc358743_irq_open,
	.release     = tc358743_irq_release,
};

//static TC358743_INT_REG IntData = {};
static struct cdev *c_dev;
static char tc358743_irq_name[] = "tc358743_irq";
static struct tc358743_irq_private *alarmlist=NULL;
static DEFINE_SPINLOCK(alarm_lock);
//static DEFINE_SPINLOCK(gpio_lock);

static DEFINE_MUTEX(tc358743_irq_mutex);

void tc358743_irq_setup(void)
{
	dev_t first;
	struct class *cl;

	if (alloc_chrdev_region(&first, 0, 1, tc358743_irq_name) < 0)  //$cat /proc/devices
	{
		return;
	}

	c_dev = cdev_alloc();
    c_dev->ops=&tc358743_irq_fops;
    c_dev->owner=THIS_MODULE;

    cdev_init(c_dev, &tc358743_irq_fops);

	if (cdev_add(c_dev, first, 1) == -1)
	{
		unregister_chrdev_region(first, 1);
		return;
	}

	if ((cl = class_create(THIS_MODULE, "chardrv")) == NULL)    //$ls /sys/class
	{
		unregister_chrdev_region(first, 1);
		return;
	}

	if (device_create(cl, NULL, first, NULL, "tc358743") == NULL) //$ls /dev/
	{
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return;
	}


	pr_info("tc358743: device /dev/tc358743 created\n");

}

static unsigned int tc358743_irq_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct tc358743_irq_private *priv = (struct tc358743_irq_private *)file->private_data;
	//unsigned long data = 0;
	poll_wait(file, &priv->alarm_wq, wait);

	if (priv->minor == TC358743_DEV_MINOR_A)
	{
		if (priv->event_count != atomic_read(&priv->event))
				return POLLIN | POLLRDNORM;
	}
	else
	{
		return 0;
	}
	return mask;

}

static long
tc358743_irq_ioctl(struct file *file,
	   unsigned int cmd, unsigned long arg)
{
	struct tc358743_irq_private *priv = (struct tc358743_irq_private *)file->private_data;
	if (_IOC_TYPE(cmd) != TC358743_DEV_IOCTYPE)
		return -EINVAL;

	switch (_IOC_NR(cmd))
	{
		case IO_READBITS: /* Use IO_READ_INBITS and IO_READ_OUTBITS instead */
			{
				/* Read the register interrupt port. */
				/** TODO: return TC358743 Hdmi registers ...*/
				u32 RegVal = 0;
				s32 retval;
				struct tc_data *td;
				td = tc358743_get_tc_data();
				mutex_lock(&td->access_lock);
				retval = tc358743_read_reg(priv->sensor, arg, &RegVal);
				mutex_unlock(&td->access_lock);
				if (retval < 0) {
					pr_err("%s: read failed, reg=0x%lx\n", __func__, arg);
					return -EINVAL;
				}
				return -EINVAL;
			}
			break;

		case IO_SETBITS:
			{
			}
			break;

		case IO_HIGHALARM:
			{
			return -EINVAL;
			}
			break;

		case IO_LOWALARM:
			{
			return -EINVAL;
			}
			break;
		default:
				return -EINVAL;
	} /* switch */

	return 0;
}

static ssize_t tc358743_irq_read(struct file *file, char __user *buf,
				size_t len, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	ssize_t retval;
	struct tc358743_irq_private *devp;
	s32 event_count;
	//printk(KERN_DEBUG "tc358743_irq_read: before read: \n");
	devp = file->private_data;

	if (len != sizeof(s32))
		return -EINVAL;


	add_wait_queue(&devp->alarm_wq, &wait);

	for ( ; ; ) {
		set_current_state(TASK_INTERRUPTIBLE);

		event_count = atomic_read(&devp->event);
		if (event_count != devp->event_count) {
			//printk(KERN_DEBUG "tc358743_irq_read: ev=%d : dev=%d\n",event_count,devp->event_count);
			if (copy_to_user(buf, &event_count, len))
			{
				retval = -EFAULT;
				printk(KERN_DEBUG "tc358743_irq_read: retval = -EFAULT;\n");
			}
			else {
				devp->event_count = event_count;
				retval = len;
			}
			break;

		}

		if (file->f_flags & O_NONBLOCK)
		{
			retval = -EAGAIN;
			goto out;
		}
		else if (signal_pending(current))
		{
			retval = -ERESTARTSYS;
			goto out;
		}
		schedule();
	}

	//printk(KERN_DEBUG "tc358743_irq_read: after read: \n");

out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&devp->alarm_wq, &wait);

	return retval;
}

static ssize_t tc358743_irq_write(struct file *file, const char *buf, size_t count,
	loff_t *off)
{
	return -EINVAL;
}

static int tc358743_irq_open(struct inode *inode, struct file *filp)
{
	struct tc_data *td = tc358743_get_tc_data();
	struct sensor_data *sensor = &td->sensor;

	struct tc358743_irq_private *priv;

	priv = kmalloc(sizeof(struct tc358743_irq_private), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_lock(&tc358743_irq_mutex);
	memset(priv, 0, sizeof(*priv));

	priv->minor = TC358743_DEV_MINOR_A;

	/* initialize the io/alarm struct */
	init_waitqueue_head(&priv->alarm_wq);

	priv->event_count = atomic_read(&priv->event);
	filp->private_data = (void *)priv;
	td->tc_irq_priv=priv;

	priv->rbufhead = 0;
	priv->rbufcnt = 0;
	priv->flags = TC358743_INT_FLAGS_DEV_OPEN;
	priv->sensor = sensor;

	/* link it into our alarmlist */
	spin_lock_irq(&alarm_lock);
	priv->next = alarmlist;
	alarmlist = priv;
	spin_unlock_irq(&alarm_lock);

#if 0
	if(0!=install_irq(GPIO_IRQn,GPIO_IRQHandler,NORMAL_PRIORITY))
	{
		pr_err("lpc178x:  failed to install gpio irqs\n");
	}
#endif

	mutex_unlock(&tc358743_irq_mutex);
	return 0;
}

static int
tc358743_irq_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct tc358743_irq_private *p;
	struct tc358743_irq_private *todel;
	/* local copies while updating them: */
	unsigned long some_alarms;
	struct tc_data *td = tc358743_get_tc_data();

	/* unlink from alarmlist and free the private structure */

	spin_lock_irq(&alarm_lock);
	p = alarmlist;
	todel = (struct tc358743_irq_private *)filp->private_data;

	if (p == todel) {
		alarmlist = todel->next;
	} else {
		while (p->next != todel)
			p = p->next;
		p->next = todel->next;
	}

	/* Check if there are still any alarms set */
	p = alarmlist;
	some_alarms = 0;
	while (p) {
		if (p->minor == TC358743_DEV_MINOR_A)
		{
			some_alarms = 1;
		}
		p = p->next;
	}

	if (some_alarms==0)
	{
		td->tc_irq_priv=NULL;
	}

	kfree(todel);

	spin_unlock_irq(&alarm_lock);

	return ret;
}

