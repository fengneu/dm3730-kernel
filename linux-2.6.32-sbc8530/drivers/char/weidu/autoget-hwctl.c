/*
 * File: brdcfg.c
 * Author: JasperZhang
 * Modify:
 * 	20170603			Create			Jasper
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/gpio.h>

#include <linux/init.h>
#include <linux/genhd.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/atomic.h>


/* ioctl cmd */
#define HWC_IOR_FEPE 	_IOR('A', 1, int)
#define HWC_IOR_BEPE 	_IOR('A', 2, int)
#define HWC_IOR_DSSW 	_IOR('A', 3, int)

#define HWC_IOW_MOTORST 	_IOW('A', 1, int)
#define HWC_IOW_MCUNOTIFY 	_IOW('A', 2, int)

/* gpio pins */
#define GPIO_IN_FPESW		73		/* Frontend photoelectricity switch */
#define GPIO_IN_BPESW		74		/* Backend photoelectricity switch */
#define GPIO_IN_DSSW		113		/* RGB led & LCD display switch */

#define GPIO_OUT_MOTORST	133		/* moto reset */
#define GPIO_OUT_MCUNOTIFY	161		/* moto reset */

static ssize_t autoget_hwctl_read(struct file *filp, char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static ssize_t autoget_hwctl_write(struct file *filp,const char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static int autoget_hwctl_ioctl(struct inode * inode, struct file * file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u32	tmp;
	int	retval = 0;
	u8	val = 0;

	switch (cmd) {
	case HWC_IOR_FEPE:
		val = gpio_get_value(GPIO_IN_FPESW);
		retval = __put_user(val, (__u8 __user *)argp);
		break;

	case HWC_IOR_BEPE:
		val = gpio_get_value(GPIO_IN_BPESW);
		retval = __put_user(val, (__u8 __user *)argp);
		break;

	case HWC_IOR_DSSW:
		val = gpio_get_value(GPIO_IN_DSSW);
		retval = __put_user(val, (__u8 __user *)argp);
		break;

	case HWC_IOW_MOTORST:
		retval = __get_user(tmp, (u8 __user *)argp);
		if (retval == 0) {
			if(tmp)
				gpio_set_value(GPIO_OUT_MOTORST, 1);
			else
				gpio_set_value(GPIO_OUT_MOTORST, 0);
		}
		break;

	case HWC_IOW_MCUNOTIFY:
		retval = __get_user(tmp, (u8 __user *)argp);
		if (retval == 0) {
			if(tmp)
				gpio_set_value(GPIO_OUT_MCUNOTIFY, 1);
			else
				gpio_set_value(GPIO_OUT_MCUNOTIFY, 0);
		}
		break;

	default:
		printk("Error hwctrl commad, please check it(%d)! \n", cmd);
		return -EIO;
	}
	return retval;
}

static int autoget_hwctl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int autoget_hwctl_close(struct inode * inode, struct file * file)
{
	return 0;
}


static ssize_t moto_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int moto_reset = simple_strtoul(buf, NULL, 10);

	if(moto_reset){
		gpio_set_value(GPIO_OUT_MOTORST, 1);
	}else{
		gpio_set_value(GPIO_OUT_MOTORST, 0);
	}

	return size;
}

static ssize_t mcu_notify_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int mcu_notify = simple_strtoul(buf, NULL, 10);

	if(mcu_notify){
		gpio_set_value(GPIO_OUT_MCUNOTIFY, 1);
	}else{
		gpio_set_value(GPIO_OUT_MCUNOTIFY, 0);
	}

	return size;
}


struct file_operations autoget_hwctl_fops = {
	.read		= autoget_hwctl_read,
	.write		= autoget_hwctl_write,
	.ioctl		= autoget_hwctl_ioctl,
	.open		= autoget_hwctl_open,
	.release	= autoget_hwctl_close,
};

static struct miscdevice autoget_hwctl_dev = {
	.minor         = MISC_DYNAMIC_MINOR,
	.name         = "autoget_hwctl",
	.fops         = &autoget_hwctl_fops,
};

static DEVICE_ATTR(moto_reset, S_IWUSR, NULL, moto_reset_store);
static DEVICE_ATTR(mcu_notify, S_IWUSR, NULL, mcu_notify_store);


static struct attribute *autoget_hwctl_attributes[] = {
	&dev_attr_moto_reset.attr,
	&dev_attr_mcu_notify.attr,
	NULL,
};

static struct attribute_group autoget_hwctl_attr_group = {
	.attrs = autoget_hwctl_attributes,
};

static int autoget_gpio_init(void)
{
	int ret = 0;

	/* photoelectricity switch */
	ret = gpio_request(GPIO_IN_FPESW, "fepe switch");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for fepe switch\n",
				GPIO_IN_FPESW);
		return ret;
	}
	gpio_direction_input(GPIO_IN_FPESW);

	ret = gpio_request(GPIO_IN_BPESW, "fepe switch");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for fepe switch\n",
				GPIO_IN_BPESW);
		return ret;
	}
	gpio_direction_input(GPIO_IN_BPESW);

	/* rgb-led and lcd switch */
	ret = gpio_request(GPIO_IN_DSSW, "fepe switch");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for fepe switch\n",
				GPIO_IN_DSSW);
		return ret;
	}
	gpio_direction_input(GPIO_IN_DSSW);

	/* moto reset */
	ret = gpio_request(GPIO_OUT_MOTORST, "moto reset");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for moto reset\n",
				GPIO_OUT_MOTORST);
		return ret;
	}
	gpio_direction_output(GPIO_OUT_MOTORST, 1);

	/* moto reset */
	ret = gpio_request(GPIO_OUT_MCUNOTIFY, "mcu notify");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for mcu notify\n",
				GPIO_OUT_MCUNOTIFY);
		return ret;
	}
	gpio_direction_output(GPIO_OUT_MCUNOTIFY, 0);

	return 0;
}


static int __init autoget_hwctl_init(void)
{
	int ret;

	ret = autoget_gpio_init();
	if(ret) {
		printk(KERN_ERR "gpio init failed\n");
		return ret;
	}
	ret = misc_register(&autoget_hwctl_dev);
	if(ret) {
		printk(KERN_ERR "misc_register failed\n");
		return ret;
	}

	ret = sysfs_create_group(&autoget_hwctl_dev.this_device->kobj, &autoget_hwctl_attr_group);
        if (ret) {
		printk(KERN_ERR "creat attr file failed\n");
		misc_deregister(&autoget_hwctl_dev);
		return ret;
	}

	return 0;
}

static void __exit autoget_hwctl_exit(void)
{
	sysfs_remove_group(&autoget_hwctl_dev.this_device->kobj, &autoget_hwctl_attr_group);
	misc_deregister(&autoget_hwctl_dev);
}

module_init (autoget_hwctl_init);
module_exit (autoget_hwctl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("JasperZhang");
MODULE_DESCRIPTION("Character device for autoget  hwctl");


