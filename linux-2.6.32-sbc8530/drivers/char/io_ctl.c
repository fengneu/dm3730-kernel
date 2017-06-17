/*
 * character device wrapper for generic gpio layer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA02111-1307USA
 *
 * Feedback, Bugs...  blogic@openwrt.org
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <linux/init.h>
#include <linux/genhd.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>


static int gpio_136_state, gpio_137_state;
static int output_sel;

static ssize_t io_ctl_read(struct file *filp, char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static ssize_t io_ctl_write(struct file *filp,const char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static int io_ctl_ioctl(struct inode * inode, struct file * file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int io_ctl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int io_ctl_close(struct inode * inode, struct file * file)
{
	return 0;
}

static ssize_t gpio_136_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n", gpio_136_state);
}

static ssize_t gpio_136_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        gpio_136_state = simple_strtoul(buf, NULL, 10);

	printk("--------gpio 136 set to %d\n", gpio_136_state);
        gpio_direction_output(136, gpio_136_state);

        return size;
}

static ssize_t gpio_137_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n", gpio_137_state);
}

static ssize_t gpio_137_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        gpio_137_state = simple_strtoul(buf, NULL, 10);

	printk("--------gpio 137 set to %d\n", gpio_137_state);
	gpio_direction_output(137, gpio_137_state);

        return size;
}

static ssize_t output_sel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n", output_sel);
}

static ssize_t output_sel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        output_sel = simple_strtoul(buf, NULL, 10);

	switch(output_sel) {
	case 0:
		gpio_direction_output(137, 0);
		gpio_direction_output(136, 0);
		break;
	case 1:
                gpio_direction_output(137, 1);
                gpio_direction_output(136, 0);		
		break;
	case 2:
                gpio_direction_output(137, 0);
                gpio_direction_output(136, 1);
		break;
	case 3:
                gpio_direction_output(137, 1);
                gpio_direction_output(136, 1);	
		break;
	default:
		printk("--------inval output_sel value!\n");
	}
	
        return size;
}

struct file_operations io_ctl_fops = {
	.read		= io_ctl_read,
	.write		= io_ctl_write,	
	.ioctl		= io_ctl_ioctl,
	.open		= io_ctl_open,
	.release	= io_ctl_close,
};

static struct miscdevice io_ctl_dev = {
        .minor         = MISC_DYNAMIC_MINOR,
        .name         = "io_ctl",                   
        .fops         = &io_ctl_fops,
};

static DEVICE_ATTR(gpio_136_state, 0644, gpio_136_state_show, gpio_136_state_store);
static DEVICE_ATTR(gpio_137_state, 0644, gpio_137_state_show, gpio_137_state_store);
static DEVICE_ATTR(output_sel, 0644, output_sel_show, output_sel_store);

static struct attribute *io_ctl_attributes[] = {
	&dev_attr_gpio_136_state.attr,
	&dev_attr_gpio_137_state.attr,
	&dev_attr_output_sel.attr,
        NULL,
};

static struct attribute_group io_ctl_attr_group = {
        .attrs = io_ctl_attributes,
};

static int __init io_ctl_dev_init(void)
{
	int ret;
	
	printk("--------io_ctl_dev_init\n");

	ret = gpio_request(136, "gpio_136");
	if(ret < 0)
		printk("--------request gpio %d fail!\n", 136);
	
        ret = gpio_request(137, "gpio_137");
        if(ret < 0)
                printk("--------request gpio %d fail!\n", 137);

	ret = misc_register(&io_ctl_dev);
	if(ret){
		printk(KERN_ERR "misc_register failed\n");
		return ret;
	}

	ret = sysfs_create_group(&io_ctl_dev.this_device->kobj, &io_ctl_attr_group);
        if (ret){
		printk(KERN_ERR "creat attr file failed\n");
		misc_deregister(&io_ctl_dev);
		return ret;
	}

	return 0;
}

static void __exit io_ctl_dev_exit(void)
{
	sysfs_remove_group(&io_ctl_dev.this_device->kobj, &io_ctl_attr_group);
	misc_deregister(&io_ctl_dev);
}

module_init (io_ctl_dev_init);
module_exit (io_ctl_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Timll");
MODULE_DESCRIPTION("Character device for power ctl");
