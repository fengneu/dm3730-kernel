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

#define BT_EN_GPIO		15

static int bt_state;

static ssize_t pwr_ctl_read(struct file *filp, char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static ssize_t pwr_ctl_write(struct file *filp,const char *buf,size_t count,loff_t *f_ops)
{
	return count;
}

static int pwr_ctl_ioctl(struct inode * inode, struct file * file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int pwr_ctl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int pwr_ctl_close(struct inode * inode, struct file * file)
{
	return 0;
}

static ssize_t bt_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n", bt_state);
}

static ssize_t bt_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        bt_state = simple_strtoul(buf, NULL, 10);

        if(bt_state){
                gpio_set_value(BT_EN_GPIO, 1);
        }else{
                gpio_set_value(BT_EN_GPIO, 0);
        }

        return size;
}

struct file_operations pwr_ctl_fops = {
	.read		= pwr_ctl_read,
	.write		= pwr_ctl_write,	
	.ioctl		= pwr_ctl_ioctl,
	.open		= pwr_ctl_open,
	.release	= pwr_ctl_close,
};

static struct miscdevice pwr_ctl_dev = {
        .minor         = MISC_DYNAMIC_MINOR,
        .name         = "pwr_ctl",                   
        .fops         = &pwr_ctl_fops,
};

static DEVICE_ATTR(bt_state, 0644, bt_state_show, bt_state_store);

static struct attribute *pwr_ctl_attributes[] = {
	&dev_attr_bt_state.attr,
        NULL,
};

static struct attribute_group pwr_ctl_attr_group = {
        .attrs = pwr_ctl_attributes,
};

static int __init pwr_ctl_dev_init(void)
{
	int ret;

	ret = misc_register(&pwr_ctl_dev);
	if(ret){
		printk(KERN_ERR "misc_register failed\n");
		return ret;
	}

	ret = sysfs_create_group(&pwr_ctl_dev.this_device->kobj, &pwr_ctl_attr_group);
        if (ret){
		printk(KERN_ERR "creat attr file failed\n");
		misc_deregister(&pwr_ctl_dev);
		return ret;
	}

	return 0;
}

static void __exit pwr_ctl_dev_exit(void)
{
	sysfs_remove_group(&pwr_ctl_dev.this_device->kobj, &pwr_ctl_attr_group);
	misc_deregister(&pwr_ctl_dev);
}

module_init (pwr_ctl_dev_init);
module_exit (pwr_ctl_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Timll");
MODULE_DESCRIPTION("Character device for power ctl");
