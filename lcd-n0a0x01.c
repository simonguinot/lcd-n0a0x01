/*
 * LCD driver for Seagate n0a0x01
 *
 * Copyright (C) 2014 Seagate
 *
 * Author: <christophe.vu-brugier@seagate.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * The IT8732 super I/O of Seagate n0a0x01 contains a 8032 embedded
 * controller which is used to control the LCD. A firmware flashed in
 * the embedded controller manages the LCD.
 *
 * A "busy" flag must be set to signal the embedded controller
 * firmware that it will have to display new characters. Once the
 * "busy" flag is set, the firmware waits for 1 ms for the characters
 * to be written to the BRAM. Once the new string is displayed on the
 * LCD, the firmware clears the "busy" flag to let the driver know
 * that is ready to handle data again.
 */

#include <linux/cdev.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/jiffies.h>

#define DRVNAME			"lcd-n0a0x01"

#define BRAM_INDEX		0x94
#define BRAM_DATA		0x95

#define LCD_DISPLAY_BASE	0x10
#define LCD_BUSY_FLAG		0x31
#define LCD_INIT		0x33
#define LCD_BACKLIGHT		0x30

#define LCD_MAX_CHARACTERS	((size_t) 32)

static int major;
static struct cdev lcd_cdev;
static struct class *lcd_class;
static dev_t lcd_dev;
static bool lcd_init = true;
static int backlight_enabled = -1;
#ifdef CONFIG_PM_SLEEP
static int backlight_susp_value;
#endif
static unsigned int polling_time = 5;

static inline void bram_set_flag(int flag, int val)
{
	outb(flag, BRAM_INDEX);
	outb(!!val, BRAM_DATA);
}

static inline int bram_get_flag(int flag)
{
	outb(flag, BRAM_INDEX);
	return inb(BRAM_DATA);
}

static inline void bram_write_char(unsigned int index, char c)
{
	outb(LCD_DISPLAY_BASE + index, BRAM_INDEX);
	outb(c, BRAM_DATA);
}

static inline char bram_read_char(unsigned int index)
{
	outb(LCD_DISPLAY_BASE + index, BRAM_INDEX);
	return inb(BRAM_DATA);
}

static ssize_t lcd_read(struct file *filp, char __user *buff,
			size_t count, loff_t *offp)
{
	int i;

	if (*offp >= LCD_MAX_CHARACTERS)
		return 0;

	for (i = *offp; i < LCD_MAX_CHARACTERS; i++)
		buff[i] = bram_read_char(i);
	buff[LCD_MAX_CHARACTERS] = '\0';

	*offp += LCD_MAX_CHARACTERS;
	return LCD_MAX_CHARACTERS;
}

static ssize_t lcd_write(struct file *filp, const char __user *buff,
			 size_t count, loff_t *offp)
{
	size_t len = min(count, LCD_MAX_CHARACTERS);
	int i;
	char c;

	while (bram_get_flag(LCD_BUSY_FLAG))
		schedule_timeout(msecs_to_jiffies(polling_time));
	bram_set_flag(LCD_BUSY_FLAG, 1);

	if (lcd_init) {
		bram_set_flag(LCD_INIT, 1);
		lcd_init = false;
	}

	if (buff[len - 1] == '\n')
		len--;

	for (i = 0; i < len; i++) {
		c = isprint(buff[i]) ? buff[i] : ' ';
		bram_write_char(i, c);
	}
	for (i = len; i < LCD_MAX_CHARACTERS; i++)
		bram_write_char(i, ' ');

	return count;
}

static int backlight_get(void)
{
	if (backlight_enabled == -1)
		backlight_enabled = bram_get_flag(LCD_BACKLIGHT);

	return backlight_enabled;
}

static void backlight_set(int enable)
{
	if (enable == backlight_get())
		return;

	while (bram_get_flag(LCD_BUSY_FLAG))
		schedule_timeout(msecs_to_jiffies(polling_time));

	bram_set_flag(LCD_BUSY_FLAG, 1);
	bram_set_flag(LCD_BACKLIGHT, enable);
	backlight_enabled = enable;
}

static ssize_t backlight_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", backlight_get());
}

static ssize_t backlight_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long enable;
	int err = kstrtoul(buf, 10, &enable);
	if (err)
		return err;

	backlight_set(enable);

	return count;
}


static ssize_t polling_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", polling_time);
}

static ssize_t polling_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long time;
	int err = kstrtoul(buf, 10, &time);
	if (err)
		return err;

	polling_time = (unsigned int)time;

	return count;
}

static DEVICE_ATTR(backlight, 0644, backlight_show, backlight_store);
static DEVICE_ATTR(polling, 0644, polling_show, polling_store);

static struct attribute *lcd_attributes[] = {
	&dev_attr_backlight.attr,
	&dev_attr_polling.attr,
	NULL,
};

static struct attribute_group lcd_attribute_group = {
	.attrs = lcd_attributes,
};

static const struct file_operations lcd_fops = {
	.read		= lcd_read,
	.write		= lcd_write,
};

static int lcd_create_device(struct device *parent)
{
	struct device *device;
	int ret;

	if (major) {
		lcd_dev = MKDEV(major, 0);
		ret = register_chrdev_region(lcd_dev, 1, DRVNAME);
	} else {
		ret = alloc_chrdev_region(&lcd_dev, 0, 1, DRVNAME);
		major = MAJOR(lcd_dev);
	}
	if (ret < 0)
		return ret;

	cdev_init(&lcd_cdev, &lcd_fops);
	lcd_cdev.owner = THIS_MODULE;
	ret = cdev_add(&lcd_cdev, lcd_dev, 1);
	if (ret < 0)
		goto unregister_chrdev;

	lcd_class = class_create(THIS_MODULE, "lcd");
	if (IS_ERR(lcd_class)) {
		ret = PTR_ERR(lcd_class);
		goto cdev_del;
	}

	device = device_create(lcd_class, parent, lcd_dev, NULL, "lcd");
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto class_destroy;
	}

	ret = sysfs_create_group(&parent->kobj, &lcd_attribute_group);
	if (ret)
		goto class_destroy;

	backlight_set(1);

	return 0;

class_destroy:
	class_destroy(lcd_class);
cdev_del:
	cdev_del(&lcd_cdev);
unregister_chrdev:
	unregister_chrdev_region(lcd_dev, 1);
	return ret;
}

static void lcd_remove_device(void)
{
	device_destroy(lcd_class, lcd_dev);
	class_destroy(lcd_class);
	cdev_del(&lcd_cdev);
	unregister_chrdev_region(lcd_dev, 1);
}

static int lcd_n0a0x01_probe(struct platform_device *pdev)
{
	return lcd_create_device(&pdev->dev);
}

static int lcd_n0a0x01_remove(struct platform_device *pdev)
{
	lcd_remove_device();
	backlight_set(0);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lcd_n0a0x01_suspend(struct device *dev)
{
	backlight_susp_value = backlight_get();
	backlight_set(0);
	return 0;
}

static int lcd_n0a0x01_resume(struct device *dev)
{
	backlight_set(backlight_susp_value);
	return 0;
}

static SIMPLE_DEV_PM_OPS(lcd_n0a0x01_pm, lcd_n0a0x01_suspend,
						lcd_n0a0x01_resume);
#define LCD_SUPERBEE_PM (&lcd_n0a0x01_pm)
#else
#define LCD_SUPERBEE_PM NULL
#endif

static struct platform_device *lcd_n0a0x01_pdev;

static struct platform_driver lcd_n0a0x01_driver = {
	.probe		= lcd_n0a0x01_probe,
	.remove		= lcd_n0a0x01_remove,
	.driver		= {
		.name	= DRVNAME,
		.pm	= LCD_SUPERBEE_PM,
		.owner	= THIS_MODULE,
	},
};

static int __init lcd_n0a0x01_init(void)
{
	int ret;

	/* FIXME: detect LCD device */

	ret = platform_driver_register(&lcd_n0a0x01_driver);
	if (ret < 0)
		return ret;

	lcd_n0a0x01_pdev =
		platform_device_register_simple(DRVNAME, -1, NULL, 0);
	if (IS_ERR(lcd_n0a0x01_pdev)) {
		ret = PTR_ERR(lcd_n0a0x01_pdev);
		platform_driver_unregister(&lcd_n0a0x01_driver);
		return ret;
	}

	return 0;
}

static void __exit lcd_n0a0x01_exit(void)
{
	platform_device_unregister(lcd_n0a0x01_pdev);
	platform_driver_unregister(&lcd_n0a0x01_driver);
}

module_init(lcd_n0a0x01_init);
module_exit(lcd_n0a0x01_exit);

module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major number for the LCD driver");
MODULE_AUTHOR("Christophe Vu-Brugier <christophe.vu-brugier@seagate.com>");
MODULE_DESCRIPTION("LCD driver for Seagate n0a0x01");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lcd-n0a0x01");
