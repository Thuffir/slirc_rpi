/***********************************************************************************************************************
 *
 * Biphase Mark Decoder Kernel Module for Raspberry Pi
 *
 * Copyright (C) 2015 Gergely Budai
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **********************************************************************************************************************/

#define DRIVER_NAME "bmd"
#define DESCRIPTION "Biphase Mark Decoder"
#define AUTHOR      "Gergely Budai"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/kfifo.h>
#include <asm/uaccess.h>

// Bit definitions
#define BIT_ZERO                  0
#define BIT_ONE                   1
#define BIT_TIMEOUT               2
#define BIT_MSK                   1
#define BIT_TIMEOUT_MSK           2

// Calculated Thresholds for bit lengths
static unsigned int bit_length_thres_low, bit_length_thres_high, halfbit_length_thres_low, halfbit_length_thres_high;

// Bits FIFO
#define FIFO_SIZE                  32
static DEFINE_KFIFO(bit_fifo, unsigned char, FIFO_SIZE);
// Fifo usage statistics
static unsigned int max_bit_fifo_used = 0;

// For blocking read
static DECLARE_WAIT_QUEUE_HEAD(file_read);

// IRQ
static int irq_num = 0;

// Device variables
static struct class* device_class = NULL;
static struct device* device_device = NULL;
static int device_major = 0;

/***********************************************************************************************************************
 * Module Parameters
 **********************************************************************************************************************/
// GPIO to use
static unsigned int gpio = 25;
module_param(gpio, uint, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(gpio, "GPIO to use (default=25)");

// Bit length in us
static unsigned int bit_length = 2000;
module_param(bit_length, uint, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(bit_length, "Bit length in us (default=2000)");

// +- Bit length tolerance in uS
static unsigned int bit_length_tolerance = 200;
module_param(bit_length_tolerance, uint, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(bit_length_tolerance, "Bit length tolerance in us (default=200)");

/***********************************************************************************************************************
 * Queues one bit into the bit fifo
 **********************************************************************************************************************/
static inline void queue_bit(unsigned char bit, unsigned int time_stamp)
{
  static unsigned int last_timestamp = 0;
  unsigned int retval, bit_length, fifo_used;

  // Get bit length
  bit_length = time_stamp - last_timestamp;
  last_timestamp = time_stamp;

  // Mark bit timeout if so
  if((bit_length < bit_length_thres_low) || (bit_length > bit_length_thres_high)) {
    bit |= BIT_TIMEOUT;
  }

  // Put into FIFO
  retval = kfifo_put(&bit_fifo, bit);

  // Provide statistics over fifo usage
  fifo_used = kfifo_len(&bit_fifo);
  if(fifo_used > max_bit_fifo_used) {
    max_bit_fifo_used = fifo_used;
  }

  // Check return value
  switch(retval) {
    // Everything OK, wake up reader
    case 1: {
      wake_up_interruptible(&file_read);
    }
    break;

    // FIFO Full
    case 0: {
      printk(KERN_WARNING DRIVER_NAME": fifo full, bit missed\n");
    }
    break;

    // Should not happen
    default: {
      printk(KERN_ERR DRIVER_NAME": kfifo_put() returned %u\n", retval);
    }
    break;
  }
}

/***********************************************************************************************************************
 * IRQ Handler
 **********************************************************************************************************************/
static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs)
{
  // Internal State for bit recognition
  static enum {
    Init,
    BitStartReceived,
    HalfBitReceived
  } state = Init;
  // Last timestamp for bit length calculation
  static unsigned int lastTimeStamp = 0;
  // Bit Length
  unsigned int bitLength, timeStamp;
  struct timeval tv;

  // Get Timestamp
  do_gettimeofday(&tv);
  timeStamp = (tv.tv_sec * 1000000) + tv.tv_usec;

  // Calculate bit length
  bitLength = timeStamp - lastTimeStamp;
  lastTimeStamp = timeStamp;

  // Bit recognition state machine
  switch(state) {
    // Init State, no start Timestamp yet
    case Init: {
      state = BitStartReceived;
    }
    break;

    // Start edge of a bit has been received
    case BitStartReceived: {
      // Check bit length
      if((bitLength >= bit_length_thres_low) && (bitLength <= bit_length_thres_high)) {
        // Full bit length, Zero received
        queue_bit(BIT_ZERO, timeStamp);
      }
      else if((bitLength >= halfbit_length_thres_low) && (bitLength <= halfbit_length_thres_high)) {
        // Half bit length, first half of a One received
        state = HalfBitReceived;
      }
    }
    break;

    // First half of a One received
    case HalfBitReceived: {
      // Check bit length
      if((bitLength >= halfbit_length_thres_low) && (bitLength <= halfbit_length_thres_high)) {
        // Second half of a One received
        queue_bit(BIT_ONE, timeStamp);
      }
      state = BitStartReceived;
    }
    break;

    // Invalid state (should not happen)
    default: {
      printk(KERN_ERR DRIVER_NAME": irq handler has invalid state: %u\n", state);
      state = BitStartReceived;
    }
    break;
  }

  return IRQ_HANDLED;
}

/***********************************************************************************************************************
 * Check if we have the right chipset
 **********************************************************************************************************************/
static int __init is_right_chip(struct gpio_chip *chip, void *data)
{
    if (strcmp(data, chip->label) == 0) {
      return 1;
    }

    return 0;
}

/***********************************************************************************************************************
 * Init GPIO Port
 **********************************************************************************************************************/
static int __init init_port(void)
{
  struct gpio_chip *gpiochip;
  int retval;

  // Find Chipset
  gpiochip = gpiochip_find("pinctrl-bcm2835", is_right_chip);
  if(!gpiochip) {
    printk(KERN_ERR DRIVER_NAME": bcm2835 chip not found!\n");
    retval = -ENODEV;
    goto exit;
  }

  // Request GPIO Port
  if(gpio_request(gpio, DRIVER_NAME)) {
    printk(KERN_ERR DRIVER_NAME": cant claim gpio pin %d\n", gpio);
    retval = -ENODEV;
    goto exit;
  }

  // Set GPIO direction
  gpiochip->direction_input(gpiochip, gpio);

  // Recort IRQ number for later
  irq_num = gpiochip->to_irq(gpiochip, gpio);

  retval = 0;

  exit:
  return retval;
}

/***********************************************************************************************************************
 * Deinit GPIO Port
 **********************************************************************************************************************/
static void __exit uninit_port(void)
{
  gpio_free(gpio);
}

/***********************************************************************************************************************
 * Device file open
 **********************************************************************************************************************/
static int device_open(struct inode* inode, struct file* filp)
{
  int result;

  // Request GPIO IRQ
  result = request_irq(irq_num, (irq_handler_t)irq_handler,
    IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING, DRIVER_NAME, (void*) 0);

  // Process result
  switch (result) {
    // IRQ Already used (probably device already opened)
    case -EBUSY: {
      printk(KERN_ERR DRIVER_NAME": IRQ %d is busy\n", irq_num);
      goto exit;
    }
    break;

    // Bad IRQ
    case -EINVAL: {
      printk(KERN_ERR DRIVER_NAME": Bad irq number or handler\n");
      goto exit;
    }
    break;
  }

  // Increment usage counter;
  try_module_get(THIS_MODULE);

  exit:
  return result;
}

/***********************************************************************************************************************
 * Device file close
 **********************************************************************************************************************/
static int device_close(struct inode* inode, struct file* filp)
{
  // Disable and free the IRQ
  irq_set_irq_type(irq_num, 0);
  disable_irq(irq_num);
  free_irq(irq_num, (void *) 0);

  // Decrement usage counter;
  module_put(THIS_MODULE);

  return 0;
}

/***********************************************************************************************************************
 * Device file read
 **********************************************************************************************************************/
static ssize_t device_read(struct file* filp, char __user *buffer, size_t length, loff_t* offset)
{
  int retval;
  unsigned int copied;

  // Wait for event from interrupt
  wait_event_interruptible(file_read, !kfifo_is_empty(&bit_fifo));
  // Get bit from fifo
  retval = kfifo_to_user(&bit_fifo, buffer, length, &copied);

  return retval ? retval : copied;
}

/***********************************************************************************************************************
 * File operations structure
 **********************************************************************************************************************/
static struct file_operations fops = {
 .read = device_read,
 .open = device_open,
 .release = device_close
};

/***********************************************************************************************************************
 * Sysfs file with Fifo Statistics
 **********************************************************************************************************************/
static ssize_t show_fifo_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
  return scnprintf(buf, PAGE_SIZE, "Max: %u\nCur: %u\n", max_bit_fifo_used, kfifo_len(&bit_fifo));
}
static DEVICE_ATTR(fifo, S_IRUSR | S_IRGRP | S_IROTH, show_fifo_stats, NULL);

/***********************************************************************************************************************
 * Initialize device file
 **********************************************************************************************************************/
static int __init init_device(void)
{
  unsigned int retval;

  // First, see if we can dynamically allocate a major for our device
  device_major = register_chrdev(0, DRIVER_NAME, &fops);
  if (device_major < 0) {
    printk(KERN_ERR DRIVER_NAME": failed to register device\n");
    retval = device_major;
    goto register_chrdev_failed;
  }

  // Tie device to a virtual class
  device_class = class_create(THIS_MODULE, DRIVER_NAME);
  if (IS_ERR(device_class)) {
    printk(KERN_ERR DRIVER_NAME": failed to register device class\n");
    retval = PTR_ERR(device_class);
    goto class_create_failed;
  }

  // Create device file
  device_device = device_create(device_class, NULL, MKDEV(device_major, 0), NULL, DRIVER_NAME);
  if (IS_ERR(device_device)) {
    printk(KERN_ERR DRIVER_NAME": failed to create device\n");
    retval = PTR_ERR(device_device);
    goto device_create_failed;
  }

  // Sysfs file for fifo stats
  if(device_create_file(device_device, &dev_attr_fifo) < 0) {
    printk(KERN_WARNING DRIVER_NAME": could not create sysfs fifo stats file\n");
  }

  // Everything OK
  retval = 0;
  goto exit;

  // Error unitialisation
  device_create_failed:
  class_unregister(device_class);
  class_destroy(device_class);
  class_create_failed:
  unregister_chrdev(device_major, DRIVER_NAME);
  register_chrdev_failed:

  // Exit point
  exit:
  return retval;
}

/***********************************************************************************************************************
 * Deinitialize device file
 **********************************************************************************************************************/
static void __exit uninit_device(void)
{
  device_remove_file(device_device, &dev_attr_fifo);
  device_destroy(device_class, MKDEV(device_major, 0));
  class_unregister(device_class);
  class_destroy(device_class);
  unregister_chrdev(device_major, DRIVER_NAME);
}

/***********************************************************************************************************************
 * Module Entry
 **********************************************************************************************************************/
static int __init init_main(void)
{
  int result;

  // Calculate bit length thresholds
  bit_length_thres_low      = bit_length - bit_length_tolerance;
  bit_length_thres_high     = bit_length + bit_length_tolerance;
  halfbit_length_thres_low  = (bit_length / 2) - bit_length_tolerance;
  halfbit_length_thres_high = (bit_length / 2) + bit_length_tolerance;

  // Init port
  result = init_port();
  if(result) {
    goto init_port_failed;
  }

  // Init device file
  result = init_device();
  if(result) {
    goto init_device_failed;
  }

  // Init OK, print info message
  printk(KERN_INFO DRIVER_NAME": driver installed on GPIO %u\n", gpio);
  result = 0;
  goto exit;

  init_device_failed:
  uninit_port();
  init_port_failed:
  exit:
  return result;
}

/***********************************************************************************************************************
 * Module Exit
 **********************************************************************************************************************/
static void __exit exit_main(void)
{
  // Deinit device file
  uninit_device();
  // Deinit GPIO Port
  uninit_port();

  printk(KERN_INFO DRIVER_NAME": driver uninstalled\n");
}

// Init and Exit functions
module_init(init_main);
module_exit(exit_main);

// Module informations
MODULE_LICENSE("GPL");
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION(DESCRIPTION);
