/***********************************************************************************************************************
 *
 * WT440H Receiver / Decoder for Raspberry Pi
 *
 * (C) 2015 Gergely Budai
 *
 **********************************************************************************************************************/
#define DRIVER_NAME "wt440h"
#define DESCRIPTION "WT440H - Wireless Temperature and Humidity sensor receiver driver"
#define AUTHOR      "Gergely Budai"

#include <linux/module.h> /* Needed by all modules */
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/init.h>   /* Needed for the macros */
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/kfifo.h>
#include <asm/uaccess.h>

// GPIO PIN
#define INPUT_PIN                   25

// Bit length in uS
#define BIT_LENGTH                2000
// +- Bit length tolerance in uS
#define BIT_LENGTH_TOLERANCE       200

// Calculated Thresholds for zeros and ones
#define BIT_LENGTH_THRES_LOW      (BIT_LENGTH - BIT_LENGTH_TOLERANCE)
#define BIT_LENGTH_THRES_HIGH     (BIT_LENGTH + BIT_LENGTH_TOLERANCE)
#define HALFBIT_LENGTH_THRES_LOW  (BIT_LENGTH_THRES_LOW  / 2)
#define HALFBIT_LENGTH_THRES_HIGH (BIT_LENGTH_THRES_HIGH / 2)

#define FIFO_SIZE                  256

// Port
static struct gpio_chip *gpiochip;
static int irq_num = 0;

// Device variables
static struct class* device_class = NULL;
static struct device* device_device = NULL;
static int device_major = 0;

DECLARE_WAIT_QUEUE_HEAD(file_read);

// Bit Value with Timestamp
typedef struct {
  unsigned char bit;
  unsigned int timeStamp;
} BitType;


static DEFINE_KFIFO(bit_fifo, BitType, FIFO_SIZE);

static int __init is_right_chip(struct gpio_chip *chip, void *data)
{
    if (strcmp(data, chip->label) == 0) {
      return 1;
    }

    return 0;
}

static inline void queue_bit(unsigned char bitVal, unsigned int timeStamp)
{
  BitType bit;
  unsigned int retval;

  bit.bit = bitVal;
  bit.timeStamp = timeStamp;
  retval = kfifo_put(&bit_fifo, bit);

  switch(retval) {
    case 1: {
      wake_up_interruptible(&file_read);
    }
    break;

    case 0: {
      printk(KERN_WARNING DRIVER_NAME": fifo full, bit missed\n");
    }
    break;

    default: {
      printk(KERN_ERR DRIVER_NAME": kfifo_put() returned %u\n", retval);
    }
    break;
  }
}

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

  do_gettimeofday(&tv);
  timeStamp = (tv.tv_sec * 1000) + tv.tv_usec;

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
      if((bitLength >= BIT_LENGTH_THRES_LOW) && (bitLength <= BIT_LENGTH_THRES_HIGH)) {
        // Full bit length, Zero received
        queue_bit(0, timeStamp);
      }
      else if((bitLength >= HALFBIT_LENGTH_THRES_LOW) && (bitLength <= HALFBIT_LENGTH_THRES_HIGH)) {
        // Half bit length, first half of a One received
        state = HalfBitReceived;
      }
    }
    break;

    // First half of a One received
    case HalfBitReceived: {
      // Check bit length
      if((bitLength >= HALFBIT_LENGTH_THRES_LOW) && (bitLength <= HALFBIT_LENGTH_THRES_HIGH)) {
        // Second half of a One received
        queue_bit(1, timeStamp);
      }
      state = BitStartReceived;
    }
    break;

    // Invalid state (should not happen)
    default: {
      state = BitStartReceived;
    }
    break;
  }

  return IRQ_HANDLED;
}

static int __init init_port(void)
{
  int retval;

  gpiochip = gpiochip_find("pinctrl-bcm2835", is_right_chip);
  if(!gpiochip) {
    printk(KERN_ERR DRIVER_NAME": bcm2835 chip not found!\n");
    retval = -ENODEV;
    goto exit;
  }

  if(gpio_request(INPUT_PIN, DRIVER_NAME)) {
    printk(KERN_ERR DRIVER_NAME": cant claim gpio pin %d\n", INPUT_PIN);
    retval = -ENODEV;
    goto exit;
  }

  gpiochip->direction_input(gpiochip, INPUT_PIN);
  irq_num = gpiochip->to_irq(gpiochip, INPUT_PIN);

  retval = 0;

  exit:
  return retval;
}

static void __exit uninit_port(void)
{
  gpio_free(INPUT_PIN);
}

static int device_open(struct inode* inode, struct file* filp)
{
  int result;

  result = request_irq(irq_num, (irq_handler_t)irq_handler,
    IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING, DRIVER_NAME, (void*) 0);

  switch (result) {
    case -EBUSY: {
      printk(KERN_ERR DRIVER_NAME": IRQ %d is busy\n", irq_num);
      return -EBUSY;
    }
    break;

    case -EINVAL: {
      printk(KERN_ERR DRIVER_NAME": Bad irq number or handler\n");
      return -EINVAL;
    }
    break;
  }

  return result;
}

static int device_close(struct inode* inode, struct file* filp)
{
  irq_set_irq_type(irq_num, 0);
  disable_irq(irq_num);
  free_irq(irq_num, (void *) 0);

  return 0;
}

static ssize_t device_read(struct file* filp, char __user *buffer, size_t length, loff_t* offset)
{
  unsigned int elements, copied = 0;
  BitType bit;
  unsigned char fmtbuf[50];

  wait_event_interruptible(file_read, !kfifo_is_empty(&bit_fifo));
  elements = kfifo_get(&bit_fifo, &bit);
  switch(elements) {
    case 1: {
      copied = snprintf(fmtbuf, sizeof(fmtbuf), "%u %u\n", bit.bit, bit.timeStamp);
      copy_to_user(buffer, fmtbuf, copied);
    }
    break;

    case 0: {
    }
    break;

    default: {
      printk(KERN_ERR DRIVER_NAME": kfifo_get() returned %u\n", elements);
    }
    break;
  }

  return copied;
}

/* The file_operation scructure tells the kernel which device operations are handled.
 * For a list of available file operations, see http://lwn.net/images/pdf/LDD3/ch03.pdf */
static struct file_operations fops = {
 .read = device_read,
 .open = device_open,
 .release = device_close
};

static int __init init_device(void)
{
  unsigned int retval;

  /* First, see if we can dynamically allocate a major for our device */
  device_major = register_chrdev(0, DRIVER_NAME, &fops);
  if (device_major < 0) {
    printk(KERN_ERR DRIVER_NAME": failed to register device\n");
    retval = device_major;
    goto register_chrdev_failed;
  }

  /* We can either tie our device to a bus (existing, or one that we create)
   * or use a "virtual" device class. For this example, we choose the latter */
  device_class = class_create(THIS_MODULE, DRIVER_NAME);
  if (IS_ERR(device_class)) {
    printk(KERN_ERR DRIVER_NAME": failed to register device class\n");
    retval = PTR_ERR(device_class);
    goto class_create_failed;
  }

  /* With a class, the easiest way to instantiate a device is to call device_create() */
  device_device = device_create(device_class, NULL, MKDEV(device_major, 0), NULL, DRIVER_NAME);
  if (IS_ERR(device_device)) {
    printk(KERN_ERR DRIVER_NAME": failed to create device\n");
    retval = PTR_ERR(device_device);
    goto device_create_failed;
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

static void __exit uninit_device(void)
{
  device_destroy(device_class, MKDEV(device_major, 0));
  class_unregister(device_class);
  class_destroy(device_class);
  unregister_chrdev(device_major, DRIVER_NAME);
}

static int __init init_main(void)
{
  int result;

  result = init_port();
  if(result) {
    goto init_port_failed;
  }

  result = init_device();
  if(result) {
    goto init_device_failed;
  }

  printk(KERN_INFO DRIVER_NAME" driver installed on GPIO %d\n", INPUT_PIN);
  result = 0;
  goto exit;

  init_device_failed:
  uninit_port();
  init_port_failed:
  exit:
  return result;
}

static void __exit exit_main(void)
{
  uninit_device();
  uninit_port();

  printk(KERN_INFO DRIVER_NAME" driver deinstalled\n");
}

module_init(init_main);
module_exit(exit_main);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION(DESCRIPTION);
