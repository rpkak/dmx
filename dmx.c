#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

// static const unsigned GPIO_PIN = 4;

#define addBitTimes(time, n) time = ktime_add_us(time, 4 * (n));

#define waitUntilTime(time)                              \
    while (ktime_to_ns(ktime_get()) < ktime_to_ns(time)) \
    {                                                    \
    }

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("rpkak");
MODULE_DESCRIPTION("Try to do DMX");

#define DMX_MAX_BUF_LEN 512

static uint16_t bufLen = 0;
static uint8_t buf[DMX_MAX_BUF_LEN];

static void sendDMX(void)
{
    ktime_t time = ktime_get();

    // BREAK
    gpio_set_value(4, 0);
    addBitTimes(time, 25);

    // MARK AFTER BREAK
    waitUntilTime(time);
    gpio_set_value(4, 1);
    addBitTimes(time, 3);

    // SLOT 0 (START CODE)
    waitUntilTime(time);
    gpio_set_value(4, 0);
    addBitTimes(time, 9);

    waitUntilTime(time);
    gpio_set_value(4, 1);
    addBitTimes(time, 2);

    for (uint16_t i = 0; i < bufLen; i++)
    {
        // START BIT
        waitUntilTime(time);
        gpio_set_value(4, 0);
        addBitTimes(time, 1);

        // DATA BITS
        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 0) & 0x1);
        addBitTimes(time, 1);

        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 1) & 0x1);
        addBitTimes(time, 1);

        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 2) & 0x1);
        addBitTimes(time, 1);

        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 3) & 0x1);
        addBitTimes(time, 1);

        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 4) & 0x1);
        addBitTimes(time, 1);

        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 5) & 0x1);
        addBitTimes(time, 1);

        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 6) & 0x1);
        addBitTimes(time, 1);

        waitUntilTime(time);
        gpio_set_value(4, (buf[i] >> 7) & 0x1);
        addBitTimes(time, 1);

        // STOP BITS
        waitUntilTime(time);
        gpio_set_value(4, 1);
        addBitTimes(time, 2);
    }
}

static struct proc_dir_entry *dmxProc;

static ssize_t dmxProcRead(struct file *file, char __user *userBuf, size_t size, loff_t *off)
{
    if (size > bufLen - *off)
    {
        size = bufLen - *off;
    }
    size -= copy_to_user(userBuf, buf + *off, size);
    *off += size;
    return size;
}

static ssize_t dmxProcWrite(struct file *file, const char __user *userBuf, size_t size, loff_t *off)
{
    if (size > DMX_MAX_BUF_LEN - *off)
    {
        size = DMX_MAX_BUF_LEN - *off;
    }
    size -= copy_from_user(buf + *off, userBuf, size);
    *off += size;
    bufLen = *off;
    sendDMX();
    return size;
}

const struct proc_ops dmxProcOps = {
    .proc_read = dmxProcRead,
    .proc_write = dmxProcWrite,
};

static int __init initModule(void)
{
    printk("init aabbcc\n");
    // buf = 0b01010101;

    if (gpio_request(4, "rpi-gpio-4"))
    {
        printk(KERN_ERR "Request gpio pin 4 failed.\n");
        goto err_none;
    }

    if (gpio_direction_output(4, 1))
    {
        printk(KERN_ERR "Setting gpio pin 4 to output failed.\n");
        goto err_free_gpio;
    }

    // dmxThread = kthread_run(DMXThread, NULL, "DMX THREAD");
    // if (!dmxThread)
    // {
    //     printk(KERN_ERR "Could not start kthread.\n");
    //     goto err_free_gpio;
    // }

    dmxProc = proc_create("dmx", 0644, NULL, &dmxProcOps);
    if (dmxProc == NULL)
    {
        printk(KERN_ERR "Creating /proc/dmx failed.\n");
        goto err_free_gpio;
    }

    return 0;
// err_stop_thread:
//     kthread_stop(dmxThread);
err_free_gpio:
    gpio_free(4);
err_none:
    return -ENOMEM;
}

static void __exit exitModule(void)
{
    proc_remove(dmxProc);
    gpio_set_value(4, 0);
    gpio_free(4);
    // kthread_stop(dmxThread);
    printk("exit aabbcc\n");
}

module_init(initModule);
module_exit(exitModule);
