#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/kthread.h>

#define addBitTimes(time, n) time = ktime_add_us(time, 4 * (n));

#define waitUntilTime(time)                              \
    while (ktime_to_ns(ktime_get()) < ktime_to_ns(time)) \
    {                                                    \
    }

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("rpkak");
MODULE_DESCRIPTION("Try to do DMX");

static struct task_struct *dmxThread;

#define DMX_MAX_BUF_LEN 512

static uint16_t bufLen = 0;
static uint8_t buf[DMX_MAX_BUF_LEN];

static struct gpio_descs *descs = NULL;
// static struct gpio_desc *clk = NULL;

static inline int setGPIOVal(unsigned long *value_bitmap)
{
    return gpiod_set_array_value(descs->ndescs, descs->desc, descs->info, value_bitmap);
}

static int DMXThread(void *data)
{
    ktime_t breakTime = ktime_get();
    ktime_t time;

    while (!kthread_should_stop())
    {
        breakTime = ktime_add_us(breakTime, 22728);
        time = breakTime;

        // BREAK
        waitUntilTime(time);
        setGPIOVal(&(unsigned long){0b00});
        addBitTimes(time, 25);

        // MARK AFTER BREAK
        waitUntilTime(time);
        setGPIOVal(&(unsigned long){0b11});
        addBitTimes(time, 3);

        // gpiod_set_value(clk, 1);

        // SLOT 0 (START CODE)
        waitUntilTime(time);
        setGPIOVal(&(unsigned long){0b00});
        addBitTimes(time, 9);

        waitUntilTime(time);
        setGPIOVal(&(unsigned long){0b11});
        addBitTimes(time, 2);

        // gpiod_set_value(clk, 0);

        for (uint16_t i = 0; i < bufLen; i++)
        {
            // START BIT
            waitUntilTime(time);
            setGPIOVal(&(unsigned long){0b00});
            addBitTimes(time, 1);

            // DATA BITS
            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] << 1) & 0b10) | ((buf[i] >> 0) & 0b01))});
            addBitTimes(time, 1);

            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] >> 0) & 0b10) | ((buf[i] >> 1) & 0b01))});
            addBitTimes(time, 1);

            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] >> 1) & 0b10) | ((buf[i] >> 2) & 0b01))});
            addBitTimes(time, 1);

            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] >> 2) & 0b10) | ((buf[i] >> 3) & 0b01))});
            addBitTimes(time, 1);

            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] >> 3) & 0b10) | ((buf[i] >> 4) & 0b01))});
            addBitTimes(time, 1);

            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] >> 4) & 0b10) | ((buf[i] >> 5) & 0b01))});
            addBitTimes(time, 1);

            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] >> 5) & 0b10) | ((buf[i] >> 6) & 0b01))});
            addBitTimes(time, 1);

            waitUntilTime(time);
            setGPIOVal(&(unsigned long){(((buf[i] >> 6) & 0b10) | ((buf[i] >> 7) & 0b01))});
            addBitTimes(time, 1);

            // STOP BITS
            waitUntilTime(time);
            setGPIOVal(&(unsigned long){0b11});
            addBitTimes(time, 2);
        }
    }
    return 0;
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
    return size;
}

const struct proc_ops dmxProcOps = {
    .proc_read = dmxProcRead,
    .proc_write = dmxProcWrite,
};

static int initDMX(struct platform_device *device)
{
    printk("hi\n");
    if (descs != NULL)
    {
        printk(KERN_ERR "DMX is already running.\n");
        return -1;
    }

    descs = gpiod_get_array(&device->dev, "dmx", GPIOD_OUT_HIGH);
    if (IS_ERR(descs))
    {
        printk(KERN_ERR "GPIO init failed\n");
        return PTR_ERR(descs);
    }
    // clk = gpiod_get(&device->dev, "clk", GPIOD_OUT_LOW);

    dmxProc = proc_create("dmx", 0644, NULL, &dmxProcOps);
    if (dmxProc == NULL)
    {
        printk(KERN_ERR "Creating /proc/dmx failed.\n");
        gpiod_put_array(descs);
        return -ENOMEM;
    }

    dmxThread = kthread_run(DMXThread, NULL, "DMX THREAD");
    if (!dmxThread)
    {
        printk(KERN_ERR "Could not start kthread.\n");
        gpiod_put_array(descs);
        proc_remove(dmxProc);
        return -ENOMEM;
    }
    return 0;
}

static int stopDMX(struct platform_device *device)
{
    printk("bye\n");
    gpiod_set_raw_array_value(descs->ndescs, descs->desc, descs->info, &(unsigned long){0b00});
    gpiod_put_array(descs);
    // gpiod_put(clk);
    proc_remove(dmxProc);
    kthread_stop(dmxThread);
    descs = NULL;
    return 0;
}

static struct of_device_id of_dmx_gpio_match[] = {
    {
        .compatible = "gpio-dmx",
    },
    {},
};

MODULE_DEVICE_TABLE(of, of_dmx_gpio_match);

static struct platform_driver dmxGPIO = {
    .probe = initDMX,
    .remove = stopDMX,
    .driver = {
        .name = "dmx-gpio",
        .of_match_table = of_dmx_gpio_match,
    },
};

module_platform_driver(dmxGPIO);
