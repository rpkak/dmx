#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/kthread.h>

#define add_bit_times(time, n) time = ktime_add_us(time, 4 * (n));

#define wait_until_time(time)                            \
    while (ktime_to_ns(ktime_get()) < ktime_to_ns(time)) \
    {                                                    \
    }

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("rpkak");
MODULE_DESCRIPTION("Try to do DMX");

static struct task_struct *dmx_thread;

#define DMX_MAX_BUF_LEN 512

static uint16_t buf_len = 0;
static uint8_t buf[DMX_MAX_BUF_LEN];

static struct gpio_desc *dmx_pin_desc = NULL;
static struct gpio_desc *clk = NULL;

static int run_dmx_thread(void *data)
{
    ktime_t breakTime = ktime_get();
    ktime_t time;

    while (!kthread_should_stop())
    {
        breakTime = ktime_add_us(breakTime, 22728);
        time = breakTime;

        // BREAK
        wait_until_time(time);
        gpiod_set_value(dmx_pin_desc, 0);
        add_bit_times(time, 25);

        // MARK AFTER BREAK
        wait_until_time(time);
        gpiod_set_value(dmx_pin_desc, 1);
        add_bit_times(time, 3);

        gpiod_set_value(clk, 1);

        // SLOT 0 (START CODE)
        wait_until_time(time);
        gpiod_set_value(dmx_pin_desc, 0);
        add_bit_times(time, 9);

        wait_until_time(time);
        gpiod_set_value(dmx_pin_desc, 1);
        add_bit_times(time, 2);

        gpiod_set_value(clk, 0);

        for (uint16_t i = 0; i < buf_len; i++)
        {
            // START BIT
            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, 0);
            add_bit_times(time, 1);

            // DATA BITS
            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 0) & 0b01);
            add_bit_times(time, 1);

            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 1) & 0b01);
            add_bit_times(time, 1);

            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 2) & 0b01);
            add_bit_times(time, 1);

            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 3) & 0b01);
            add_bit_times(time, 1);

            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 4) & 0b01);
            add_bit_times(time, 1);

            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 5) & 0b01);
            add_bit_times(time, 1);

            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 6) & 0b01);
            add_bit_times(time, 1);

            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, (buf[i] >> 7) & 0b01);
            add_bit_times(time, 1);

            // STOP BITS
            wait_until_time(time);
            gpiod_set_value(dmx_pin_desc, 1);
            add_bit_times(time, 2);
        }
    }
    return 0;
}

static struct proc_dir_entry *dmx_proc;

static ssize_t dmx_proc_read(struct file *file, char __user *user_buf, size_t size, loff_t *off)
{
    if (size > buf_len - *off)
    {
        size = buf_len - *off;
    }
    size -= copy_to_user(user_buf, buf + *off, size);
    *off += size;
    return size;
}

static ssize_t dmx_proc_write(struct file *file, const char __user *user_buf, size_t size, loff_t *off)
{
    if (size > DMX_MAX_BUF_LEN - *off)
    {
        size = DMX_MAX_BUF_LEN - *off;
    }
    size -= copy_from_user(buf + *off, user_buf, size);
    *off += size;
    buf_len = *off;
    return size;
}

const struct proc_ops dmx_proc_ops = {
    .proc_read = dmx_proc_read,
    .proc_write = dmx_proc_write,
};

static int dmx_init(struct platform_device *device)
{
    printk("hi\n");
    if (dmx_pin_desc != NULL)
    {
        printk(KERN_ERR "DMX is already running.\n");
        return -1;
    }

    dmx_pin_desc = gpiod_get(&device->dev, "dmx", GPIOD_OUT_HIGH);

    if (IS_ERR(dmx_pin_desc))
    {
        printk(KERN_ERR "GPIO init failed\n");
        return PTR_ERR(dmx_pin_desc);
    }
    clk = gpiod_get(&device->dev, "clk", GPIOD_OUT_LOW);

    dmx_proc = proc_create("dmx", 0644, NULL, &dmx_proc_ops);
    if (dmx_proc == NULL)
    {
        printk(KERN_ERR "Creating /proc/dmx failed.\n");
        gpiod_put(dmx_pin_desc);
        return -ENOMEM;
    }

    dmx_thread = kthread_run(run_dmx_thread, NULL, "DMX THREAD");
    if (!dmx_thread)
    {
        printk(KERN_ERR "Could not start kthread.\n");
        gpiod_put(dmx_pin_desc);
        proc_remove(dmx_proc);
        return -ENOMEM;
    }
    return 0;
}

static int dmx_stop(struct platform_device *device)
{
    printk("bye\n");
    kthread_stop(dmx_thread);
    proc_remove(dmx_proc);
    gpiod_put(clk);
    gpiod_set_value(dmx_pin_desc, 0);
    gpiod_put(dmx_pin_desc);
    dmx_pin_desc = NULL;
    return 0;
}

static struct of_device_id of_dmx_gpio_match[] = {
    {
        .compatible = "gpio-dmx",
    },
    {},
};

MODULE_DEVICE_TABLE(of, of_dmx_gpio_match);

static struct platform_driver dmx_gpio = {
    .probe = dmx_init,
    .remove = dmx_stop,
    .driver = {
        .name = "dmx-gpio",
        .of_match_table = of_dmx_gpio_match,
    },
};

module_platform_driver(dmx_gpio);
