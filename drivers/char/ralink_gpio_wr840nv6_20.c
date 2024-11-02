/*
 ***************************************************************************
 * Ralink Tech Inc.
 * 4F, No. 2 Technology 5th Rd.
 * Science-based Industrial Park
 * Hsin-chu, Taiwan, R.O.C.
 *
 * (c) Copyright, Ralink Technology, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 ***************************************************************************
 *
 */
#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#ifdef CONFIG_RALINK_GPIO_LED
#include <linux/timer.h>
#endif
#include <asm/uaccess.h>
#include "ralink_gpio.h"

#include <asm/rt2880/surfboardint.h>

#ifdef  CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
static  devfs_handle_t devfs_handle;
#endif
#include <linux/proc_fs.h>

#include <linux/rtnetlink.h>
#include <net/rtnetlink.h>

#define NAME				"ralink_gpio"
#define RALINK_GPIO_DEVNAME	"gpio"
int ralink_gpio_major = 252;
int ralink_gpio_irqnum = 0;
u32 ralink_gpio_intp = 0;
u32 ralink_gpio_edge = 0;
#if defined (RALINK_GPIO_HAS_2722)
u32 ralink_gpio2722_intp = 0;
u32 ralink_gpio2722_edge = 0;
#elif defined (RALINK_GPIO_HAS_4524)
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio4540_intp = 0;
u32 ralink_gpio4540_edge = 0;
#elif defined (RALINK_GPIO_HAS_5124)
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio5140_intp = 0;
u32 ralink_gpio5140_edge = 0;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
u32 ralink_gpio3924_intp = 0;
u32 ralink_gpio3924_edge = 0;
u32 ralink_gpio7140_intp = 0;
u32 ralink_gpio7140_edge = 0;
#if defined (RALINK_GPIO_HAS_7224)
u32 ralink_gpio72_intp = 0;
u32 ralink_gpio72_edge = 0;
#else
u32 ralink_gpio9572_intp = 0;
u32 ralink_gpio9572_edge = 0;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
u32 ralink_gpio6332_intp = 0;
u32 ralink_gpio6332_edge = 0;
u32 ralink_gpio9564_intp = 0;
u32 ralink_gpio9564_edge = 0;
#endif
ralink_gpio_reg_info ralink_gpio_info[RALINK_GPIO_NUMBER];
extern unsigned long volatile jiffies;

#ifdef CONFIG_RALINK_GPIO_LED
#define RALINK_LED_DEBUG 0
#define RALINK_GPIO_LED_FREQ (HZ/5)
#define RESET_TRIGGER (5*3)
#define WLAN_LOCK_TIMES (5*5)

#define SYS_READY_TIME  (30 * (HZ / RALINK_GPIO_LED_FREQ))

typedef enum _ENUM_BUTTON_CHK
{
    RESET_BUTTON_PRESSED = 0,
    WIFI_BUTTON_PRESSED,
    SWITCH_BUTTON_PRESSED,
    LED_BUTTON_PRESSED,
    RE_BUTTON_PRESSED,
    WPS_BUTTON_PRESSED,
    BUTTON_CHK_NUM,
}ENUM_BUTTON_CHK;

struct timer_list ralink_gpio_led_timer;
ralink_gpio_led_info ralink_gpio_led_data[RALINK_GPIO_NUMBER];

u32 ra_gpio_led_set = 0;
u32 ra_gpio_led_clr = 0;
#if defined (RALINK_GPIO_HAS_2722)
u32 ra_gpio2722_led_set = 0;
u32 ra_gpio2722_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_4524)
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio4540_led_set = 0;
u32 ra_gpio4540_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_5124)
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio5140_led_set = 0;
u32 ra_gpio5140_led_clr = 0;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
u32 ra_gpio3924_led_set = 0;
u32 ra_gpio3924_led_clr = 0;
u32 ra_gpio7140_led_set = 0;
u32 ra_gpio7140_led_clr = 0;
#if defined (RALINK_GPIO_HAS_7224)
u32 ra_gpio72_led_set = 0;
u32 ra_gpio72_led_clr = 0;
#else
u32 ra_gpio9572_led_set = 0;
u32 ra_gpio9572_led_clr = 0;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
u32 ra_gpio6332_led_set = 0;
u32 ra_gpio6332_led_clr = 0;
u32 ra_gpio9564_led_set = 0;
u32 ra_gpio9564_led_clr = 0;
#endif
struct ralink_gpio_led_status_t {
	int ticks;
	unsigned int ons;
	unsigned int offs;
	unsigned int resting;
	unsigned int times;
} ralink_gpio_led_stat[RALINK_GPIO_NUMBER];
#endif
void ralink_gpio_notify_user(int usr);
static struct work_struct gpio_event_hold;
static struct work_struct gpio_event_click;

static struct proc_dir_entry *simple_config_entry = NULL;

MODULE_DESCRIPTION("Ralink SoC GPIO Driver");
MODULE_AUTHOR("Winfred Lu <winfred_lu@ralinktech.com.tw>");
MODULE_LICENSE("GPL");
ralink_gpio_reg_info info;

void send_gpiobtn_event_to_user(char *buf, int len);

typedef enum _ENUM_MULTIMODE
{
    MULTIMODE_NONE_MODE = -1,
    MULTIMODE_ROUTER_MODE = 0,
    MULTIMODE_AP_MODE,
    MULTIMODE_CLIENT_MODE,
    MULTIMODE_REPEATER_MODE,
    MULTIMODE_MSSID_MODE,
    MULTIMODE_HOTSPOT_MODE,
    MULTIMODE_MODEM_MODE,
    MULTIMODE_NUM,
}ENUM_MULTIMODE;


typedef enum _ENUM_RE_BRIDGE_STATUS
{
	RE_BRIDGE_STATUS_IDLE = 0,
	RE_BRIDGE_STATUS_BRIDGING,
	RE_BRIDGE_STATUS_BRIDGED
} ENUM_RE_BRIDGE_STATUS;

static int wlan_24G_status = 0;
static int wlan_5G_status = 0;
static int wan_status = 0;
static int sys_status = 0;
static int sys_ready = 0;
static int hw_version = 0;
static int led_status = 1;
static int wl_mode = MULTIMODE_ROUTER_MODE; 
static int onekey_re_pressed = 0;
static int onekey_re_event_sent = 0;
static ENUM_RE_BRIDGE_STATUS re_bridge_status = RE_BRIDGE_STATUS_IDLE;
static int re_bridging_time = 0;
static int hw_mode = MULTIMODE_ROUTER_MODE; 

static unsigned int button_test_flag = 0;
static unsigned int button_test_press_flag = 0;
static unsigned int button_test_mode_switch_flag = 0;
static unsigned int online_ignore_flag = 0;

void gpio_click_notify(struct work_struct *work)
{
    ralink_gpio_notify_user(1);
}

void gpio_hold_notify(struct work_struct *work)
{
    ralink_gpio_notify_user(2);
}

int ralink_gpio_led_set(ralink_gpio_led_info led)
{
#ifdef CONFIG_RALINK_GPIO_LED
	unsigned long tmp;
	if (0 <= led.gpio && led.gpio < RALINK_GPIO_NUMBER) {
		if (led.on > RALINK_GPIO_LED_INFINITY)
			led.on = RALINK_GPIO_LED_INFINITY;
		if (led.off > RALINK_GPIO_LED_INFINITY)
			led.off = RALINK_GPIO_LED_INFINITY;
		if (led.blinks > RALINK_GPIO_LED_INFINITY)
			led.blinks = RALINK_GPIO_LED_INFINITY;
		if (led.rests > RALINK_GPIO_LED_INFINITY)
			led.rests = RALINK_GPIO_LED_INFINITY;
		if (led.times > RALINK_GPIO_LED_INFINITY)
			led.times = RALINK_GPIO_LED_INFINITY;
		if (led.on == 0 && led.off == 0 && led.blinks == 0 &&
				led.rests == 0) {
			ralink_gpio_led_data[led.gpio].gpio = -1; //stop it
			return 0;
		}
		//register led data
		ralink_gpio_led_data[led.gpio].gpio = led.gpio;
		ralink_gpio_led_data[led.gpio].on = (led.on == 0)? 1 : led.on;
		ralink_gpio_led_data[led.gpio].off = (led.off == 0)? 1 : led.off;
		ralink_gpio_led_data[led.gpio].blinks = (led.blinks == 0)? 1 : led.blinks;
		ralink_gpio_led_data[led.gpio].rests = (led.rests == 0)? 1 : led.rests;
		ralink_gpio_led_data[led.gpio].times = (led.times == 0)? 1 : led.times;

		//clear previous led status
		ralink_gpio_led_stat[led.gpio].ticks = -1;
		ralink_gpio_led_stat[led.gpio].ons = 0;
		ralink_gpio_led_stat[led.gpio].offs = 0;
		ralink_gpio_led_stat[led.gpio].resting = 0;
		ralink_gpio_led_stat[led.gpio].times = 0;

		printk("led=%d, on=%d, off=%d, blinks,=%d, reset=%d, time=%d\n",
				ralink_gpio_led_data[led.gpio].gpio,
				ralink_gpio_led_data[led.gpio].on,
				ralink_gpio_led_data[led.gpio].off,
				ralink_gpio_led_data[led.gpio].blinks,
				ralink_gpio_led_data[led.gpio].rests,
				ralink_gpio_led_data[led.gpio].times);
		//set gpio direction to 'out'
#if defined (RALINK_GPIO_HAS_2722)
		if (led.gpio <= 21) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp |= RALINK_GPIO(led.gpio);
			*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		}
		else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DIR));
			tmp |= RALINK_GPIO((led.gpio-22));
			*(volatile u32 *)(RALINK_REG_PIO2722DIR) = tmp;
		}
#elif defined (RALINK_GPIO_HAS_9532)
		if (led.gpio <= 31) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp |= RALINK_GPIO(led.gpio);
			*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		} else if (led.gpio <= 63) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
			tmp |= RALINK_GPIO((led.gpio-32));
			*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
		} else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
			tmp |= RALINK_GPIO((led.gpio-64));
			*(volatile u32 *)(RALINK_REG_PIO9564DIR) = tmp;
		}
#else
		if (led.gpio <= 23) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
			tmp |= RALINK_GPIO(led.gpio);
			*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		}
#if defined (RALINK_GPIO_HAS_4524)
		else if (led.gpio <= 39) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
			tmp |= RALINK_GPIO((led.gpio-24));
			*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		}
		else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DIR));
			tmp |= RALINK_GPIO((led.gpio-40));
			*(volatile u32 *)(RALINK_REG_PIO4540DIR) = tmp;
		}
#elif defined (RALINK_GPIO_HAS_5124)
		else if (led.gpio <= 39) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
			tmp |= RALINK_GPIO((led.gpio-24));
			*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		}
		else {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
			tmp |= RALINK_GPIO((led.gpio-40));
			*(volatile u32 *)(RALINK_REG_PIO5140DIR) = tmp;
		}
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
		else if (led.gpio <= 39) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
			tmp |= RALINK_GPIO((led.gpio-24));
			*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		}
		else if (led.gpio <= 71) {
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DIR));
			tmp |= RALINK_GPIO((led.gpio-40));
			*(volatile u32 *)(RALINK_REG_PIO7140DIR) = tmp;
		}
		else {
#if defined (RALINK_GPIO_HAS_7224)
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DIR));
			tmp |= RALINK_GPIO((led.gpio-72));
			*(volatile u32 *)(RALINK_REG_PIO72DIR) = tmp;
#else
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DIR));
			tmp |= RALINK_GPIO((led.gpio-72));
			*(volatile u32 *)(RALINK_REG_PIO9572DIR) = tmp;
#endif
		}
#endif
#endif
#if RALINK_LED_DEBUG
		printk("dir_%x gpio_%d - %d %d %d %d %d\n", tmp,
				led.gpio, led.on, led.off, led.blinks,
				led.rests, led.times);
#endif
	}
	else {
		printk(KERN_ERR NAME ": gpio(%d) out of range\n", led.gpio);
		return -1;
	}
	return 0;
#else
	printk(KERN_ERR NAME ": gpio led support not built\n");
	return -1;
#endif
}
EXPORT_SYMBOL(ralink_gpio_led_set);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
long ralink_gpio_ioctl(struct file *file, unsigned int req,
		unsigned long arg)
#else
int ralink_gpio_ioctl(struct inode *inode, struct file *file, unsigned int req,
		unsigned long arg)
#endif
{
	unsigned long tmp;
	ralink_gpio_reg_info info;
#ifdef CONFIG_RALINK_GPIO_LED
	ralink_gpio_led_info led;
#endif

	req &= RALINK_GPIO_DATA_MASK;

	switch(req) {
	case RALINK_GPIO_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIODIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		break;
	case RALINK_GPIO_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIODIR) = tmp;
		break;
	case RALINK_GPIO_READ: //RALINK_GPIO_READ_INT
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO_WRITE: //RALINK_GPIO_WRITE_INT
		*(volatile u32 *)(RALINK_REG_PIODATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO_SET: //RALINK_GPIO_SET_INT
		*(volatile u32 *)(RALINK_REG_PIOSET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO_CLEAR: //RALINK_GPIO_CLEAR_INT
		*(volatile u32 *)(RALINK_REG_PIORESET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO_ENABLE_INTP:
		*(volatile u32 *)(RALINK_REG_INTENA) = cpu_to_le32(RALINK_INTCTL_PIO);
		break;
	case RALINK_GPIO_DISABLE_INTP:
		*(volatile u32 *)(RALINK_REG_INTDIS) = cpu_to_le32(RALINK_INTCTL_PIO);
		break;
	case RALINK_GPIO_REG_IRQ:
		copy_from_user(&info, (ralink_gpio_reg_info *)arg, sizeof(info));
		if (0 <= info.irq && info.irq < RALINK_GPIO_NUMBER) {
			ralink_gpio_info[info.irq].pid = info.pid;
#if defined (RALINK_GPIO_HAS_2722)
			if (info.irq <= 21) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIORENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIOFENA) = cpu_to_le32(tmp);
			}
			else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722RENA));
				tmp |= (0x1 << (info.irq-22));
				*(volatile u32 *)(RALINK_REG_PIO2722RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722FENA));
				tmp |= (0x1 << (info.irq-22));
				*(volatile u32 *)(RALINK_REG_PIO2722FENA) = cpu_to_le32(tmp);
			}
#elif defined (RALINK_GPIO_HAS_9532)
			if (info.irq <= 31) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIORENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIOFENA) = cpu_to_le32(tmp);
			} else if (info.irq <= 63) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332RENA));
				tmp |= (0x1 << (info.irq-32));
				*(volatile u32 *)(RALINK_REG_PIO6332RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332FENA));
				tmp |= (0x1 << (info.irq-32));
				*(volatile u32 *)(RALINK_REG_PIO6332FENA) = cpu_to_le32(tmp);
			} else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564RENA));
				tmp |= (0x1 << (info.irq-64));
				*(volatile u32 *)(RALINK_REG_PIO9564RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564FENA));
				tmp |= (0x1 << (info.irq-64));
				*(volatile u32 *)(RALINK_REG_PIO9564FENA) = cpu_to_le32(tmp);
			}
#else
			if (info.irq <= 23) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIORENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIORENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOFENA));
				tmp |= (0x1 << info.irq);
				*(volatile u32 *)(RALINK_REG_PIOFENA) = cpu_to_le32(tmp);
			}
#if defined (RALINK_GPIO_HAS_4524)
			else if (info.irq <= 39) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924RENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924FENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924FENA) = cpu_to_le32(tmp);
			}
			else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540RENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO4540RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540FENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO4540FENA) = cpu_to_le32(tmp);
			}
#elif defined (RALINK_GPIO_HAS_5124)
			else if (info.irq <= 39) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924RENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924FENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924FENA) = cpu_to_le32(tmp);
			}
			else {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140RENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO5140RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140FENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO5140FENA) = cpu_to_le32(tmp);
			}
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
			else if (info.irq <= 39) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924RENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924FENA));
				tmp |= (0x1 << (info.irq-24));
				*(volatile u32 *)(RALINK_REG_PIO3924FENA) = cpu_to_le32(tmp);
			}
			else if (info.irq <= 71) {
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140RENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO7140RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140FENA));
				tmp |= (0x1 << (info.irq-40));
				*(volatile u32 *)(RALINK_REG_PIO7140FENA) = cpu_to_le32(tmp);
			}
			else {
#if defined (RALINK_GPIO_HAS_7224)
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72RENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO72RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72FENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO72FENA) = cpu_to_le32(tmp);
#else
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572RENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO9572RENA) = cpu_to_le32(tmp);
				tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572FENA));
				tmp |= (0x1 << (info.irq-72));
				*(volatile u32 *)(RALINK_REG_PIO9572FENA) = cpu_to_le32(tmp);
#endif
			}
#endif
#endif
		}
		else
			printk(KERN_ERR NAME ": irq number(%d) out of range\n",
					info.irq);
		break;

#if defined (RALINK_GPIO_HAS_2722)
	case RALINK_GPIO2722_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO2722DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO2722_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO2722DIR) = tmp;
		break;
	case RALINK_GPIO2722_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO2722DIR) = tmp;
		break;
	case RALINK_GPIO2722_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO2722_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO2722DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO2722_SET:
		*(volatile u32 *)(RALINK_REG_PIO2722SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO2722_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO2722SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_9532)
	case RALINK_GPIO6332_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO6332DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO6332_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
		break;
	case RALINK_GPIO6332_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
		break;
	case RALINK_GPIO6332_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO6332_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO6332DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO6332_SET:
		*(volatile u32 *)(RALINK_REG_PIO6332SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO6332_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO6332SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO9564DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9564DIR) = tmp;
		break;
	case RALINK_GPIO9564_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9564DIR) = tmp;
		break;
	case RALINK_GPIO9564_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO9564_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO9564DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_SET:
		*(volatile u32 *)(RALINK_REG_PIO9564SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9564_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO9564SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_4524)
	case RALINK_GPIO3924_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO3924_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO3924DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;

	case RALINK_GPIO4540_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO4540DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO4540_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO4540DIR) = tmp;
		break;
	case RALINK_GPIO4540_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO4540DIR) = tmp;
		break;
	case RALINK_GPIO4540_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO4540_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO4540DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO4540_SET:
		*(volatile u32 *)(RALINK_REG_PIO4540SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO4540_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO4540SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_5124)
	case RALINK_GPIO3924_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO3924_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO3924DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;

	case RALINK_GPIO5140_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO5140DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO5140_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO5140DIR) = tmp;
		break;
	case RALINK_GPIO5140_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO5140DIR) = tmp;
		break;
	case RALINK_GPIO5140_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO5140_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO5140DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO5140_SET:
		*(volatile u32 *)(RALINK_REG_PIO5140SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO5140_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO5140SET) = cpu_to_le32(arg);
		break;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	case RALINK_GPIO3924_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO3924DIR) = tmp;
		break;
	case RALINK_GPIO3924_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO3924_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO3924DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_SET:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO3924_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO3924SET) = cpu_to_le32(arg);
		break;

	case RALINK_GPIO7140_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO7140DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO7140_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO7140DIR) = tmp;
		break;
	case RALINK_GPIO7140_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO7140DIR) = tmp;
		break;
	case RALINK_GPIO7140_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO7140_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO7140DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO7140_SET:
		*(volatile u32 *)(RALINK_REG_PIO7140SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO7140_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO7140SET) = cpu_to_le32(arg);
		break;
#if defined (RALINK_GPIO_HAS_7224)
	case RALINK_GPIO72_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO72DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO72_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO72DIR) = tmp;
		break;
	case RALINK_GPIO72_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO72DIR) = tmp;
		break;
	case RALINK_GPIO72_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO72_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO72DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO72_SET:
		*(volatile u32 *)(RALINK_REG_PIO72SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO72_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO72SET) = cpu_to_le32(arg);
		break;
#else
	case RALINK_GPIO9572_SET_DIR:
		*(volatile u32 *)(RALINK_REG_PIO9572DIR) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9572_SET_DIR_IN:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DIR));
		tmp &= ~cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9572DIR) = tmp;
		break;
	case RALINK_GPIO9572_SET_DIR_OUT:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DIR));
		tmp |= cpu_to_le32(arg);
		*(volatile u32 *)(RALINK_REG_PIO9572DIR) = tmp;
		break;
	case RALINK_GPIO9572_READ:
		tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572DATA));
		put_user(tmp, (int __user *)arg);
		break;
	case RALINK_GPIO9572_WRITE:
		*(volatile u32 *)(RALINK_REG_PIO9572DATA) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9572_SET:
		*(volatile u32 *)(RALINK_REG_PIO9572SET) = cpu_to_le32(arg);
		break;
	case RALINK_GPIO9572_CLEAR:
		*(volatile u32 *)(RALINK_REG_PIO9572SET) = cpu_to_le32(arg);
		break;
#endif
#endif

	case RALINK_GPIO_LED_SET:
#ifdef CONFIG_RALINK_GPIO_LED
		copy_from_user(&led, (ralink_gpio_led_info *)arg, sizeof(led));
		ralink_gpio_led_set(led);
#else
		printk(KERN_ERR NAME ": gpio led support not built\n");
#endif
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

int ralink_gpio_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	MOD_INC_USE_COUNT;
#else
	try_module_get(THIS_MODULE);
#endif
    INIT_WORK(&gpio_event_hold, gpio_hold_notify);
    INIT_WORK(&gpio_event_click, gpio_click_notify);
	return 0;
}

int ralink_gpio_release(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	MOD_DEC_USE_COUNT;
#else
	module_put(THIS_MODULE);
#endif
	return 0;
}

struct file_operations ralink_gpio_fops =
{
	owner:		THIS_MODULE,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
	unlocked_ioctl:	ralink_gpio_ioctl,
#else
	ioctl:		ralink_gpio_ioctl,
#endif
	open:		ralink_gpio_open,
	release:	ralink_gpio_release,
};

#ifdef CONFIG_RALINK_GPIO_LED

#if RALINK_GPIO_LED_LOW_ACT

#define __LED_ON(gpio)      ra_gpio_led_clr |= RALINK_GPIO(gpio);
#define __LED_OFF(gpio)     ra_gpio_led_set |= RALINK_GPIO(gpio);
#define __LED2722_ON(gpio)  ra_gpio2722_led_clr |= RALINK_GPIO((gpio-22));
#define __LED2722_OFF(gpio) ra_gpio2722_led_set |= RALINK_GPIO((gpio-22));
#define __LED3924_ON(gpio)  ra_gpio3924_led_clr |= RALINK_GPIO((gpio-24));
#define __LED3924_OFF(gpio) ra_gpio3924_led_set |= RALINK_GPIO((gpio-24));
#define __LED4540_ON(gpio)  ra_gpio4540_led_clr |= RALINK_GPIO((gpio-40));
#define __LED4540_OFF(gpio) ra_gpio4540_led_set |= RALINK_GPIO((gpio-40));
#define __LED5140_ON(gpio)  ra_gpio5140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED5140_OFF(gpio) ra_gpio5140_led_set |= RALINK_GPIO((gpio-40));
#define __LED7140_ON(gpio)  ra_gpio7140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED7140_OFF(gpio) ra_gpio7140_led_set |= RALINK_GPIO((gpio-40));

#if defined (RALINK_GPIO_HAS_7224)
#define __LED72_ON(gpio)  ra_gpio72_led_clr |= RALINK_GPIO((gpio-72));
#define __LED72_OFF(gpio) ra_gpio72_led_set |= RALINK_GPIO((gpio-72));
#else
#define __LED9572_ON(gpio)  ra_gpio9572_led_clr |= RALINK_GPIO((gpio-72));
#define __LED9572_OFF(gpio) ra_gpio9572_led_set |= RALINK_GPIO((gpio-72));
#endif

#if defined (RALINK_GPIO_HAS_9532)
#define __LED6332_ON(gpio)  ra_gpio6332_led_clr |= RALINK_GPIO((gpio-32));
#define __LED6332_OFF(gpio) ra_gpio6332_led_set |= RALINK_GPIO((gpio-32));
#define __LED9564_ON(gpio)  ra_gpio9564_led_clr |= RALINK_GPIO((gpio-64));
#define __LED9564_OFF(gpio) ra_gpio9564_led_set |= RALINK_GPIO((gpio-64));
#endif

#else

#define __LED_ON(gpio)      ra_gpio_led_set |= RALINK_GPIO(gpio);
#define __LED_OFF(gpio)     ra_gpio_led_clr |= RALINK_GPIO(gpio);
#define __LED2722_ON(gpio)  ra_gpio2722_led_set |= RALINK_GPIO((gpio-22));
#define __LED2722_OFF(gpio) ra_gpio2722_led_clr |= RALINK_GPIO((gpio-22));
#define __LED3924_ON(gpio)  ra_gpio3924_led_set |= RALINK_GPIO((gpio-24));
#define __LED3924_OFF(gpio) ra_gpio3924_led_clr |= RALINK_GPIO((gpio-24));
#define __LED4540_ON(gpio)  ra_gpio4540_led_set |= RALINK_GPIO((gpio-40));
#define __LED4540_OFF(gpio) ra_gpio4540_led_clr |= RALINK_GPIO((gpio-40));
#define __LED5140_ON(gpio)  ra_gpio5140_led_set |= RALINK_GPIO((gpio-40));
#define __LED5140_OFF(gpio) ra_gpio5140_led_clr |= RALINK_GPIO((gpio-40));
#define __LED7140_ON(gpio)  ra_gpio7140_led_set |= RALINK_GPIO((gpio-40));
#define __LED7140_OFF(gpio) ra_gpio7140_led_clr |= RALINK_GPIO((gpio-40));
#if defined (RALINK_GPIO_HAS_7224)
#define __LED72_ON(gpio)  ra_gpio72_led_set |= RALINK_GPIO((gpio-72));
#define __LED72_OFF(gpio) ra_gpio72_led_clr |= RALINK_GPIO((gpio-72));
#else
#define __LED9572_ON(gpio)  ra_gpio9572_led_set |= RALINK_GPIO((gpio-72));
#define __LED9572_OFF(gpio) ra_gpio9572_led_clr |= RALINK_GPIO((gpio-72));
#endif

#if defined (RALINK_GPIO_HAS_9532)
#define __LED6332_ON(gpio)  ra_gpio6332_led_set |= RALINK_GPIO((gpio-32));
#define __LED6332_OFF(gpio) ra_gpio6332_led_clr |= RALINK_GPIO((gpio-32));
#define __LED9564_ON(gpio)  ra_gpio9564_led_set |= RALINK_GPIO((gpio-64));
#define __LED9564_OFF(gpio) ra_gpio9564_led_clr |= RALINK_GPIO((gpio-64));
#endif


#endif

static void pollingGpio(void);

static void ralink_gpio_led_do_timer(unsigned long unused)
{
	static unsigned int pollingCount = 0;
	if (pollingCount > SYS_READY_TIME)
	{
		sys_ready = 1;
	}
	else
	{
		pollingCount++;
	}
	
	pollingGpio();
	
	init_timer(&ralink_gpio_led_timer);
	ralink_gpio_led_timer.expires = jiffies + RALINK_GPIO_LED_FREQ;
	add_timer(&ralink_gpio_led_timer);
}

void ralink_gpio_led_init_timer(void)
{
	int i;

	for (i = 0; i < RALINK_GPIO_NUMBER; i++)
		ralink_gpio_led_data[i].gpio = -1; //-1 means unused
#if RALINK_GPIO_LED_LOW_ACT
	ra_gpio_led_set = 0xffffffff;
#if defined (RALINK_GPIO_HAS_2722)
	ra_gpio2722_led_set = 0xff;
#elif defined (RALINK_GPIO_HAS_4524)
	ra_gpio3924_led_set = 0xffff;
	ra_gpio4540_led_set = 0xff;
#elif defined (RALINK_GPIO_HAS_5124)
	ra_gpio3924_led_set = 0xffff;
	ra_gpio5140_led_set = 0xfff;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	ra_gpio3924_led_set = 0xffff;
	ra_gpio7140_led_set = 0xffffffff;
#if defined (RALINK_GPIO_HAS_7224)
	ra_gpio72_led_set = 0xffffff;
#else
	ra_gpio9572_led_set = 0xffffff;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	ra_gpio6332_led_set = 0xffffffff;
	ra_gpio9564_led_set = 0xffffffff;
#endif
#else // RALINK_GPIO_LED_LOW_ACT //
	ra_gpio_led_clr = 0xffffffff;
#if defined (RALINK_GPIO_HAS_2722)
	ra_gpio2722_led_clr = 0xff;
#elif defined (RALINK_GPIO_HAS_4524)
	ra_gpio3924_led_clr = 0xffff;
	ra_gpio4540_led_clr = 0xff;
#elif defined (RALINK_GPIO_HAS_5124)
	ra_gpio3924_led_clr = 0xffff;
	ra_gpio5140_led_clr = 0xfff;
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	ra_gpio3924_led_clr = 0xffff;
	ra_gpio7140_led_clr = 0xffffffff;
#if defined (RALINK_GPIO_HAS_7224)
	ra_gpio72_led_clr = 0xffffff;
#else
	ra_gpio9572_led_clr = 0xffffff;
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	ra_gpio6332_led_clr = 0xffffffff;
	ra_gpio9564_led_clr = 0xffffffff;
#endif
#endif // RALINK_GPIO_LED_LOW_ACT //

	init_timer(&ralink_gpio_led_timer);
	ralink_gpio_led_timer.function = ralink_gpio_led_do_timer;
	ralink_gpio_led_timer.expires = jiffies + RALINK_GPIO_LED_FREQ;
	add_timer(&ralink_gpio_led_timer);
}
#endif

typedef enum  _GPIODIR
{
    GPIO_DIR_INPUT = 0,
    GPIO_DIR_OUTPUT = 1    
}GPIODIR;

void setGpioDir(u32 gpio, GPIODIR gpiodir)
{
    u32 tmp = 0;

    //printk("[setGpioDir] gpio %d, gpiodir %d\n", gpio,gpiodir);
    if (gpio >= 0 && gpio < 32)
    {
        tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIODIR));
        tmp &= ~(1 << (gpio - 0));
        tmp |= (gpiodir << (gpio - 0));
        *(volatile u32 *)(RALINK_REG_PIODIR) = tmp;        
    }
    else if (gpio >= 32 && gpio < 64)
    {
        tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR)); 
        tmp &= ~(1 << (gpio - 32));
        tmp |= (gpiodir << (gpio - 32));
        *(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;          
    }
    else if (gpio >= 64 && gpio < 96)
    {
        tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564DIR));
        tmp &= ~(1 << (gpio - 64));
        tmp |= (gpiodir << (gpio - 64));
        *(volatile u32 *)(RALINK_REG_PIO9564DIR) = tmp;         
    }
    else
    {
        printk("[error]gpio range error: %d.\n", gpio);
    }
    
}

int setGpioData(u32 gpio, u32 data)
{
	u32 bit = 0;
	u32 reg = 0;
	u32 tmp = 0;
	/* Get reg and bit of the reg */

    /* printk("[%s:%d]#######gpio %d, data %d.\n", __FUNCTION__, __LINE__, gpio, data); */
    
	if (gpio > 95)
	{
		printk(KERN_ERR NAME ": %s, Unsupport GPIO(%d)\n", __FUNCTION__, gpio);
		return -1;
	}
	if (gpio <= 31)
	{
		reg = RALINK_REG_PIODATA;
		bit = (1 << gpio);
	}
	else if (gpio <= 63)
	{
		/* RALINK_REG_PIO3924DATA for GPIO 32~63 */
		reg = RALINK_REG_PIO6332DATA;
		bit = (1 << (gpio - 32));
	}
	else if (gpio <= 95)
	{
		/* RALINK_REG_PIO7140DATA for GPIO 64~95 */
		reg = RALINK_REG_PIO9564DATA;
		bit = (1 << (gpio - 64));
	}

	/* Set to reg base on bit and data */
	tmp = le32_to_cpu(*(volatile u32 *)(reg));
	if (0 == data)
		tmp &= ~bit;
	else
		tmp |= bit;
	*(volatile u32 *)(reg) = tmp;
	return 0;
}

int getGpioData(u32 gpio, u32 *data)
{
	u32 bit = 0;
	u32 reg = 0;
	u32 tmp = 0;
	/* Get reg and bit of the reg */
	if (gpio > 95)
	{
		printk(KERN_ERR NAME ": %s, Unsupport GPIO(%d)\n", __FUNCTION__, gpio);
		return -1;
	}
	if (gpio <= 31)
	{
		/* RALINK_REG_PIODATA for GPIO 0~31 */
		reg = RALINK_REG_PIODATA;
		bit = (1 << gpio);
	}
	else if (gpio <= 63)
	{
		/* RALINK_REG_PIO3924DATA for GPIO 32~63 */
		reg = RALINK_REG_PIO6332DATA;
		bit = (1 << (gpio - 32));
	}
	else if (gpio <= 95)
	{
		/* RALINK_REG_PIO7140DATA for GPIO 64~95 */
		reg = RALINK_REG_PIO9564DATA;
		bit = (1 << (gpio - 64));
	}

	/* Get to reg base on bit */
	tmp = le32_to_cpu(*(volatile u32 *)(reg));
	if (bit & tmp)
		*data = 1;
	else
		*data = 0;
	
	return 0;
}

enum GPIO_ACT_TYPE
{
	GPIO_ACT_OFF	= 0,
	GPIO_ACT_ON		= 1,
	GPIO_ACT_BLINK	= 2,
	GPIO_ACT_BLINK_RAPID = 3,
	
	/* CONFIG_SINGLE_LED_ORANGE_GREEN add ACT  */
	GPIO_ACT_ON_ORANGE = 4,
	GPIO_ACT_LOCK = 8,
	GPIO_ACT_UNLOCK = 16,
	GPIO_ACT_PRIOTY_MIDDLE=32,
	GPIO_ACT_PRIOTY_HIGH=64,
};

enum GPIO_FREG_TYPE
{
	/* single bit num, like b0001, b0010, b0100, but not b0101 */
	GPIO_FREQ_NO	= 0,
	GPIO_FREQ_FAST	= 1,
	GPIO_FREQ_NORMAL= 4,
	GPIO_FREQ_SLOW = 8
};

typedef struct _TP_LED_CELL
{
	u32 gpioId;
	u32 gpioAction;
	u32 gpioFreq;
	
	u32 gpioStatus;
	u32 gpioTimes;
}TP_LED_CELL;



void initLedData(TP_LED_CELL *pGpio, u32 gpio);
void initLedData_ON(TP_LED_CELL *pGpio, u32 gpio);

#define GPIO_BTN_RST 						GPIO_BTN_WPS
#define GPIO_LED_SYSTEM 					GPIO_LED_INTERNET
#define SYSTEM_LED_BLINK_PERIOD 			1200
#define SYSTEM_LED_BLINK_PERIOD_LINKDOWN 	500
#define SYSTEM_LED_BLINK_PERIOD_RST 		500
#define LED_NUM 							4

#define STA_WPS_SUPPORT

/* GPIO Configuration:
  * GPIO38: WPS/RESET Button
  * GPIO39: LAN_led, yellowGreen: 39 low
  * GPIO41, GPIO43: Internet_led, yellowGreen: 41 low, 43 high, orangeRed: 41 high, 43 highZ
  * GPIO44: WIFI_led, yellowGreen: 44 low
  */
enum GPIO_COMMON {
	GPIO_LED_INTERNET 			= 0,
	GPIO_LED_LAN 				= 39,
	GPIO_LED_INTERNET_GREEN 	= 40,
	GPIO_LED_INTERNET_ORANGE 	= 42,
	GPIO_LED_WLAN 				= 44, 
	GPIO_BTN_WPS 				= 38,
	GPIO_LED_41 				= 41,
	GPIO_LED_43 				= 43,
};

#define GPIO_LED_SINGLE 		GPIO_LED_41
#define WLAN_BUTTON_GPIO 		(37)

TP_LED_CELL c2_led[LED_NUM];

void initGpioMode_W8(void)
{
    u32 gpiomode;

	u32 gpiomode2;
	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_REG_GPIOMODE));
	gpiomode2 = le32_to_cpu(*(volatile u32 *)(RALINK_REG_GPIOMODE2));
	
	gpiomode |= (RALINK_GPIOMODE_REFCLK) | (RALINK_GPIOMODE_WDT) | (RALINK_GPIOMODE_PERST);
	
	*(volatile u32 *)(RALINK_REG_GPIOMODE) = cpu_to_le32(gpiomode);

	/*printk("gpiomode2 %08x.\n", gpiomode);*/
	gpiomode2 &= ((0xf << 12) | (0xf << 28));
	/*printk("gpiomode2 %08x.\n", gpiomode);*/
	gpiomode2 |= (0x555 | (0x555 << 16));
	/*printk("gpiomode2 %08x.\n", gpiomode);*/
	*(volatile u32 *)(RALINK_REG_GPIOMODE2) = cpu_to_le32(gpiomode2);
	
	return;
}

void initLedData(TP_LED_CELL *pGpio, u32 gpio)
{
	pGpio->gpioId = gpio;
	pGpio->gpioAction = GPIO_ACT_OFF;
	pGpio->gpioFreq = GPIO_FREQ_NO;
	pGpio->gpioStatus = GPIO_ACT_OFF;
	setGpioData(gpio, 1);
	pGpio->gpioTimes = 0;
}

void initLedData_ON(TP_LED_CELL *pGpio, u32 gpio)
{
	pGpio->gpioId = gpio;
	pGpio->gpioAction = GPIO_ACT_ON;
	pGpio->gpioFreq = GPIO_FREQ_NO;
	pGpio->gpioStatus = GPIO_ACT_ON;
	setGpioData(gpio, 0);
	pGpio->gpioTimes = 0;
}

int setGpioDataEx(u32 gpio, u32 data)
{
	u32 tmp;
	if (gpio == GPIO_LED_INTERNET)
	{
        if (GPIO_ACT_ON_ORANGE == data)
        {
            setGpioDir(GPIO_LED_INTERNET_ORANGE, GPIO_DIR_INPUT);
            setGpioDir(GPIO_LED_INTERNET_GREEN, GPIO_DIR_OUTPUT);
            setGpioData(GPIO_LED_INTERNET_GREEN, 1);
        }
        else if (GPIO_ACT_ON == data)
        {
            setGpioDir(GPIO_LED_INTERNET_ORANGE, GPIO_DIR_OUTPUT);
            setGpioDir(GPIO_LED_INTERNET_GREEN, GPIO_DIR_OUTPUT);
            setGpioData(GPIO_LED_INTERNET_ORANGE, 1);
            setGpioData(GPIO_LED_INTERNET_GREEN, 0);
        }
        else if (GPIO_ACT_OFF == data)
        {
            setGpioDir(GPIO_LED_INTERNET_ORANGE, GPIO_DIR_OUTPUT);
            setGpioDir(GPIO_LED_INTERNET_GREEN, GPIO_DIR_OUTPUT);
            setGpioData(GPIO_LED_INTERNET_ORANGE, 1);
            setGpioData(GPIO_LED_INTERNET_GREEN, 1);
        }
        else
        {
            printk("[ERROR]invalid Internet LED data or mode: %d.\n", data);
        }
	}
	else if (gpio == GPIO_LED_SINGLE)
	{
		/*
		printk("gpio %d data %d\n",  gpio, data);
		*/
		if ((data == GPIO_ACT_ON) || (data ==GPIO_ACT_OFF))
		{
			/* data == GPIO_ACT_ON: light, yellowGreen, 41 is low and 43 is high */
			/* data == GPIO_ACT_OFF: shut off, 41 and 43 is high*/
			setGpioData(GPIO_LED_41, (data == GPIO_ACT_OFF)?1:0);
			/* make sure GPIO_LED_43 is output */
			/* RALINK_REG_PIO6332DIR for GPIO 32~63 */
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
			if (!(tmp &  (1 << (GPIO_LED_43-32))))
			{
				tmp |= (1 << (GPIO_LED_43-32));
				*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
			}
			setGpioData(GPIO_LED_43, 1);
		}
		else if (data == GPIO_ACT_ON_ORANGE)
		{
			/* orangeRed, 41 is high and 43 is highZ. set 43 to input. */
			setGpioData(GPIO_LED_41, 1);
			/* set gpio43 to input */
			/* RALINK_REG_PIO6332DIR for GPIO 32~63 */
			tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332DIR));
			if (tmp &  (1 << (GPIO_LED_43-32)))
			{
				tmp &= ~((1 << (GPIO_LED_43-32)));
				*(volatile u32 *)(RALINK_REG_PIO6332DIR) = tmp;
			}
		}
	}
    else
    {
        setGpioData(gpio, !data);
    }
    
	return 0;
}

void gpio_common_init(void)
{
    u32 gpiomode, tmp;

    /* GPIO configuration:
     * GPIO38: WPS/RESET Button
     * GPIO39: LAN_led, yellowGreen: 39 low
     * GPIO40, GPIO42: Internet_led, yellowGreen: 42 high, 40 low, orangeRed: 40 high, 42 highZ
     * GPIO44: WIFI_led, yellowGreen: 44 low
     */

    /* GPIO Mode */
	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_SYSCTL_BASE + 0x60));
	gpiomode |= (1 << 14) | (1 << 16) | (1 << 18);
	*(volatile u32 *)(RALINK_SYSCTL_BASE + 0x60) = cpu_to_le32(gpiomode);
	
	gpiomode = le32_to_cpu(*(volatile u32 *)(RALINK_SYSCTL_BASE + 0x64));
	//printf("gpiomode2 %08x.\n", gpiomode);
	gpiomode &= ((0xf << 12) | (0xf << 28));
	//printf("gpiomode2 %08x.\n", gpiomode);
	gpiomode |= (0x555 | (0x555 << 16));
	//printf("gpiomode2 %08x.\n", gpiomode);
	*(volatile u32 *)(RALINK_SYSCTL_BASE + 0x64) = cpu_to_le32(gpiomode);	

	/* Set Direction to output */
	/* GPIO CTRL 1 for GPIO 32~64 OUTPUT */
	tmp = le32_to_cpu(*(volatile u32 *)(RALINK_PIO_BASE + 0x4));
	tmp |= ((1 << (39 - 32)) | (1 << (40 - 32)) | (1 << (41 - 32)) | (1 << (42 - 32)) | (1 << (43 - 32)) | (1 << (44 - 32)));
	*(volatile u32 *)(RALINK_PIO_BASE + 0x4) = tmp;
	
	/* GPIO CTRL 1 for GPIO 32~64 INPUT */
	tmp = le32_to_cpu(*(volatile u32 *)(RALINK_PIO_BASE + 0x4));
	tmp &= (~((1 << (38 - 32)) ));
	*(volatile u32 *)(RALINK_PIO_BASE + 0x4) = tmp;

	initLedData(&c2_led[0], GPIO_LED_LAN);
	initLedData(&c2_led[1], GPIO_LED_INTERNET);
	initLedData(&c2_led[2], GPIO_LED_WLAN);
	initLedData(&c2_led[3], GPIO_LED_41);
}

/* single bit num, like b0001, b0010, b0100, but not b0101
 * return 0 is ok
 */
u32 chkLedFreq(u32 freq)
{
	while (freq != 0)
	{
		if (freq == 1)
		{
			freq = 0;
		}
		else if ((freq & 1) == 1)
		{
			break;
		}
		freq = (freq >> 1);
	}

	if (freq != 0)
	{
		printk("LED Freq is not ok(%d)\n", freq);
	}
	return freq;
}

/* freq is the cycle time(in uints of ms) of flashing */
int setLedCfg(u32 action, u32 freq, u32 gpio)
{
	u32 count = 0;
	TP_LED_CELL *pGpio = c2_led;
	while(count < LED_NUM && (pGpio[count].gpioId != gpio))
	{
		count++;
	}
	if (count >= LED_NUM)
	{
		return -1;
	}
	u32 prioty = 0;
	u32 action_prioty=0;
	if (pGpio[count].gpioAction & GPIO_ACT_PRIOTY_MIDDLE)
	{
		prioty=1;
	}
	if (pGpio[count].gpioAction & GPIO_ACT_PRIOTY_HIGH)
	{
		prioty=2;
	}
	if (action & GPIO_ACT_PRIOTY_MIDDLE)
	{
		action_prioty=1;
	}
	if (action & GPIO_ACT_PRIOTY_HIGH)
	{
		action_prioty=2;
	}
		
	if ((pGpio[count].gpioAction & GPIO_ACT_LOCK) && (action_prioty<prioty))
	{
		return 0;
	}
	
	if (action & GPIO_ACT_UNLOCK)
	{
		pGpio[count].gpioAction &= ~GPIO_ACT_LOCK;
		action &= ~GPIO_ACT_UNLOCK;
	}
	
	if (action == GPIO_ACT_OFF || (sys_status == 1 && led_status == 0))
	{
		pGpio[count].gpioAction = GPIO_ACT_OFF;
		pGpio[count].gpioFreq = GPIO_FREQ_NO;
	}
	else 
	{
		if (action & GPIO_ACT_BLINK )
		{
			freq =freq /( 1000 * RALINK_GPIO_LED_FREQ / (HZ) );
			if (freq < 2 )
			{
				freq = 2;
			}
			
			pGpio[count].gpioFreq = freq;
			
		}
		else
		{
		pGpio[count].gpioFreq = GPIO_FREQ_NO;
		}

		pGpio[count].gpioAction = action;
	}
	return 0;
	
}

void led_setSysOn(void)
{
}
void led_setSysOff(void)
{
}
void led_setSysFlash(void)
{
	setLedCfg(GPIO_ACT_BLINK | GPIO_ACT_ON, SYSTEM_LED_BLINK_PERIOD, GPIO_LED_SYSTEM);
	setLedCfg(GPIO_ACT_BLINK | GPIO_ACT_ON, SYSTEM_LED_BLINK_PERIOD, GPIO_LED_SINGLE);
}

void led_setSysFlashUpgrade()
{
	setLedCfg(GPIO_ACT_BLINK|GPIO_ACT_ON|GPIO_ACT_LOCK|GPIO_ACT_PRIOTY_HIGH, SYSTEM_LED_BLINK_PERIOD, GPIO_LED_SYSTEM);
	setLedCfg(GPIO_ACT_BLINK|GPIO_ACT_ON|GPIO_ACT_LOCK|GPIO_ACT_PRIOTY_HIGH, SYSTEM_LED_BLINK_PERIOD, GPIO_LED_SINGLE);
}

void led_setSysFlashReset()
{
	setLedCfg(GPIO_ACT_BLINK|GPIO_ACT_ON|GPIO_ACT_LOCK|GPIO_ACT_PRIOTY_HIGH, SYSTEM_LED_BLINK_PERIOD_RST, GPIO_LED_SYSTEM);
	setLedCfg(GPIO_ACT_BLINK|GPIO_ACT_ON|GPIO_ACT_LOCK|GPIO_ACT_PRIOTY_HIGH, SYSTEM_LED_BLINK_PERIOD_RST, GPIO_LED_SINGLE);
}

void led_setSysFlashNoCal()
{
	setLedCfg(GPIO_ACT_BLINK|GPIO_ACT_ON|GPIO_ACT_ON_ORANGE|GPIO_ACT_LOCK|GPIO_ACT_PRIOTY_HIGH, 
		SYSTEM_LED_BLINK_PERIOD, GPIO_LED_SYSTEM);
	setLedCfg(GPIO_ACT_BLINK|GPIO_ACT_ON|GPIO_ACT_ON_ORANGE|GPIO_ACT_LOCK|GPIO_ACT_PRIOTY_HIGH, 
		SYSTEM_LED_BLINK_PERIOD, GPIO_LED_SINGLE);
}

void led_setWlanOn(void)
{
    if (!sys_ready)
    {
        return;
    }
    setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_WLAN);
}

void led_setWlanOff(void)
{
	setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, GPIO_LED_WLAN);
}

void led_setLanBits(u32 lanBits)
{
	u32 bit = 0;
	u32 maxLanPortNum = 4;
	u32 ledGpioBase = 42; /* GPIO 42~39 LAN(1~4) leds */

	initGpioMode_W8();

	while(bit < maxLanPortNum)
	{
		if(lanBits & (1 << bit))
		{
			setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, ledGpioBase - bit);
		}
		else
		{
			setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, ledGpioBase - bit);
		}
		bit ++;
	}
}


void led_setLanOn(void)
{
    if (!sys_ready)
    {
        return;
    }
    setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_LAN);
}

void led_setLanOff(void)
{
    setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, GPIO_LED_LAN);
}

void led_setWanOn(u32 internet)
{
    switch (wl_mode)
    {
    case MULTIMODE_ROUTER_MODE:
    case MULTIMODE_HOTSPOT_MODE:
    	if (internet == 1 && wan_status == 1)
    	{
    		setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_INTERNET);
    	}
    	else
    	{
    	    /* wan linked, no internet */
    		if (internet == 0)
    		{
    			setLedCfg(GPIO_ACT_ON_ORANGE, GPIO_FREQ_NO, GPIO_LED_INTERNET);
    		}
    	}
        break;
    case MULTIMODE_AP_MODE:
    case MULTIMODE_REPEATER_MODE:
    	setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_INTERNET);
        break;
    default:
        printk("not support this mode %d.\n", wl_mode);
        break;
    }
	setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_SINGLE);
}

void led_setWanOff(u32 internet)
{
    switch (wl_mode)
    {
    case MULTIMODE_ROUTER_MODE:
        if (internet == 1)
        {
            setLedCfg(GPIO_ACT_ON_ORANGE, GPIO_FREQ_NO, GPIO_LED_INTERNET);
        }
        else
        {
            setLedCfg(GPIO_ACT_BLINK | GPIO_ACT_ON_ORANGE, SYSTEM_LED_BLINK_PERIOD_LINKDOWN, GPIO_LED_INTERNET);
        }
        break;
    case MULTIMODE_AP_MODE:
    case MULTIMODE_REPEATER_MODE:
    case MULTIMODE_HOTSPOT_MODE:
        setLedCfg(GPIO_ACT_ON_ORANGE, GPIO_FREQ_NO, GPIO_LED_INTERNET);
        break;
    default:
        printk("not support this mode %d.\n", wl_mode);
        break;
    }

	if (MULTIMODE_ROUTER_MODE == wl_mode && internet == 0)/*unlink*/
	{
		setLedCfg(GPIO_ACT_BLINK | GPIO_ACT_ON_ORANGE, SYSTEM_LED_BLINK_PERIOD_LINKDOWN, GPIO_LED_SINGLE);
	}
	else /*link but no internet*/
	{
	    setLedCfg(GPIO_ACT_ON_ORANGE, GPIO_FREQ_NO, GPIO_LED_SINGLE);
	}
}

void led_setWanFlash(void)
{
	setLedCfg(GPIO_ACT_BLINK | GPIO_ACT_ON_ORANGE, 500, GPIO_LED_SINGLE);
}

void led_setWpsOn(void)
{
    setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_WLAN);
}

void led_setWpsOff(void)
{
    setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, GPIO_LED_WLAN);
	setLedCfg(GPIO_ACT_UNLOCK | GPIO_ACT_OFF | GPIO_ACT_PRIOTY_MIDDLE, GPIO_FREQ_NO, GPIO_LED_SINGLE);
}

void led_setWpsFlash(u32 cycle)
{
	setLedCfg(GPIO_ACT_LOCK | GPIO_ACT_BLINK | GPIO_ACT_ON | GPIO_ACT_PRIOTY_MIDDLE, 
		cycle, GPIO_LED_SINGLE);
}

typedef irqreturn_t(*sc_callback_t)(int, void *, struct pt_regs *);

static sc_callback_t registered_cb = NULL;
static sc_callback_t registered_cb_5G = NULL;
static void *cb_arg;
static void *cb_arg_5G;

void register_simple_config_callback (void *callback, void *arg)
{
    registered_cb = (sc_callback_t) callback;
    cb_arg = arg;
}
void register_simple_config_callback_5G (void *callback, void *arg)
{
    registered_cb_5G = (sc_callback_t) callback;
    cb_arg_5G = arg;
}

EXPORT_SYMBOL(register_simple_config_callback);
EXPORT_SYMBOL(register_simple_config_callback_5G);

EXPORT_SYMBOL(led_setWpsOn);
EXPORT_SYMBOL(led_setWpsOff);
EXPORT_SYMBOL(led_setWpsFlash);

/* return 0 means Test OK */
u32 (*fp_gpioTestLan)(void) = NULL;
EXPORT_SYMBOL(fp_gpioTestLan);
u32 (*fp_ethTestWan)(void) = NULL;
EXPORT_SYMBOL(fp_ethTestWan);
u32 led_get_add_hw_version()
{
	return hw_version;
}
EXPORT_SYMBOL(led_get_add_hw_version);


int resetCount = 0;
int wlanCount = 0;
int clickCount = 0;

extern int spi_flash_erase_config(void);

static void pollingGpio(void)
{
	u32 count = 0;
	TP_LED_CELL *pGpio = c2_led;
	u32 mask;
	static u32 cycCount = 0;
	u32 buttonStat = 0;
	static u32 isReset = 0;
	static u32 isWlan = 0;
	/* Part 1, Do LED Display */
	for (count = 0; count < LED_NUM; count++)
	{
		/*printk("GPIO %d, act %d, freq %d, status %d, times %d\n",
				pGpio[count].gpioId, 
				pGpio[count].gpioAction,
				pGpio[count].gpioFreq,
				pGpio[count].gpioStatus,
				pGpio[count].gpioTimes);*/
        
		if (pGpio[count].gpioAction ==  GPIO_ACT_OFF)
		{
			/* turn off */       
			setGpioDataEx(pGpio[count].gpioId, GPIO_ACT_OFF);

			pGpio[count].gpioStatus = GPIO_ACT_OFF;
			pGpio[count].gpioTimes = 0;
		}
		else
		{
			u32 mode = pGpio[count].gpioStatus;
			u32 nextMode = GPIO_ACT_OFF;
			if (pGpio[count].gpioAction & GPIO_ACT_BLINK)
			{
				if ((pGpio[count].gpioAction & GPIO_ACT_ON) && (pGpio[count].gpioAction & GPIO_ACT_ON_ORANGE))
				{
					nextMode = (mode == GPIO_ACT_ON)?GPIO_ACT_ON_ORANGE:GPIO_ACT_ON;
				}
				else if (pGpio[count].gpioAction & GPIO_ACT_ON)
				{
					nextMode = (mode == GPIO_ACT_OFF)?GPIO_ACT_ON:GPIO_ACT_OFF;
				}
				else if (pGpio[count].gpioAction & GPIO_ACT_ON_ORANGE)
				{
					nextMode = (mode == GPIO_ACT_OFF)?GPIO_ACT_ON_ORANGE:GPIO_ACT_OFF;
				}
				if (pGpio[count].gpioFreq == 0)
				{
					/* turn off */         
			        setGpioDataEx(pGpio[count].gpioId, mode);
                   
					pGpio[count].gpioStatus = mode;
					pGpio[count].gpioTimes = 0;
				}
				else
				{	
					if (pGpio[count].gpioTimes >= (pGpio[count].gpioFreq >>1) )
					{
						pGpio[count].gpioTimes = 0;
						/*printk("GPIO %d, mask %d, gpioTimes %d\n", pGpio[count].gpioId, mask, pGpio[count].gpioTimes);*/        
			            setGpioDataEx(pGpio[count].gpioId, nextMode);
						
						pGpio[count].gpioStatus = nextMode;
					}
					pGpio[count].gpioTimes++;
				}
			}
			else
			{
				if (pGpio[count].gpioAction & GPIO_ACT_ON)
				{
					mode = GPIO_ACT_ON;
				}
				else if (pGpio[count].gpioAction & GPIO_ACT_ON_ORANGE)
				{
					mode = GPIO_ACT_ON_ORANGE;
				}        
			    setGpioDataEx(pGpio[count].gpioId, mode);
               
				pGpio[count].gpioStatus = mode;
				pGpio[count].gpioTimes = 0;
			}
		}
	}

	/* Part 2, Do GPIO Cycle Function */
	cycCount++;

	if (0 == (cycCount & 0xf))/* Freq = 5 * RALINK_GPIO_LED_FREQ = 0.5s */
	{
		if(6 == led_get_add_hw_version())
		{
			if (MULTIMODE_ROUTER_MODE == wl_mode) // not RE MODE && CLIENT MODE
			{
				if ((NULL != fp_ethTestWan) && (1 == fp_ethTestWan()))
				{
					if (wan_status == 1)//only check internet
					{
						led_setWanOn(0);
					}
					else
					{
						led_setWanOff(1);
					}
				}
				else
				{
					led_setWanOff(0);
				}
			}
			else if (MULTIMODE_HOTSPOT_MODE == wl_mode)
			{
				if (wan_status == 1)//only check internet
				{
					led_setWanOn(0);
				}
				else
				{
					led_setWanOff(0);
				}
			}
			else if (MULTIMODE_AP_MODE == wl_mode)
			{
				if (((NULL != fp_gpioTestLan) && (1 == fp_gpioTestLan())) || ((NULL != fp_ethTestWan) && (1 == fp_ethTestWan())))
				{
					led_setWanOn(0);
				}
				else /* No func or test failed */
				{
					led_setWanOff(0);
				}
			}
			else if ((MULTIMODE_REPEATER_MODE == wl_mode) || (MULTIMODE_CLIENT_MODE == wl_mode))
			{
				if (re_bridge_status == 2)
				{
					led_setWanOn(0);
				}
				else if (re_bridge_status == 1)
				{
					led_setWanFlash();
				}
				else /* No func or test failed */
				{
					led_setWanOff(0);
				}
			}
		}
		else
		{
			/* LAN Led Polling */
			if ((NULL != fp_gpioTestLan) && (1 == fp_gpioTestLan()))
				led_setLanOn();
			else
				led_setLanOff();

			/* WAN/Wlan Led Polling */
			if (MULTIMODE_REPEATER_MODE == wl_mode)
			{
				if (RE_BRIDGE_STATUS_BRIDGED == re_bridge_status)
					led_setWanOn(0);
				else
					led_setWanOff(0);
				
				if (wlan_24G_status)
					led_setWlanOn();
				else
					led_setWlanOff();
			}
			else
			{
				if ((NULL != fp_ethTestWan) && (1 == fp_ethTestWan()))
				{
					if (wan_status)
						led_setWanOn(1);
					else
					{
						if (MULTIMODE_ROUTER_MODE == wl_mode)
							led_setWanOff(1);
						else
							led_setWanOn(0);
					}
				}
				else
				{
					if (MULTIMODE_HOTSPOT_MODE == wl_mode)
					{
						if (wan_status)
							led_setWanOn(1);
						else
							led_setWanOff(0);
					}
					else
						led_setWanOff(0);
				}
				if (wlan_24G_status)
					led_setWlanOn();
				else
					led_setWlanOff();
			}

			if (sys_status == 0)
				led_setSysFlash();
			else
				led_setSysOn();
		}

	}

	/* Reset/WPS Button */
	if (0 == getGpioData(GPIO_BTN_RST, &buttonStat))
	{
		if (1 == buttonStat) /* ??Unpressed */
		{
			if (button_test_press_flag & (1 << RESET_BUTTON_PRESSED))
			{
				button_test_flag |= (1 << RESET_BUTTON_PRESSED);
			}

			/* clean Button stat */
			if (isReset == 1)
			{
				resetCount = 0;
				isReset = 0;               
			}

			if ((resetCount != 0) && (isReset == 0))
			{
				resetCount = 0;
				/* Do WPS */
				if (!(online_ignore_flag & (1 << WPS_BUTTON_PRESSED)))
				{
					printk("Call WPS now\n");
					if (registered_cb)
					{
						if (MULTIMODE_REPEATER_MODE == wl_mode ||
							MULTIMODE_CLIENT_MODE == wl_mode) // RE MODE && CLIENT MODE
						{
							printk("wps client 2.4G begin\n");
							registered_cb (1, cb_arg, NULL);
						}
						else
						{
							printk("wps 2.4G begin\n");
							registered_cb (0, cb_arg, NULL);
						}
						printk("wps 2.4G begin\n");
						
					}
					else
					{
						printk("register 2.4G func is NULL\n");
					}
				}
			}
		
		}
		else /* Pressed */
		{
			button_test_press_flag |= (1 << RESET_BUTTON_PRESSED);
			if (resetCount >= RESET_TRIGGER)
			{
				/* Do Reset */
				if (0 == isReset && !(online_ignore_flag & (1 << RESET_BUTTON_PRESSED)))/* Avoid more times */
				{
					if (sys_ready)
					{
						printk("Factory configuration restored...\n");
						spi_flash_erase_config();
						isReset = 1;
	                    char buf[] = "FACT_RESET";
	                    int len = sizeof(buf);
						send_gpiobtn_event_to_user(buf, len);
					}
					else
					{
						printk("System is on loading, please wait...\n");
					}

				}
			}
			else
			{	
				printk("resetCount ++ %d.\n", resetCount);
				resetCount++;    
			}
		}
	}
}

static int led_internet_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	return count;
}

static int led_internet_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val, val1;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d %d", &val, &val1) != 2)
	{
		printk("usage: <action> <unused>\n");
		return count;
	}

    wan_status = val;
	if (val && ((NULL != fp_ethTestWan) && (1 == fp_ethTestWan())))
	{
		led_setWanOn(1);
	}
	else
	{
		if ((NULL != fp_ethTestWan) && (1 == fp_ethTestWan()))
		{
			led_setWanOff(1);
		}
		else
		{
			led_setWanOff(0);
		}		
	}

	return count;
}


static int led_wlan_24G_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	return count;
}

static int led_wlan_24G_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}
	wlan_24G_status = val;

	if (wlan_24G_status)
	{
		led_setWlanOn();
	}
	else
	{
		led_setWlanOff();
	}
	return count;
}

static int led_sys_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	return count;
}
static int led_sys_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{

	char val_string[16];
	int val;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}
	sys_status = val;
	if (sys_status)
	{
		if (sys_status == 2)
			led_setSysFlashUpgrade();
		else if (sys_status == 3)
			led_setSysFlashNoCal();
		else if (sys_status == 4)
			led_setSysFlashReset();
		else
			led_setSysOn();
	}
	else
	{
		led_setSysFlash();/*flash*/
	}

	return count;
}

static int button_test_flag_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	int len;
	
	len = sprintf(page, "%d\n", button_test_flag);

	len -= off; *start = page + off;
	if (len > count)
		len = count;
	else
		*eof = 1;
	if (len < 0)
		len = 0;

	return len;
}

static int button_test_flag_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}

	button_test_flag = val;
	return count;
}


static int online_ignore_flag_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	int len;
	
	len = sprintf(page, "%d\n", online_ignore_flag);

	len -= off; *start = page + off;
	if (len > count)
		len = count;
	else
		*eof = 1;
	if (len < 0)
		len = 0;

	return len;
}

static int online_ignore_flag_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val, val1;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}

	online_ignore_flag = val;
	return count;
}


static int sys_ready_flag_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	int len;
	
	len = sprintf(page, "%d\n", sys_ready);

	len -= off; *start = page + off;
	if (len > count)
		len = count;
	else
		*eof = 1;
	if (len < 0)
		len = 0;

	return len;
}

static int sys_ready_flag_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val, val1;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}

	sys_ready = val;
	return count;
}

static int add_hw_ver_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	int len;
	
	len = sprintf(page, "%d\n", hw_version);

	len -= off; *start = page + off;
	if (len > count)
		len = count;
	else
		*eof = 1;
	if (len < 0)
		len = 0;

	return len;
}

static int add_hw_ver_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val, val1;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}

	hw_version = val;
	return count;
}


#ifdef STA_WPS_STATUS_SUPPORT
typedef enum
{
	WIFI_RSSI_NONE = 0,
	WIFI_RSSI_LOW,
	WIFI_RSSI_MEDIAN,
	WIFI_RSSI_HIGH
}WIFI_RSSI_VAL;

static int led_wps_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
	return count;
}
static int led_wps_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val, val1;
    int wps_status = 0;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}

    wps_status = val;
    switch (wps_status)
    {
    /* not connected, turn off */
    case WIFI_RSSI_NONE:
        setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_WPS_GREEN);
        setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_WPS_RED);
        break;
    /* connected, level 1, < -75dBm, red */
    case WIFI_RSSI_LOW:
        setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_WPS_GREEN);
        setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, GPIO_LED_WPS_RED);
        break;        
    /* connected, level 1, >= -75dBm, <=-40dBm, orange */
    case WIFI_RSSI_MEDIAN:
        setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, GPIO_LED_WPS_GREEN);
        setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, GPIO_LED_WPS_RED);
        break;
    /* connected, level 1, > -40dBm, green */
    case WIFI_RSSI_HIGH:
        setLedCfg(GPIO_ACT_OFF, GPIO_FREQ_NO, GPIO_LED_WPS_GREEN);
        setLedCfg(GPIO_ACT_ON, GPIO_FREQ_NO, GPIO_LED_WPS_RED);
        break;
    default:
        // TODO
        break;
    }

	return count;
}
#endif

static int wl_mode_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
    int len;
    
    len = sprintf(page, "%d\n", wl_mode);

    len -= off; *start = page + off;
    if (len > count)
        len = count;
    else
        *eof = 1;
    if (len < 0)
        len = 0;
    return len;
}

static int wl_mode_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[16];
	int val, val1;
	
	if (count > sizeof(val_string) - 1)
		return -EINVAL;
	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

	if (sscanf(val_string, "%d", &val) != 1)
	{
		printk("usage: <action>\n");
		return count;
	}

    wl_mode = val;

	return count;
}

static int led_status_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
    sprintf(page, "%d\n", led_status);
    *eof = 1;
    return strlen(page);

}

static int led_status_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[6] = {0};
	int val;
	
	if (count > sizeof(val_string) - 1)
    {
        count = sizeof(val_string) - 1;
    }

	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

    val = simple_strtoul(val_string, NULL, 10);
    led_status = (val != 0);

	return count;
}

static int onekey_re_status_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
    sprintf(page, "%d\n", onekey_re_pressed);
    *eof = 1;
    return strlen(page);
}

static int onekey_re_status_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[6] = {0};
	int val;
	
	if (count > sizeof(val_string) - 1)
    {
        count = sizeof(val_string) - 1;
    }

	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

    val = simple_strtoul(val_string, NULL, 10);
    onekey_re_pressed = (val != 0);

	return count;
}

static int hw_mode_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
    int len;
    
    len = sprintf(page, "%d\n", hw_mode);

    len -= off; *start = page + off;
    if (len > count)
        len = count;
    else
        *eof = 1;
    if (len < 0)
        len = 0;
    return len;

}

static int hw_mode_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	printk("/proc/tplink/hw_mode is read only \n");
	return count;
}

static int re_bridge_status_read_proc(char *page, char **start, off_t off,	
	int count, int *eof, void *data)
{
    sprintf(page, "%d\n", re_bridge_status);
    *eof = 1;
    return strlen(page);
}

static int re_bridge_status_write_proc(struct file *file, const char *buffer,	
	unsigned long count, void *data)
{
	char val_string[6] = {0};
	int val;
	
	if (count > sizeof(val_string) - 1)
    {
        count = sizeof(val_string) - 1;
    }

	if (copy_from_user(val_string, buffer, count))
		return -EFAULT;

    val = simple_strtoul(val_string, NULL, 10);
	if (val >= RE_BRIDGE_STATUS_IDLE || val <= RE_BRIDGE_STATUS_BRIDGED)
	{
		re_bridge_status = val;
	}
	else
	{
		re_bridge_status = RE_BRIDGE_STATUS_IDLE;
	}
	return count;
}

static int rtnetlink_fill_info(struct sk_buff *skb, int type, char *data, int data_len)
{	
	struct ifinfomsg *r;	
	struct nlmsghdr  *nlh;
	unsigned char	 *b = skb_tail_pointer(skb);
	nlh = NLMSG_PUT(skb, 0, 0, type, sizeof(*r));
	r = NLMSG_DATA(nlh);	
	r->ifi_family = AF_UNSPEC;	
	r->__ifi_pad = 0;	
	r->ifi_type = 0;	
	r->ifi_index = 0;	
	r->ifi_flags = 0;	
	r->ifi_change = 0;	
	/* Wireless changes don't affect those flags */	
	/* Add the wireless events in the netlink packet */	
	RTA_PUT(skb, IFLA_WIRELESS, data_len, data);	
	nlh->nlmsg_len = skb_tail_pointer(skb) - b;	
	return skb->len;
	nlmsg_failure:
	rtattr_failure:	
	//nlmsg_trim(skb, b);
	return -1;
}

#ifndef BUF_SIZE_RTNL
#define BUF_SIZE_RTNL 	256
#endif

void send_gpiobtn_event_to_user(char *buf, int len)
{
	unsigned int size = BUF_SIZE_RTNL;
	int ret = 0;

	struct sk_buff *skb = alloc_skb(size, GFP_ATOMIC);

	if (skb == NULL)
	{
		printk("no enough memory!\n");
		return;
	}

	if (rtnetlink_fill_info(skb, RTM_NEWLINK,
				  buf, len) < 0) 
	{
		printk("fill reset info error!\n");
		kfree_skb(skb);
		return;
	}
	//struct net *net = sock_net(skb->sk);
	struct net *net = dev_net(skb->dev);
	rtnl_notify(skb, net, 0, RTNLGRP_LINK, NULL, GFP_ATOMIC);
	/*ret = rtnl_notify(skb, 0, RTNLGRP_LINK, NULL, GFP_ATOMIC);
	if (ret)
	{
		printk("Err to send\n");
	}*/
	return;
	
}


void init_gpio_tplink(void)
{
	struct proc_dir_entry *led_proc;
	
    led_proc = create_proc_entry("tplink/led_status", 0, NULL);
    led_proc->read_proc = led_status_read_proc;
    led_proc->write_proc = led_status_write_proc;

	led_proc = create_proc_entry("tplink/led_internet",  0, NULL);
	led_proc->read_proc = led_internet_read_proc;
	led_proc->write_proc = led_internet_write_proc;

	led_proc = create_proc_entry("tplink/led_wlan_24G",  0, NULL);
	led_proc->read_proc = led_wlan_24G_read_proc;
	led_proc->write_proc = led_wlan_24G_write_proc;
	
	led_proc = create_proc_entry("tplink/led_sys",  0, NULL);
	led_proc->read_proc = led_sys_read_proc;
	led_proc->write_proc = led_sys_write_proc;

#ifdef STA_WPS_STATUS_SUPPORT
    led_proc = create_proc_entry("tplink/led_wps",  0, NULL);
    led_proc->read_proc = led_wps_read_proc;
    led_proc->write_proc = led_wps_write_proc;
#endif

    led_proc = create_proc_entry("tplink/wl_mode",  0, NULL);
    led_proc->read_proc = wl_mode_read_proc;
    led_proc->write_proc = wl_mode_write_proc;

    led_proc = create_proc_entry("tplink/onekey_re_status", 0, NULL);
    led_proc->read_proc = onekey_re_status_read_proc;
    led_proc->write_proc = onekey_re_status_write_proc;

    led_proc = create_proc_entry("tplink/re_bridge_status", 0, NULL);
    led_proc->read_proc = re_bridge_status_read_proc;
    led_proc->write_proc = re_bridge_status_write_proc;
    
	led_proc = create_proc_entry("tplink/hw_mode",  0, NULL);
    led_proc->read_proc = hw_mode_read_proc;
    led_proc->write_proc = hw_mode_write_proc;

    led_proc = create_proc_entry("tplink/button_test_flag",  0, NULL);
    led_proc->read_proc = button_test_flag_read_proc;
    led_proc->write_proc = button_test_flag_write_proc;

    led_proc = create_proc_entry("tplink/online_ignore_flag",  0, NULL);
    led_proc->read_proc = online_ignore_flag_read_proc;
    led_proc->write_proc = online_ignore_flag_write_proc;

	led_proc = create_proc_entry("tplink/sys_ready",  0, NULL);
	led_proc->read_proc = sys_ready_flag_read_proc;
	led_proc->write_proc = sys_ready_flag_write_proc;

	led_proc = create_proc_entry("tplink/add_hw_version",  0, NULL);
	led_proc->read_proc = add_hw_ver_read_proc;
	led_proc->write_proc = add_hw_ver_write_proc;

	gpio_common_init();
}


int __init ralink_gpio_init(void)
{
	unsigned int i;

#ifdef  CONFIG_DEVFS_FS
	if (devfs_register_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME,
				&ralink_gpio_fops)) {
		printk(KERN_ERR NAME ": unable to register character device\n");
		return -EIO;
	}
	devfs_handle = devfs_register(NULL, RALINK_GPIO_DEVNAME,
			DEVFS_FL_DEFAULT, ralink_gpio_major, 0,
			S_IFCHR | S_IRUGO | S_IWUGO, &ralink_gpio_fops, NULL);
#else
	int r = 0;
	r = register_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME,
			&ralink_gpio_fops);
	if (r < 0) {
		printk(KERN_ERR NAME ": unable to register character device\n");
		return r;
	}
	if (ralink_gpio_major == 0) {
		ralink_gpio_major = r;
		printk(KERN_DEBUG NAME ": got dynamic major %d\n", r);
	}
#endif

	init_gpio_tplink();

	//enable gpio interrupt
	*(volatile u32 *)(RALINK_REG_INTENA) = cpu_to_le32(RALINK_INTCTL_PIO);
	for (i = 0; i < RALINK_GPIO_NUMBER; i++) {
		ralink_gpio_info[i].irq = i;
		ralink_gpio_info[i].pid = 0;
	}	

#ifdef CONFIG_RALINK_GPIO_LED
	ralink_gpio_led_init_timer();
#endif
	printk("Ralink gpio driver initialized\n");
	return 0;
}

void __exit ralink_gpio_exit(void)
{
#ifdef  CONFIG_DEVFS_FS
	devfs_unregister_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME);
	devfs_unregister(devfs_handle);
#else
	unregister_chrdev(ralink_gpio_major, RALINK_GPIO_DEVNAME);
#endif

	//config these pins to normal mode
	*(volatile u32 *)(RALINK_REG_GPIOMODE) &= ~RALINK_GPIOMODE_DFT;
	//disable gpio interrupt
	*(volatile u32 *)(RALINK_REG_INTDIS) = cpu_to_le32(RALINK_INTCTL_PIO);
#ifdef CONFIG_RALINK_GPIO_LED
	del_timer(&ralink_gpio_led_timer);
#endif
	printk("Ralink gpio driver exited\n");
}

/*
 * send a signal(SIGUSR1) to the registered user process whenever any gpio
 * interrupt comes
 * (called by interrupt handler)
 */
void ralink_gpio_notify_user(int usr)
{
	struct task_struct *p = NULL;

	if (ralink_gpio_irqnum < 0 || RALINK_GPIO_NUMBER <= ralink_gpio_irqnum) {
		printk(KERN_ERR NAME ": gpio irq number out of range\n");
		return;
	}

	//don't send any signal if pid is 0 or 1
	if ((int)ralink_gpio_info[ralink_gpio_irqnum].pid < 2)
		return;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
	p = find_task_by_vpid(ralink_gpio_info[ralink_gpio_irqnum].pid);
#else
	p = find_task_by_pid(ralink_gpio_info[ralink_gpio_irqnum].pid);
#endif

	if (NULL == p) {
		printk(KERN_ERR NAME ": no registered process to notify\n");
		return;
	}

	if (usr == 1) {
		printk(KERN_NOTICE NAME ": sending a SIGUSR1 to process %d\n",
				ralink_gpio_info[ralink_gpio_irqnum].pid);
		send_sig(SIGUSR1, p, 0);
	}
	else if (usr == 2) {
		printk(KERN_NOTICE NAME ": sending a SIGUSR2 to process %d\n",
				ralink_gpio_info[ralink_gpio_irqnum].pid);
		send_sig(SIGUSR2, p, 0);
	}
}

/*
 * 1. save the PIOINT and PIOEDGE value
 * 2. clear PIOINT by writing 1
 * (called by interrupt handler)
 */
void ralink_gpio_save_clear_intp(void)
{
	ralink_gpio_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOINT));
	ralink_gpio_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIOEDGE));
	
#if defined (RALINK_GPIO_HAS_9532)	
	*(volatile u32 *)(RALINK_REG_PIOINT) = cpu_to_le32(0xFFFFFFFF);
	*(volatile u32 *)(RALINK_REG_PIOEDGE) = cpu_to_le32(0xFFFFFFFF);
#else
	*(volatile u32 *)(RALINK_REG_PIOINT) = cpu_to_le32(0x00FFFFFF);
	*(volatile u32 *)(RALINK_REG_PIOEDGE) = cpu_to_le32(0x00FFFFFF);
#endif
#if defined (RALINK_GPIO_HAS_2722)
	ralink_gpio2722_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722INT));
	ralink_gpio2722_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO2722EDGE));
	*(volatile u32 *)(RALINK_REG_PIO2722INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO2722EDGE) = cpu_to_le32(0x0000FFFF);
#elif defined (RALINK_GPIO_HAS_4524)
	ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
	ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
	*(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
	ralink_gpio4540_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540INT));
	ralink_gpio4540_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO4540EDGE));
	*(volatile u32 *)(RALINK_REG_PIO4540INT) = cpu_to_le32(0x00000FFF);
	*(volatile u32 *)(RALINK_REG_PIO4540EDGE) = cpu_to_le32(0x00000FFF);
#elif defined (RALINK_GPIO_HAS_5124)
	ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
	ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
	*(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
	ralink_gpio5140_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140INT));
	ralink_gpio5140_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO5140EDGE));
	*(volatile u32 *)(RALINK_REG_PIO5140INT) = cpu_to_le32(0x00000FFF);
	*(volatile u32 *)(RALINK_REG_PIO5140EDGE) = cpu_to_le32(0x00000FFF);
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	ralink_gpio3924_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924INT));
	ralink_gpio3924_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO3924EDGE));
	*(volatile u32 *)(RALINK_REG_PIO3924INT) = cpu_to_le32(0x0000FFFF);
	*(volatile u32 *)(RALINK_REG_PIO3924EDGE) = cpu_to_le32(0x0000FFFF);
	ralink_gpio7140_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140INT));
	ralink_gpio7140_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140EDGE));
	*(volatile u32 *)(RALINK_REG_PIO7140INT) = cpu_to_le32(0xFFFFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO7140EDGE) = cpu_to_le32(0xFFFFFFFF);
#if defined (RALINK_GPIO_HAS_7224)
	ralink_gpio72_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72INT));
	ralink_gpio72_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO72EDGE));
	*(volatile u32 *)(RALINK_REG_PIO72INT) = cpu_to_le32(0x00FFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO72EDGE) = cpu_to_le32(0x00FFFFFF);
#else
	ralink_gpio9572_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572INT));
	ralink_gpio9572_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9572EDGE));
	*(volatile u32 *)(RALINK_REG_PIO9572INT) = cpu_to_le32(0x00FFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO9572EDGE) = cpu_to_le32(0x00FFFFFF);
#endif
#elif defined (RALINK_GPIO_HAS_9532)
	ralink_gpio6332_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332INT));
	ralink_gpio6332_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO6332EDGE));
	*(volatile u32 *)(RALINK_REG_PIO6332INT) = cpu_to_le32(0xFFFFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO6332EDGE) = cpu_to_le32(0xFFFFFFFF);


	ralink_gpio9564_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564INT));
	ralink_gpio9564_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO9564EDGE));
	*(volatile u32 *)(RALINK_REG_PIO9564INT) = cpu_to_le32(0xFFFFFFFF);
	*(volatile u32 *)(RALINK_REG_PIO9564EDGE) = cpu_to_le32(0xFFFFFFFF);

#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
void ralink_gpio_irq_handler(unsigned int irq, struct irqaction *irqaction)
#else
irqreturn_t ralink_gpio_irq_handler(int irq, void *irqaction)
#endif
{
	struct gpio_time_record {
		unsigned long falling;
		unsigned long rising;
	};
	static struct gpio_time_record record[RALINK_GPIO_NUMBER];
	unsigned long now;
	int i;
	ralink_gpio_save_clear_intp();
	now = jiffies;
#if defined (RALINK_GPIO_HAS_2722)
	for (i = 0; i < 22; i++) {
		if (! (ralink_gpio_intp & (1 << i)))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio_edge & (1 << i)) { //rising edge
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
				/*
				 * If the interrupt comes in a short period,
				 * it might be floating. We ignore it.
				 */
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					//one click
					schedule_work(&gpio_event_click);
				}
				else {
					//press for several seconds
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else { //falling edge
			record[i].falling = now;
		}
		break;
	}
	for (i = 22; i < 28; i++) {
		if (! (ralink_gpio2722_intp & (1 << (i - 22))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio2722_edge & (1 << (i - 22))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					schedule_work(&gpio_event_click);
				}
				else {
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#elif defined (RALINK_GPIO_HAS_9532)
	for (i = 0; i < 32; i++) {
		if (! (ralink_gpio_intp & (1 << i)))
			continue;
			ralink_gpio_irqnum = i;
		if (ralink_gpio_edge & (1 << i)) { //rising edge
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
				/*
				 * If the interrupt comes in a short period,
				 * it might be floating. We ignore it.
				 */
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					//one click
					printk("one click\n");
					schedule_work(&gpio_event_click);
				}
				else {
					//press for several seconds
					printk("press for several seconds\n");
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else { //falling edge
			 record[i].falling = now;
		}
		break;
	}
	for (i = 32; i < 64; i++) {
		if (! (ralink_gpio6332_intp & (1 << (i - 32))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio6332_edge & (1 << (i - 32))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					schedule_work(&gpio_event_click);
				}
				else {
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 64; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio9564_intp & (1 << (i - 64))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio9564_edge & (1 << (i - 64))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					schedule_work(&gpio_event_click);
				}
				else {
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#else
	for (i = 0; i < 24; i++) {
		if (! (ralink_gpio_intp & (1 << i)))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio_edge & (1 << i)) { //rising edge
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
				/*
				 * If the interrupt comes in a short period,
				 * it might be floating. We ignore it.
				 */
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					//one click
					printk("i=%d, one click\n", i);
					schedule_work(&gpio_event_click);
				}
				else {
					//press for several seconds
					printk("i=%d, push several seconds\n", i);
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else { //falling edge
			record[i].falling = now;
		}
		break;
	}
#if defined (RALINK_GPIO_HAS_4524)
	for (i = 24; i < 40; i++) {
		if (! (ralink_gpio3924_intp & (1 << (i - 24))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio3924_edge & (1 << (i - 24))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					schedule_work(&gpio_event_click);
				}
				else {
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 40; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio4540_intp & (1 << (i - 40))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio4540_edge & (1 << (i - 40))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					schedule_work(&gpio_event_click);
				}
				else {
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#elif defined (RALINK_GPIO_HAS_5124)
	for (i = 24; i < 40; i++) {
		if (! (ralink_gpio3924_intp & (1 << (i - 24))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio3924_edge & (1 << (i - 24))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					schedule_work(&gpio_event_click);
				}
				else {
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 40; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio5140_intp & (1 << (i - 40))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio5140_edge & (1 << (i - 40))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					schedule_work(&gpio_event_click);
				}
				else {
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#elif defined (RALINK_GPIO_HAS_9524) || defined (RALINK_GPIO_HAS_7224)
	for (i = 24; i < 40; i++) {
		if (! (ralink_gpio3924_intp & (1 << (i - 24))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio3924_edge & (1 << (i - 24))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					schedule_work(&gpio_event_click);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
	for (i = 40; i < 72; i++) {
		if (! (ralink_gpio7140_intp & (1 << (i - 40))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio7140_edge & (1 << (i - 40))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					schedule_work(&gpio_event_click);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#if defined (RALINK_GPIO_HAS_7224)
	for (i = 72; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio72_intp & (1 << (i - 72))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio72_edge & (1 << (i - 72))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					schedule_work(&gpio_event_click);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#else
	for (i = 72; i < RALINK_GPIO_NUMBER; i++) {
		if (! (ralink_gpio9572_intp & (1 << (i - 72))))
			continue;
		ralink_gpio_irqnum = i;
		if (ralink_gpio9572_edge & (1 << (i - 72))) {
			if (record[i].rising != 0 && time_before_eq(now,
						record[i].rising + 40L)) {
			}
			else {
				record[i].rising = now;
				if (time_before(now, record[i].falling + 200L)) {
					printk("i=%d, one click\n", i);
					schedule_work(&gpio_event_click);
				}
				else {
					printk("i=%d, push several seconds\n", i);
					schedule_work(&gpio_event_hold);
				}
			}
		}
		else {
			record[i].falling = now;
		}
		break;
	}
#endif
#endif
#endif

	return IRQ_HANDLED;
}

struct irqaction ralink_gpio_irqaction = {
	.handler = ralink_gpio_irq_handler,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	.flags = IRQF_DISABLED,
#else
	.flags = SA_INTERRUPT,
#endif
	.name = "ralink_gpio",
};

void __init ralink_gpio_init_irq(void)
{
	setup_irq(SURFBOARDINT_GPIO, &ralink_gpio_irqaction);
}

module_init(ralink_gpio_init);
module_exit(ralink_gpio_exit);

