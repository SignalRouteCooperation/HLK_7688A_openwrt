/* 
 gukq modified @20160527 renamed CF-E350N to KT9671
 gukq modified @20160627 renamed CF-E355AC to KT9672
*/

#include <linux/init.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/ath9k_platform.h>
#include <linux/etherdevice.h>
#include <linux/ar8216_platform.h>
#include <linux/platform_data/phy-at803x.h>

#include <asm/mach-ath79/irq.h>
#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/ar71xx_regs.h>

#include "common.h"
#include "dev-ap9x-pci.h"
#include "dev-eth.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-usb.h"
#include "dev-wmac.h"
#include "dev-nfc.h"
#include "gpio.h"
#include "machtypes.h"
#include "pci.h"


// For HS-UART
static struct resource ar934x_hs_uart_resources[] = {
	{
		.start	= (AR71XX_APB_BASE + 0x00500000),
		.end	= (AR71XX_APB_BASE + 0x00500000) + 0x100 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= ATH79_MISC_IRQ(6),
		.end	= ATH79_MISC_IRQ(6),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ar934x_hs_uart_device = {
	.name		= "atheros_hs_uart",
	.id			= 0,
	.resource	= ar934x_hs_uart_resources,
	.num_resources	= ARRAY_SIZE(ar934x_hs_uart_resources),
	.dev = {
		.platform_data	= NULL,
	},
};

extern void __iomem *ath79_gpio_base;
static void ath79_hs_uart_setup(int rx_gpio, int tx_gpio)
{
	// UART1 Out of Reset
	ath79_reset_wr(AR934X_RESET_REG_RESET_MODULE,
			ath79_reset_rr(AR934X_RESET_REG_RESET_MODULE) & ~MISC_INT_UART);
	// Set UART to operate on 100 MHz
	ath79_pll_wr(AR934X_PLL_SWITCH_CLOCK_CONTROL_REG,
			ath79_pll_rr(AR934X_PLL_SWITCH_CLOCK_CONTROL_REG) | BIT(7));

	__raw_writel(__raw_readl(ath79_gpio_base + AR71XX_GPIO_REG_OE)| (1 << rx_gpio),
                     ath79_gpio_base + AR71XX_GPIO_REG_OE);

	__raw_writel((rx_gpio) << 16, ath79_gpio_base + 0x68);
	ath79_gpio_output_select(tx_gpio, 79);

	platform_device_register(&ar934x_hs_uart_device);
}
// End of HS-UART


// AP9341FE
#define AP9341FE_GPIO_HSUART_RX			12
#define	AP9341FE_GPIO_HSUART_TX			21
#define	AP9341FE_GPIO_HSUART_SEL		11
#define	AP9341FE_GPIO_BEACON_RST0		18
#define	AP9341FE_GPIO_BEACON_RST1		19

#define COMFAST_KEYS_POLL_INTERVAL	20
#define COMFAST_KEYS_DEBOUNCE_INTERVAL	(3 * COMFAST_KEYS_POLL_INTERVAL)

#define	AP9341FE_GPIO_XWDT_TRIGGER	16

#define	XWDT_AUTOFEED_DURATION		(HZ / 3)
static int gpio_external_wdt = -1;
static int wdt_timeout = -1, wdt_autofeed_count = 0;

static void watchdog_fire(unsigned long);
static struct timer_list watchdog_ticktock = TIMER_INITIALIZER(watchdog_fire, 0, 0);
static void external_wdt_toggle(void);

static void enable_external_wdt(int gpio)
{
	gpio_external_wdt = gpio;
	
	external_wdt_toggle();
	
	wdt_timeout = -1;
	mod_timer(&watchdog_ticktock, jiffies + XWDT_AUTOFEED_DURATION);
}

static void external_wdt_toggle(void)
{
	static u32 data;
	data ++;
	gpio_set_value(gpio_external_wdt, data & 0x01);
}

void ath79_external_wdt_disable(void)
{
	if(gpio_external_wdt >= 0) {
		wdt_timeout = -1;
		mod_timer(&watchdog_ticktock, jiffies + XWDT_AUTOFEED_DURATION);
	}
}
EXPORT_SYMBOL(ath79_external_wdt_disable);

void ath79_external_wdt_trigger(void)
{
	if(gpio_external_wdt >= 0) {
		//printk(KERN_ERR "XWDT TRIGGER\n");
		wdt_autofeed_count = 0;
		mod_timer(&watchdog_ticktock, jiffies + XWDT_AUTOFEED_DURATION);
	}
}
EXPORT_SYMBOL(ath79_external_wdt_trigger);

void ath79_external_wdt_set_timeout(int timeout)
{
	if(gpio_external_wdt >= 0) {
		wdt_timeout = timeout;
		external_wdt_toggle();
		//printk(KERN_ERR "XWDT SET TIMEOUT: %d\n", timeout);
	}
}
EXPORT_SYMBOL(ath79_external_wdt_set_timeout);

static void watchdog_fire(unsigned long data)
{
	if(wdt_timeout > 0) 
		wdt_autofeed_count++;
	
	if((wdt_timeout < 0) || (wdt_autofeed_count < wdt_timeout)) {
		//printk(KERN_ERR "XWDT AUTOFEED: %d\n", wdt_autofeed_count);
		external_wdt_toggle();
		mod_timer(&watchdog_ticktock, jiffies + XWDT_AUTOFEED_DURATION);
	}
}

static struct gpio_keys_button comfast_ap934x_gpio_keys[] __initdata = {
	{
		.desc			= "reset",
		.type			= EV_KEY,
		.code			= KEY_RESTART,
		.debounce_interval	= COMFAST_KEYS_DEBOUNCE_INTERVAL,
		.gpio			= 20,
		.active_low		= 1,
	}
};

static struct gpio_led comfast_ap934x_gpio_leds[] __initdata = {
	{
		.name		= "comfast:red",
		.gpio		= 2,
	}, {
		.name		= "comfast:green",
		.gpio		= 3,
	}, {
		.name		= "comfast:blue",
		.gpio		= 0,
	}
};

static void ext_lna_control_gpio_setup(int gpio_rx0, int gpio_rx1)
{
	ath79_gpio_output_select(gpio_rx0, AR934X_GPIO_OUT_EXT_LNA0);
	ath79_gpio_output_select(gpio_rx1, AR934X_GPIO_OUT_EXT_LNA1);
}


static void __init comfast_ap9341fe_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);

	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, AR934X_GPIO_FUNC_CLK_OBS4_EN);

	ath79_gpio_output_select(AP9341FE_GPIO_XWDT_TRIGGER, 0);
	enable_external_wdt(AP9341FE_GPIO_XWDT_TRIGGER);

	ath79_register_m25p80(NULL);

	ath79_register_leds_gpio(-1, ARRAY_SIZE(comfast_ap934x_gpio_leds),
				 comfast_ap934x_gpio_leds);
	ath79_register_gpio_keys_polled(-1, COMFAST_KEYS_POLL_INTERVAL,
                                        ARRAY_SIZE(comfast_ap934x_gpio_keys),
                                        comfast_ap934x_gpio_keys);
					
	ext_lna_control_gpio_setup(13, 14);

	ath79_setup_ar934x_eth_cfg(AR934X_ETH_CFG_SW_PHY_SWAP);

	ath79_register_mdio(1, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 2);

	/* GMAC0 is connected to the PHY0 of the internal switch */
	ath79_switch_data.phy4_mii_en = 1;
	ath79_switch_data.phy_poll_mask = BIT(0);
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_eth0_data.phy_mask = BIT(0);
	ath79_eth0_data.mii_bus_dev = &ath79_mdio1_device.dev;
	ath79_register_eth(0);

	/* GMAC1 is connected to the internal switch */
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_GMII;	
	
	ath79_register_eth(1);
	
	ath79_register_usb();

	ath79_register_wmac(art, NULL);
}
MIPS_MACHINE(ATH79_MACH_COMFAST_AP9341FE, "COMFAST-AP9341FE", "COMFAST AP9341FE", comfast_ap9341fe_setup);

#define CF_WR600N_GPIO_LED_LAN1	22
#define CF_WR600N_GPIO_LED_LAN2	11
#define CF_WR600N_GPIO_LED_LAN3	19
#define CF_WR600N_GPIO_LED_WAN1	18

#define	CF_WR600N_GPIO_XWDT_TRIGGER	20

static struct gpio_keys_button comfast_cf_wr600n_gpio_keys[] __initdata = {
	{
		.desc			= "reset",
		.type			= EV_KEY,
		.code			= KEY_RESTART,
		.debounce_interval	= COMFAST_KEYS_DEBOUNCE_INTERVAL,
		.gpio			= 16,	
		.active_low		= 1,
	}
};

static struct gpio_led comfast_cf_wr600n_gpio_leds[] __initdata = {
	{
		.name		= "comfast:red",
		.gpio		= 2,
	}, {
		.name		= "comfast:green",
		.gpio		= 3,
	}, {
		.name		= "comfast:blue",
		.gpio		= 0,
	}, {
		.name		= "cf_wr600n:green:lan1",
		.gpio		= CF_WR600N_GPIO_LED_LAN1,
		.active_low	= 1,
	}, {
		.name		= "cf_wr600n:green:lan2",
		.gpio		= CF_WR600N_GPIO_LED_LAN2,
		.active_low	= 1,
	}, {
		.name		= "cf_wr600n:green:lan3",
		.gpio		= CF_WR600N_GPIO_LED_LAN3,
		.active_low	= 1,
	}, {
		.name		= "cf_wr600n:green:wan1",
		.gpio		= CF_WR600N_GPIO_LED_WAN1,
		.active_low	= 1,
	},
};

static void __init comfast_cf_wr600n_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);

	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, AR934X_GPIO_FUNC_CLK_OBS4_EN);

//	ath79_gpio_output_select(CF_WR600N_GPIO_XWDT_TRIGGER, 0);
//	enable_external_wdt(CF_WR600N_GPIO_XWDT_TRIGGER);

	ath79_register_m25p80(NULL);

	ath79_gpio_output_select(CF_WR600N_GPIO_LED_LAN2, 0);
	
	ath79_register_leds_gpio(-1, ARRAY_SIZE(comfast_cf_wr600n_gpio_leds),
				 comfast_cf_wr600n_gpio_leds);
	ath79_register_gpio_keys_polled(-1, COMFAST_KEYS_POLL_INTERVAL,
                                        ARRAY_SIZE(comfast_cf_wr600n_gpio_keys),
                                        comfast_cf_wr600n_gpio_keys);
					
	ext_lna_control_gpio_setup(13, 14);
	
	ath79_register_mdio(1, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 2);

	/* GMAC0 is connected to the PHY0 of the internal switch */
	ath79_switch_data.phy4_mii_en = 1;
	ath79_switch_data.phy_poll_mask = BIT(4);
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_eth0_data.phy_mask = BIT(4);
	ath79_eth0_data.mii_bus_dev = &ath79_mdio1_device.dev;
	ath79_register_eth(0);

	/* GMAC1 is connected to the internal switch */
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_GMII;	
	
	ath79_register_eth(1);
	
	ath79_register_usb();

	ath79_register_wmac(art, NULL);
} 

MIPS_MACHINE(ATH79_MACH_COMFAST_CF_WR600N, "COMFAST-CF-WR600N", "COMFAST CF-WR600N", comfast_cf_wr600n_setup);

#define CF_WR605N_GPIO_LED_LAN1	22
#define CF_WR605N_GPIO_LED_LAN2	11
#define CF_WR605N_GPIO_LED_LAN3	19
#define CF_WR605N_GPIO_LED_WAN1	18

#define CF_WR605N_GPIO_LED_WAN_EXT	15
#define CF_WR605N_GPIO_LED_LAN_EXT	17
#define CF_WR605N_GPIO_LED_WIFI_EXT	20

#define	CF_WR605N_GPIO_XWDT_TRIGGER	20

static struct gpio_keys_button comfast_cf_wr605n_gpio_keys[] __initdata = {
	{
		.desc			= "reset",
		.type			= EV_KEY,
		.code			= KEY_RESTART,
		.debounce_interval	= COMFAST_KEYS_DEBOUNCE_INTERVAL,
		.gpio			= 16,	
		.active_low		= 1,
	}
};

static struct gpio_led comfast_cf_wr605n_gpio_leds[] __initdata = {
	{
		.name		= "comfast:red",
		.gpio		= 2,
	}, {
		.name		= "comfast:green",
		.gpio		= 3,
	}, {
		.name		= "comfast:blue",
		.gpio		= 0,
	}, {
		.name		= "cf_wr605n:green:lan1",
		.gpio		= CF_WR605N_GPIO_LED_LAN1,
		.active_low	= 1,
	}, {
		.name		= "cf_wr605n:green:lan2",
		.gpio		= CF_WR605N_GPIO_LED_LAN2,
		.active_low	= 1,
	}, {
		.name		= "cf_wr605n:green:lan3",
		.gpio		= CF_WR605N_GPIO_LED_LAN3,
		.active_low	= 1,
	}, {
		.name		= "cf_wr605n:green:wan1",
		.gpio		= CF_WR605N_GPIO_LED_WAN1,
		.active_low	= 1,
	}, {
		.name		= "cf_wr605n:white:wan",
		.gpio		= CF_WR605N_GPIO_LED_WAN_EXT,
		.active_low	= 1,
	}, {
		.name		= "cf_wr605n:white:lan",
		.gpio		= CF_WR605N_GPIO_LED_LAN_EXT,
		.active_low	= 1,
	}, {
		.name		= "cf_wr605n:white:wifi",
		.gpio		= CF_WR605N_GPIO_LED_WIFI_EXT,
		.active_low	= 1,
	},
};


static void __init comfast_cf_wr605n_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);

	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, AR934X_GPIO_FUNC_CLK_OBS4_EN);

//	ath79_gpio_output_select(CF_WR605N_GPIO_XWDT_TRIGGER, 0);
//	enable_external_wdt(CF_WR605N_GPIO_XWDT_TRIGGER);

	ath79_register_m25p80(NULL);

	ath79_gpio_output_select(CF_WR605N_GPIO_LED_LAN2, 0);
	
	ath79_gpio_output_select(CF_WR605N_GPIO_LED_WAN_EXT, 0);
	ath79_gpio_output_select(CF_WR605N_GPIO_LED_LAN_EXT, 0);
	ath79_gpio_output_select(CF_WR605N_GPIO_LED_WIFI_EXT, 0);
	
	ath79_register_leds_gpio(-1, ARRAY_SIZE(comfast_cf_wr605n_gpio_leds),
				 comfast_cf_wr605n_gpio_leds);
	ath79_register_gpio_keys_polled(-1, COMFAST_KEYS_POLL_INTERVAL,
                                        ARRAY_SIZE(comfast_cf_wr605n_gpio_keys),
                                        comfast_cf_wr605n_gpio_keys);
					
	ext_lna_control_gpio_setup(13, 14);
	
	ath79_register_mdio(1, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 2);

	/* GMAC0 is connected to the PHY0 of the internal switch */
	ath79_switch_data.phy4_mii_en = 1;
	ath79_switch_data.phy_poll_mask = BIT(4);
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_eth0_data.phy_mask = BIT(4);
	ath79_eth0_data.mii_bus_dev = &ath79_mdio1_device.dev;
	ath79_register_eth(0);

	/* GMAC1 is connected to the internal switch */
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_GMII;	
	
	ath79_register_eth(1);
	
	ath79_register_usb();

	ath79_register_wmac(art, NULL);
} 

MIPS_MACHINE(ATH79_MACH_COMFAST_CF_WR605N, "COMFAST-CF-WR605N", "COMFAST CF-WR605N", comfast_cf_wr605n_setup);

#define	CF_E316NV2_GPIO_XWDT_TRIGGER	16

static struct gpio_keys_button comfast_cf_e316nv2_gpio_keys[] __initdata = {
	{
		.desc			= "reset",
		.type			= EV_KEY,
		.code			= KEY_RESTART,
		.debounce_interval	= COMFAST_KEYS_DEBOUNCE_INTERVAL,
		.gpio			= 20,
		.active_low		= 1,
	}
};

static struct gpio_led comfast_cf_e316nv2_gpio_leds[] __initdata = {
	{
		.name		= "comfast:white:wifi",
		.gpio		= 12,
		.active_low	= 1,
	}, {
		.name		= "comfast:white:lan",
		.gpio		= 19,
		.active_low	= 1,
	}, {
		.name		= "comfast:white:wan",
		.gpio		= 17,
		.active_low	= 1,
	}, {
		.name		= "comfast:green",
		.gpio		= 3,
	}
};

static void __init comfast_cf_e316nv2_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);

	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, AR934X_GPIO_FUNC_CLK_OBS4_EN);

	ath79_gpio_output_select(CF_E316NV2_GPIO_XWDT_TRIGGER, 0);
	enable_external_wdt(CF_E316NV2_GPIO_XWDT_TRIGGER);

	ath79_register_m25p80(NULL);
	
	ath79_gpio_output_select(12, 0);
	ath79_gpio_output_select(17, 0);
	ath79_gpio_output_select(19, 0);

	ath79_register_leds_gpio(-1, ARRAY_SIZE(comfast_cf_e316nv2_gpio_leds),
				 comfast_cf_e316nv2_gpio_leds);
	ath79_register_gpio_keys_polled(-1, COMFAST_KEYS_POLL_INTERVAL,
                                        ARRAY_SIZE(comfast_cf_e316nv2_gpio_keys),
                                        comfast_cf_e316nv2_gpio_keys);
					
	ext_lna_control_gpio_setup(13, 14);

	ath79_setup_ar934x_eth_cfg(AR934X_ETH_CFG_SW_PHY_SWAP);

	ath79_register_mdio(1, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 2);

	/* GMAC0 is connected to the PHY0 of the internal switch */
	ath79_switch_data.phy4_mii_en = 1;
	ath79_switch_data.phy_poll_mask = BIT(0);
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_eth0_data.phy_mask = BIT(0);
	ath79_eth0_data.mii_bus_dev = &ath79_mdio1_device.dev;
	ath79_register_eth(0);

	/* GMAC1 is connected to the internal switch */
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_GMII;	
	
	ath79_register_eth(1);
	
	ath79_register_usb();

	ath79_register_wmac(art, NULL);
}
MIPS_MACHINE(ATH79_MACH_COMFAST_CF_E316NV2, "COMFAST-CF-E316NV2", "COMFAST CF-E316NV2", comfast_cf_e316nv2_setup);

#define	CF_E325N_GPIO_XWDT_TRIGGER	16

static struct gpio_keys_button comfast_cf_e325n_gpio_keys[] __initdata = {
	{
		.desc			= "reset",
		.type			= EV_KEY,
		.code			= KEY_RESTART,
		.debounce_interval	= COMFAST_KEYS_DEBOUNCE_INTERVAL,
		.gpio			= 20,
		.active_low		= 1,
	}
};

static struct gpio_led comfast_cf_e325n_gpio_leds[] __initdata = {
	{
		.name		= "comfast:red",
		.gpio		= 2,
	}, {
		.name		= "comfast:green",
		.gpio		= 3,
	}, {
		.name		= "comfast:blue",
		.gpio		= 0,
	}, {
		.name		= "comfast:brst0",
		.gpio		= AP9341FE_GPIO_BEACON_RST0,
		.active_low     = 1,	
	}, {
		.name		= "comfast:brst1",
		.gpio		= AP9341FE_GPIO_BEACON_RST1,
		.active_low     = 1,
	}, {
		.name		= "comfast:bsel",
		.gpio		= AP9341FE_GPIO_HSUART_SEL,
	},
};

static void __init comfast_cf_e325n_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);

	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, AR934X_GPIO_FUNC_CLK_OBS4_EN);

	ath79_gpio_output_select(CF_E325N_GPIO_XWDT_TRIGGER, 0);
	enable_external_wdt(CF_E325N_GPIO_XWDT_TRIGGER);
	
	ath79_gpio_output_select(AP9341FE_GPIO_HSUART_SEL, 0);
	ath79_gpio_output_select(AP9341FE_GPIO_BEACON_RST0, 0);
	ath79_gpio_output_select(AP9341FE_GPIO_BEACON_RST1, 0);
	
	ath79_hs_uart_setup(AP9341FE_GPIO_HSUART_RX, AP9341FE_GPIO_HSUART_TX);

	ath79_register_m25p80(NULL);

	ath79_register_leds_gpio(-1, ARRAY_SIZE(comfast_cf_e325n_gpio_leds),
				 comfast_cf_e325n_gpio_leds);
	ath79_register_gpio_keys_polled(-1, COMFAST_KEYS_POLL_INTERVAL,
                                        ARRAY_SIZE(comfast_cf_e325n_gpio_keys),
                                        comfast_cf_e325n_gpio_keys);
					
	ext_lna_control_gpio_setup(13, 14);

	ath79_setup_ar934x_eth_cfg(AR934X_ETH_CFG_SW_PHY_SWAP);

	ath79_register_mdio(1, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 2);

	/* GMAC0 is connected to the PHY0 of the internal switch */
	ath79_switch_data.phy4_mii_en = 1;
	ath79_switch_data.phy_poll_mask = BIT(0);
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_eth0_data.phy_mask = BIT(0);
	ath79_eth0_data.mii_bus_dev = &ath79_mdio1_device.dev;
	ath79_register_eth(0);

	/* GMAC1 is connected to the internal switch */
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_GMII;	
	
	ath79_register_eth(1);
	
	ath79_register_usb();

	ath79_register_wmac(art, NULL);
}
MIPS_MACHINE(ATH79_MACH_COMFAST_CF_E325N, "COMFAST-CF-E325N", "COMFAST CF-E325N", comfast_cf_e325n_setup);

#define CF_WR610N_GPIO_BTN_RESET	17

#define CF_WR610N_KEYS_POLL_INTERVAL	20	/* msecs */
#define CF_WR610N_KEYS_DEBOUNCE_INTERVAL (3 * CF_WR610N_KEYS_POLL_INTERVAL)

#define	CF_WR610N_GPIO_XWDT_TRIGGER	13

#define CF_WR610N_GPIO_LED_WAN		2
#define CF_WR610N_GPIO_LED_LAN		3
#define CF_WR610N_GPIO_LED_WLAN		0

#define CF_WR610N_GPIO_LED_WAN1		4
#define CF_WR610N_GPIO_LED_LAN1		16
#define CF_WR610N_GPIO_LED_LAN2		15
#define CF_WR610N_GPIO_LED_LAN3		14
#define CF_WR610N_GPIO_LED_LAN4		11


static struct gpio_led cf_wr610n_leds_gpio[] __initdata = {
	{
		.name		= "comfast:red:wan",
		.gpio		= CF_WR610N_GPIO_LED_WAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:green:lan",
		.gpio		= CF_WR610N_GPIO_LED_LAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:blue:wlan",
		.gpio		= CF_WR610N_GPIO_LED_WLAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:green:wan1",
		.gpio		= CF_WR610N_GPIO_LED_WAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan1",
		.gpio		= CF_WR610N_GPIO_LED_LAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan2",
		.gpio		= CF_WR610N_GPIO_LED_LAN2,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan3",
		.gpio		= CF_WR610N_GPIO_LED_LAN3,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan4",
		.gpio		= CF_WR610N_GPIO_LED_LAN4,
		.active_low	= 1,
	}, 
};

static struct gpio_keys_button cf_wr610n_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = CF_WR610N_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= CF_WR610N_GPIO_BTN_RESET,
		.active_low	= 1,
	}
};

static void __init comfast_ap143_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);
	
	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/	
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, 0);	
	
	ath79_gpio_output_select(CF_WR610N_GPIO_XWDT_TRIGGER, 0);	
	enable_external_wdt(CF_WR610N_GPIO_XWDT_TRIGGER);
	
	ath79_gpio_output_select(CF_WR610N_GPIO_LED_WAN, 0);
	ath79_gpio_output_select(CF_WR610N_GPIO_LED_LAN, 0);
	ath79_gpio_output_select(CF_WR610N_GPIO_LED_WLAN, 0);	

	ath79_gpio_output_select(CF_WR610N_GPIO_LED_WAN1, 0);
	ath79_gpio_output_select(CF_WR610N_GPIO_LED_LAN1, 0);
	ath79_gpio_output_select(CF_WR610N_GPIO_LED_LAN2, 0);
	ath79_gpio_output_select(CF_WR610N_GPIO_LED_LAN3, 0);
	ath79_gpio_output_select(CF_WR610N_GPIO_LED_LAN4, 0);

	ath79_register_m25p80(NULL);

	ath79_setup_ar933x_phy4_switch(false, false);

	ath79_register_mdio(0, 0x0);
	
	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 2);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 0);

	// wan
	ath79_register_eth(1);

	// lan
	ath79_switch_data.phy4_mii_en = 1;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_register_eth(0);	

	ath79_register_wmac(art, NULL);
	
	ap91_pci_init(art + 0x5000, NULL);
}

static void __init cf_wr610n_setup(void)
{
	comfast_ap143_setup();

	ath79_register_leds_gpio(-1, ARRAY_SIZE(cf_wr610n_leds_gpio),
				 cf_wr610n_leds_gpio);

	ath79_register_gpio_keys_polled(1, CF_WR610N_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(cf_wr610n_gpio_keys),
					cf_wr610n_gpio_keys);
}

MIPS_MACHINE(ATH79_MACH_COMFAST_CF_WR610N, "COMFAST-CF-WR610N", "COMFAST CF-WR610N",
	     cf_wr610n_setup);
     
#define CF_WR615N_GPIO_BTN_RESET	17

#define CF_WR615N_KEYS_POLL_INTERVAL	20	/* msecs */
#define CF_WR615N_KEYS_DEBOUNCE_INTERVAL (3 * CF_WR615N_KEYS_POLL_INTERVAL)

#define	CF_WR615N_GPIO_XWDT_TRIGGER	13

#define CF_WR615N_GPIO_LED_WAN		2
#define CF_WR615N_GPIO_LED_LAN		3
#define CF_WR615N_GPIO_LED_WLAN		0

#define CF_WR615N_GPIO_LED_USR		1
#define CF_WR615N_GPIO_LED_WIFI		12

#define CF_WR615N_GPIO_LED_WAN1		4
#define CF_WR615N_GPIO_LED_LAN1		16
#define CF_WR615N_GPIO_LED_LAN2		15
#define CF_WR615N_GPIO_LED_LAN3		14
#define CF_WR615N_GPIO_LED_LAN4		11

static struct gpio_led cf_wr615n_leds_gpio[] __initdata = {
	{
		.name		= "comfast:red:wan",
		.gpio		= CF_WR615N_GPIO_LED_WAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:green:lan",
		.gpio		= CF_WR615N_GPIO_LED_LAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:blue:wlan",
		.gpio		= CF_WR615N_GPIO_LED_WLAN,
		.active_low	= 0,
	},
	{
		.name		= "comfast:green:wan1",
		.gpio		= CF_WR615N_GPIO_LED_WAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan1",
		.gpio		= CF_WR615N_GPIO_LED_LAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan2",
		.gpio		= CF_WR615N_GPIO_LED_LAN2,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan3",
		.gpio		= CF_WR615N_GPIO_LED_LAN3,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan4",
		.gpio		= CF_WR615N_GPIO_LED_LAN4,
		.active_low	= 1,
	},{
		.name		= "comfast:blue:wifi",
		.gpio		= CF_WR615N_GPIO_LED_WIFI,
		.active_low	= 1,
	}, {
		.name		= "comfast:blue:usr",
		.gpio		= CF_WR615N_GPIO_LED_USR,
		.active_low	= 1,
	},
};

static struct gpio_keys_button cf_wr615n_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = CF_WR615N_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= CF_WR615N_GPIO_BTN_RESET,
		.active_low	= 1,
	}
};

static void __init comfast_wr615n_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);
	
	ath79_gpio_output_select(CF_WR615N_GPIO_XWDT_TRIGGER, 0);
	enable_external_wdt(CF_WR615N_GPIO_XWDT_TRIGGER);
	
	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/	
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, 0);	
	
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_WAN, 0);
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_LAN, 0);
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_WLAN, 0);	
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_WIFI, 0);
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_USR, 0);

	ath79_gpio_output_select(CF_WR615N_GPIO_LED_WAN1, 0);
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_LAN1, 0);
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_LAN2, 0);
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_LAN3, 0);
	ath79_gpio_output_select(CF_WR615N_GPIO_LED_LAN4, 0);

	ath79_register_m25p80(NULL);

	ath79_setup_ar933x_phy4_switch(false, false);

	ath79_register_mdio(0, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 2);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 0);

	// wan
	ath79_register_eth(1);

	// lan
	ath79_switch_data.phy4_mii_en = 1;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_register_eth(0);
	
	ath79_register_wmac(art, NULL);	
}

static void __init cf_wr615n_setup(void)
{
	comfast_wr615n_setup();

	ath79_register_leds_gpio(-1, ARRAY_SIZE(cf_wr615n_leds_gpio),
				 cf_wr615n_leds_gpio);

	ath79_register_gpio_keys_polled(1, CF_WR615N_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(cf_wr615n_gpio_keys),
					cf_wr615n_gpio_keys);
}
MIPS_MACHINE(ATH79_MACH_COMFAST_CF_WR615N, "COMFAST-CF-WR615N", "COMFAST CF-WR615N",
	     cf_wr615n_setup);
     
#define CF_E520N_GPIO_BTN_RESET	17

#define CF_E520N_KEYS_POLL_INTERVAL	20	/* msecs */
#define CF_E520N_KEYS_DEBOUNCE_INTERVAL (3 * CF_E520N_KEYS_POLL_INTERVAL)

#define CF_E520N_GPIO_LED_WAN		11

static struct gpio_led cf_e520n_leds_gpio[] __initdata = {
	{
		.name		= "comfast:blue:wan",
		.gpio		= CF_E520N_GPIO_LED_WAN,
		.active_low	= 0,
	}
};

static struct gpio_keys_button cf_e520n_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = CF_E520N_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= CF_E520N_GPIO_BTN_RESET,
		.active_low	= 1,
	}
};

static void __init comfast_e520n_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);

	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/	
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, 0);	
	
	ath79_gpio_output_select(CF_E520N_GPIO_LED_WAN, 0);

	ath79_register_m25p80(NULL);

	ath79_setup_ar933x_phy4_switch(false, false);

	ath79_register_mdio(0, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 2);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 0);

	// wan
	ath79_register_eth(1);

	// lan
	ath79_switch_data.phy4_mii_en = 1;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_register_eth(0);
	
	ath79_register_wmac(art, NULL);	
}

static void __init cf_e520n_e530n_setup(void)
{
	comfast_e520n_setup();

	ath79_register_leds_gpio(-1, ARRAY_SIZE(cf_e520n_leds_gpio),
				 cf_e520n_leds_gpio);

	ath79_register_gpio_keys_polled(1, CF_E520N_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(cf_e520n_gpio_keys),
					cf_e520n_gpio_keys);
}
MIPS_MACHINE(ATH79_MACH_COMFAST_CF_E520N, "COMFAST-CF-E520N", "COMFAST CF-E520N",
	     cf_e520n_e530n_setup);
MIPS_MACHINE(ATH79_MACH_COMFAST_CF_E530N, "COMFAST-CF-E530N", "COMFAST CF-E530N",
	     cf_e520n_e530n_setup);	     
	     
     
#define CF_WR650AC_KEYS_POLL_INTERVAL	20	/* msecs */
#define CF_WR650AC_KEYS_DEBOUNCE_INTERVAL (3 * CF_WR650AC_KEYS_POLL_INTERVAL)

#define	CF_WR650AC_GPIO_XWDT_TRIGGER	17

#define CF_WR650AC_GPIO_BTN_RESET_WPS	19

#define CF_WR650AC_GPIO_LED_NETWORK	4
#define CF_WR650AC_GPIO_LED_24G		13
#define CF_WR650AC_GPIO_LED_58G		2
#define CF_WR650AC_GPIO_LED_WPS		20

#define CF_WR650AC_WMAC_CALDATA_OFFSET		0x1000
#define CF_WR650AC_PCIE_CALDATA_OFFSET		0x5000

#define CF_WR650AC_LAN_PHYMASK              BIT(0)
#define CF_WR650AC_WAN_PHYMASK              BIT(5)
#define CF_WR650AC_MDIO_MASK                (~(CF_WR650AC_LAN_PHYMASK | CF_WR650AC_WAN_PHYMASK))

static struct gpio_led cf_wr650ac_leds_gpio[] __initdata = {
	{
		.name		= "comfast:blue:network",
		.gpio		= CF_WR650AC_GPIO_LED_NETWORK,
		.active_low	= 1,
	},
	{
		.name		= "comfast:blue:24g",
		.gpio		= CF_WR650AC_GPIO_LED_24G,
		.active_low	= 1,
	},
	{
		.name		= "comfast:blue:58g",
		.gpio		= CF_WR650AC_GPIO_LED_58G,
		.active_low	= 1,
	},
	{
		.name		= "comfast:blue:wps",
		.gpio		= CF_WR650AC_GPIO_LED_WPS,
		.active_low	= 1,
	},
};

static struct gpio_keys_button cf_wr650ac_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button/WPS button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = CF_WR650AC_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= CF_WR650AC_GPIO_BTN_RESET_WPS,
		.active_low	= 1,
	},
};

static struct ar8327_pad_cfg cf_wr650ac_ar8327_pad0_cfg = {
	/* GMAC0 of the AR8337 switch is connected to GMAC0 via RGMII */
	.mode = AR8327_PAD_MAC_RGMII,
	.txclk_delay_en = true,
	.rxclk_delay_en = true,
	.txclk_delay_sel = AR8327_CLK_DELAY_SEL1,
	.rxclk_delay_sel = AR8327_CLK_DELAY_SEL2,
};

static struct ar8327_pad_cfg cf_wr650ac_ar8327_pad6_cfg = {
	/* GMAC6 of the AR8337 switch is connected to GMAC1 via SGMII */
	.mode = AR8327_PAD_MAC_SGMII,
	.rxclk_delay_en = true,
	.rxclk_delay_sel = AR8327_CLK_DELAY_SEL0,
};

static struct ar8327_platform_data cf_wr650ac_ar8327_data = {
	.pad0_cfg = &cf_wr650ac_ar8327_pad0_cfg,
	.pad6_cfg = &cf_wr650ac_ar8327_pad6_cfg,
	.port0_cfg = {
		.force_link = 1,
		.speed = AR8327_PORT_SPEED_1000,
		.duplex = 1,
		.txpause = 1,
		.rxpause = 1,
	},
	.port6_cfg = {
		.force_link = 1,
		.speed = AR8327_PORT_SPEED_1000,
		.duplex = 1,
		.txpause = 1,
		.rxpause = 1,
	},
};

static struct mdio_board_info cf_wr650ac_mdio0_info[] = {
	{
		.bus_id = "ag71xx-mdio.0",
		.phy_addr = 0,
		.platform_data = &cf_wr650ac_ar8327_data,
	},
};

static void __init cf_wr650ac_setup(void)
{
	u8 *art = (u8 *) KSEG1ADDR(0x1f020000);
	u8 wlan0_mac[ETH_ALEN];
	u8 wlan1_mac[ETH_ALEN];

	ath79_init_local_mac(ath79_eth0_data.mac_addr, art);
	ath79_init_mac(wlan0_mac, art, 1);
	ath79_init_mac(wlan1_mac, art, 3);
	
	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/	
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, 0);		

	ath79_gpio_output_select(CF_WR650AC_GPIO_XWDT_TRIGGER, 0);	
	enable_external_wdt(CF_WR650AC_GPIO_XWDT_TRIGGER);
	
	ath79_gpio_output_select(CF_WR650AC_GPIO_LED_NETWORK, 0);
	ath79_gpio_output_select(CF_WR650AC_GPIO_LED_24G, 0);
	ath79_gpio_output_select(CF_WR650AC_GPIO_LED_58G, 0);	
	ath79_gpio_output_select(CF_WR650AC_GPIO_LED_WPS, 0);

	ath79_register_m25p80(NULL);
	ath79_register_leds_gpio(-1, ARRAY_SIZE(cf_wr650ac_leds_gpio),
					cf_wr650ac_leds_gpio);
	ath79_register_gpio_keys_polled(-1, CF_WR650AC_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(cf_wr650ac_gpio_keys),
					cf_wr650ac_gpio_keys);

	ath79_register_usb();

	ath79_register_wmac(art + CF_WR650AC_WMAC_CALDATA_OFFSET, wlan0_mac);

	ath79_setup_qca955x_eth_cfg(QCA955X_ETH_CFG_RGMII_EN);

	ath79_register_mdio(0, 0x0);

	mdiobus_register_board_info(cf_wr650ac_mdio0_info,
				    ARRAY_SIZE(cf_wr650ac_mdio0_info));

	/* GMAC0 is connected to the RMGII interface */
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_RGMII;
	ath79_eth0_data.phy_mask = CF_WR650AC_LAN_PHYMASK;
	ath79_eth0_data.mii_bus_dev = &ath79_mdio0_device.dev;
	ath79_eth0_pll_data.pll_1000 = 0xa6000000;
	ath79_register_eth(0);

	/* GMAC1 is connected to the SGMII interface */
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_SGMII;
	ath79_eth1_data.speed = SPEED_1000;
	ath79_eth1_data.duplex = DUPLEX_FULL;
	ath79_eth1_pll_data.pll_1000 = 0x03000101;
	ath79_register_eth(1);

	ap91_pci_init(art + CF_WR650AC_PCIE_CALDATA_OFFSET, wlan1_mac);
}

MIPS_MACHINE(ATH79_MACH_COMFAST_CF_WR650AC, "COMFAST-CF-WR650AC", "COMFAST CF-WR650AC",
	     cf_wr650ac_setup);	     
     
#define KT9671_GPIO_BTN_RESET	17

#define KT9671_KEYS_POLL_INTERVAL	20	/* msecs */
#define KT9671_KEYS_DEBOUNCE_INTERVAL (3 * KT9671_KEYS_POLL_INTERVAL)

#define	KT9671_GPIO_XWDT_TRIGGER	13

#define KT9671_GPIO_LED_WAN		2
#define KT9671_GPIO_LED_LAN		3
#define KT9671_GPIO_LED_WLAN		0

#define KT9671_GPIO_LED_WAN1		4
#define KT9671_GPIO_LED_LAN1		16
#define KT9671_GPIO_LED_LAN2		15
#define KT9671_GPIO_LED_LAN3		14
#define KT9671_GPIO_LED_LAN4		11


static struct gpio_led cf_kt9671_leds_gpio[] __initdata = {
	{
		.name		= "comfast:red:wan",
		.gpio		= KT9671_GPIO_LED_WAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:green:lan",
		.gpio		= KT9671_GPIO_LED_LAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:blue:wlan",
		.gpio		= KT9671_GPIO_LED_WLAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:green:wan1",
		.gpio		= KT9671_GPIO_LED_WAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan1",
		.gpio		= KT9671_GPIO_LED_LAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan2",
		.gpio		= KT9671_GPIO_LED_LAN2,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan3",
		.gpio		= KT9671_GPIO_LED_LAN3,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan4",
		.gpio		= KT9671_GPIO_LED_LAN4,
		.active_low	= 1,
	}, 
};

static struct gpio_keys_button cf_kt9671_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = KT9671_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= KT9671_GPIO_BTN_RESET,
		.active_low	= 1,
	}
};

static void __init comfast_cf_kt9671_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);
	
	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/	
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, 0);	
	
	ath79_gpio_output_select(KT9671_GPIO_XWDT_TRIGGER, 0);	
	enable_external_wdt(KT9671_GPIO_XWDT_TRIGGER);
	
	ath79_gpio_output_select(KT9671_GPIO_LED_WAN, 0);
	ath79_gpio_output_select(KT9671_GPIO_LED_LAN, 0);
	ath79_gpio_output_select(KT9671_GPIO_LED_WLAN, 0);	

	ath79_gpio_output_select(KT9671_GPIO_LED_WAN1, 0);
	ath79_gpio_output_select(KT9671_GPIO_LED_LAN1, 0);
	ath79_gpio_output_select(KT9671_GPIO_LED_LAN2, 0);
	ath79_gpio_output_select(KT9671_GPIO_LED_LAN3, 0);
	ath79_gpio_output_select(KT9671_GPIO_LED_LAN4, 0);

	ath79_register_m25p80(NULL);

	ath79_setup_ar933x_phy4_switch(false, false);

	ath79_register_mdio(0, 0x0);
	
	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 2);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 0);
	// wan
	ath79_register_eth(1);

	// lan
	ath79_switch_data.phy4_mii_en = 1;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_register_eth(0);	

	ath79_register_wmac(art, NULL);
	
	ap91_pci_init(art + 0x5000, NULL);
}

static void __init cf_kt9671_setup(void)
{
	comfast_cf_kt9671_setup();

	ath79_register_leds_gpio(-1, ARRAY_SIZE(cf_kt9671_leds_gpio),
				 cf_kt9671_leds_gpio);

	ath79_register_gpio_keys_polled(1, KT9671_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(cf_kt9671_gpio_keys),
					cf_kt9671_gpio_keys);
}

MIPS_MACHINE(ATH79_MACH_KUNTENG_KT9671, "KUNTENG-KT9671", "KunTeng KT9671",
	     cf_kt9671_setup);
	     
#define KT9672_GPIO_BTN_RESET	17

#define KT9672_KEYS_POLL_INTERVAL	20	/* msecs */
#define KT9672_KEYS_DEBOUNCE_INTERVAL (3 * KT9672_KEYS_POLL_INTERVAL)

#define	KT9672_GPIO_XWDT_TRIGGER	13

#define KT9672_GPIO_LED_WAN		2
#define KT9672_GPIO_LED_LAN		3
#define KT9672_GPIO_LED_WLAN		0

#define KT9672_GPIO_LED_WAN1		4
#define KT9672_GPIO_LED_LAN1		16
#define KT9672_GPIO_LED_LAN2		15
#define KT9672_GPIO_LED_LAN3		14
#define KT9672_GPIO_LED_LAN4		11

static struct gpio_led kt9672_leds_gpio[] __initdata = {
	{
		.name		= "comfast:red:wan",
		.gpio		= KT9672_GPIO_LED_WAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:green:lan",
		.gpio		= KT9672_GPIO_LED_LAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:blue:wlan",
		.gpio		= KT9672_GPIO_LED_WLAN,
		.active_low	= 0,
	}, {
		.name		= "comfast:green:wan1",
		.gpio		= KT9672_GPIO_LED_WAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan1",
		.gpio		= KT9671_GPIO_LED_LAN1,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan2",
		.gpio		= KT9672_GPIO_LED_LAN2,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan3",
		.gpio		= KT9672_GPIO_LED_LAN3,
		.active_low	= 1,
	}, {
		.name		= "comfast:green:lan4",
		.gpio		= KT9672_GPIO_LED_LAN4,
		.active_low	= 1,
	}, 
};

static struct gpio_keys_button kt9672_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = KT9672_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= KT9672_GPIO_BTN_RESET,
		.active_low	= 1,
	}
};

static void __init comfast_kt9672_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);
	u8 *art = (u8 *) KSEG1ADDR(0x1f011000);
	
	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/	
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, 0);	
	
	ath79_gpio_output_select(KT9672_GPIO_XWDT_TRIGGER, 0);	
	enable_external_wdt(KT9672_GPIO_XWDT_TRIGGER);
	
	ath79_gpio_output_select(KT9672_GPIO_LED_WAN, 0);
	ath79_gpio_output_select(KT9672_GPIO_LED_LAN, 0);
	ath79_gpio_output_select(KT9672_GPIO_LED_WLAN, 0);	

	ath79_gpio_output_select(KT9672_GPIO_LED_WAN1, 0);
	ath79_gpio_output_select(KT9672_GPIO_LED_LAN1, 0);
	ath79_gpio_output_select(KT9672_GPIO_LED_LAN2, 0);
	ath79_gpio_output_select(KT9672_GPIO_LED_LAN3, 0);
	ath79_gpio_output_select(KT9672_GPIO_LED_LAN4, 0);

	ath79_register_m25p80(NULL);

	ath79_setup_ar933x_phy4_switch(false, false);

	ath79_register_mdio(0, 0x0);
	
	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 2);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 0);

	// wan
	ath79_register_eth(1);

	// lan
	ath79_switch_data.phy4_mii_en = 1;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_register_eth(0);	

	ath79_register_wmac(art, NULL);
	
	ap91_pci_init(art + 0x5000, NULL);
}

static void __init kt9672_setup(void)
{
	comfast_kt9672_setup();

	ath79_register_leds_gpio(-1, ARRAY_SIZE(kt9672_leds_gpio),
				 kt9672_leds_gpio);

	ath79_register_gpio_keys_polled(1, KT9672_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(kt9672_gpio_keys),
					kt9672_gpio_keys);
}

MIPS_MACHINE(ATH79_MACH_KUNTENG_KT9672, "KUNTENG-KT9672", "KunTeng KT9672",
	     kt9672_setup);
	     
#define CF_E380AC_KEYS_POLL_INTERVAL	20	/* msecs */
#define CF_E380AC_KEYS_DEBOUNCE_INTERVAL (3 * CF_E380AC_KEYS_POLL_INTERVAL)

#define	CF_E380AC_GPIO_XWDT_TRIGGER	17

#define CF_E380AC_GPIO_BTN_RESET_WPS	19

#define CF_E380AC_GPIO_LED_WAN		3
#define CF_E380AC_GPIO_LED_LAN		0
#define CF_E380AC_GPIO_LED_WLAN		2

#define CF_E380AC_WMAC_CALDATA_OFFSET		0x1000
#define CF_E380AC_PCIE_CALDATA_OFFSET		0x5000

#define CF_E380AC_LAN_PHYMASK              BIT(0)
#define CF_E380AC_WAN_PHYMASK              BIT(5)
#define CF_E380AC_MDIO_MASK                (~(CF_E380AC_LAN_PHYMASK | CF_E380AC_WAN_PHYMASK))

static struct gpio_led cf_e380ac_leds_gpio[] __initdata = {
	{
		.name		= "comfast:red:wan",
		.gpio		= CF_E380AC_GPIO_LED_WAN,
		.active_low	= 0,
	}, 
	{
		.name		= "comfast:green:lan",
		.gpio		= CF_E380AC_GPIO_LED_LAN,
		.active_low	= 0,
	}, 
	{
		.name		= "comfast:blue:wlan",
		.gpio		= CF_E380AC_GPIO_LED_WLAN,
		.active_low	= 0,
	},

};

static struct gpio_keys_button cf_e380ac_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button/WPS button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = CF_E380AC_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= CF_E380AC_GPIO_BTN_RESET_WPS,
		.active_low	= 1,
	},
};

static struct at803x_platform_data cf_e380ac_at803x_data = {
	.disable_smarteee = 1,
//	.enable_rgmii_rx_delay = 1,
//	.enable_rgmii_tx_delay = 1,
};

static struct mdio_board_info cf_e380ac_mdio0_info[] = {
	{
		.bus_id = "ag71xx-mdio.0",
		.phy_addr = 0,
		.platform_data = &cf_e380ac_at803x_data,
	},
};

static void __init cf_e380ac_setup(void)
{
	u8 *art = (u8 *) KSEG1ADDR(0x1f020000);
	u8 wlan0_mac[ETH_ALEN];
	u8 wlan1_mac[ETH_ALEN];

	ath79_init_local_mac(ath79_eth0_data.mac_addr, art);
	ath79_init_mac(wlan0_mac, art, 1);
	ath79_init_mac(wlan1_mac, art, 3);
	
	/* Disable JTAG, enabling GPIOs 0-3 */
	/* Configure OBS4 line, for GPIO 4*/	
	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, 0);		

	ath79_gpio_output_select(CF_E380AC_GPIO_XWDT_TRIGGER, 0);	
	enable_external_wdt(CF_E380AC_GPIO_XWDT_TRIGGER);
	
	ath79_gpio_output_select(CF_E380AC_GPIO_LED_WAN, 0);
	ath79_gpio_output_select(CF_E380AC_GPIO_LED_LAN, 0);
	ath79_gpio_output_select(CF_E380AC_GPIO_LED_WLAN, 0);	

	ath79_register_m25p80(NULL);
	ath79_register_leds_gpio(-1, ARRAY_SIZE(cf_e380ac_leds_gpio),
					cf_e380ac_leds_gpio);
	ath79_register_gpio_keys_polled(-1, CF_E380AC_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(cf_e380ac_gpio_keys),
					cf_e380ac_gpio_keys);

	ath79_register_usb();
	ath79_register_wmac(art + CF_E380AC_WMAC_CALDATA_OFFSET, wlan0_mac);
	ath79_setup_qca955x_eth_cfg(QCA955X_ETH_CFG_RGMII_EN);
	
	ath79_register_mdio(0, 0x0);
	mdiobus_register_board_info(cf_e380ac_mdio0_info,
				    ARRAY_SIZE(cf_e380ac_mdio0_info));

	ath79_init_local_mac(ath79_eth0_data.mac_addr, art);
	ath79_eth0_data.mii_bus_dev = &ath79_mdio0_device.dev;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_RGMII;
	ath79_eth0_data.phy_mask = BIT(0);
	ath79_eth0_pll_data.pll_10 = 0xB0001313;
	ath79_eth0_pll_data.pll_100 = 0xB0000101;
	ath79_eth0_pll_data.pll_1000 = 0xBE000000;	
	ath79_register_eth(0);

	ap91_pci_init(art + CF_E380AC_PCIE_CALDATA_OFFSET, wlan1_mac);
}	


MIPS_MACHINE(ATH79_MACH_COMFAST_CF_E380AC, "COMFAST-CF-E380AC", "COMFAST CF-E380AC",
	     cf_e380ac_setup);	     
	     
#if 0
#define DBDC9344GE_PHYMASK		BIT(0)
#define	DBDC9344GE_WMAC_CALDATA_OFFSET	0x1000
#define DBDC9344GE_PCI_CALDATA_OFFSET	0x5000

#define	DBDC9344GE_GPIO_EXTPHY_RESET	11

static struct gpio_keys_button comfast_dbdc9344ge_gpio_keys[] __initdata = {
	{
		.desc			= "reset",
		.type			= EV_KEY,
		.code			= KEY_RESTART,
		.debounce_interval	= COMFAST_KEYS_DEBOUNCE_INTERVAL,
		.gpio			= 22,
		.active_low		= 1,
	}
};

static struct gpio_led comfast_dbdc9344ge_gpio_leds[] __initdata = {
	{
		.name		= "comfast:red",
		.gpio		= 2,
	}, {
		.name		= "comfast:green",
		.gpio		= 3,
	}, {
		.name		= "comfast:blue",
		.gpio		= 0,
	}
};

static void __init comfast_dbdc9344ge_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f010000);

	ath79_gpio_function_setup(AR934X_GPIO_FUNC_JTAG_DISABLE, AR934X_GPIO_FUNC_CLK_OBS4_EN);

	ath79_register_m25p80(NULL);

	ath79_register_leds_gpio(-1, ARRAY_SIZE(comfast_dbdc9344ge_gpio_leds),
				 comfast_dbdc9344ge_gpio_leds);
				 
	ath79_register_gpio_keys_polled(-1, COMFAST_KEYS_POLL_INTERVAL,
                                        ARRAY_SIZE(comfast_dbdc9344ge_gpio_keys),
                                        comfast_dbdc9344ge_gpio_keys);
					
	ext_lna_control_gpio_setup(18, 19);

	ath79_register_mdio(0, 0x00);

	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);

	/* GMAC0 is connected to GE-PHY by RGMII */
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_RGMII;
	ath79_eth0_data.phy_mask = DBDC9344GE_PHYMASK;
	ath79_register_eth(0);

	ath79_register_wmac(mac + DBDC9344GE_WMAC_CALDATA_OFFSET, NULL);
	ap91_pci_init(mac + DBDC9344GE_PCI_CALDATA_OFFSET, NULL);
}

MIPS_MACHINE(ATH79_MACH_COMFAST_DBDC9344GE, "COMFAST-DBDC9344GE", "COMFAST DBDC9344GE", comfast_dbdc9344ge_setup);
#endif
