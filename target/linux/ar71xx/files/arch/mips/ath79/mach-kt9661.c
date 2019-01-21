/*
 *
 *
 */

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/ar71xx_regs.h>

#include "common.h"
#include "dev-eth.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-wmac.h"
#include "machtypes.h"

#define KT9661_GPIO_LED_SYSTEM 11
#define KT9661_GPIO_LED_WLAN   12
#define KT9661_GPIO_LED_WAN    13
#define KT9661_GPIO_LED_LAN    4

#define KT9661_GPIO_BTN_RESET  3

#define KT9661_KEYS_POLL_INTERVAL  		20  /* msecs */
#define KT9661_KEYS_DEBOUNCE_INTERVAL 	(3 * KT9661_KEYS_POLL_INTERVAL)

/* jack tseng 01/07/2016 add */
/* use the mac from the command: "./mfg-info --set --mac-addr-main 00:01:6C:10:00:A3 2"  */
#define MFC_BASE_ADDR 			0x9F060000
#define MAC_OFFSET    			0xD0
#define RADIO_0_OFFSET    		0x110
#define MAC_ADDR      			MFC_BASE_ADDR + MAC_OFFSET
#define RADIO_0_MAC_ADDR      		MFC_BASE_ADDR + RADIO_0_OFFSET
/* end of jack tseng 01/07/2016 */

static struct gpio_led kt9661_leds_gpio[] __initdata = {
	{
       .name       = "kt9661:green:wan",
       .gpio       = KT9661_GPIO_LED_WAN,
       .active_low = 1,
   	},{
       .name       = "kt9661:green:system",
       .gpio       = KT9661_GPIO_LED_SYSTEM,
       .active_low = 1,
   	}, {
       .name       = "kt9661:green:wlan",
       .gpio       = KT9661_GPIO_LED_WLAN,
       .active_low = 1,
   	}, {
       .name       = "kt9661:green:lan",
       .gpio       = KT9661_GPIO_LED_LAN,
       .active_low = 1,
   	}
};

static struct gpio_keys_button kt9661_gpio_keys[] __initdata = {
   {
       .desc       = "Reset button",
       .type       = EV_KEY,
       .code       = KEY_RESTART,
       .debounce_interval = KT9661_KEYS_DEBOUNCE_INTERVAL,
       .gpio       = KT9661_GPIO_BTN_RESET,
       .active_low = 1,
   }
};


void ath79_get_mac(char* smac, u8* umac)
{
	int i = 0, char_cnt = 0;
	char *pos_ptr = smac;
	u8 mac_char[2];

	do {
		if ((*pos_ptr == 0x3A) || (*pos_ptr == 0x0)) {
			if (char_cnt==2) {
				if (mac_char[0] > 0x60) {
					mac_char[0] = (mac_char[0] - 0x57)<<4;
				} else if (mac_char[0] > 0x40) {
					mac_char[0] = (mac_char[0] - 0x37)<<4;
				} else {
					mac_char[0] = (mac_char[0] - 0x30)<<4;
				}

				if (mac_char[1] > 0x60) {
					mac_char[1] = mac_char[1] - 0x57;
				} else if (mac_char[1] > 0x40) {
					mac_char[1] = mac_char[1] - 0x37;
				} else {
					mac_char[1] = mac_char[1] - 0x30;
				}       

				umac[i] = mac_char[0] + mac_char[1];
			} else if (char_cnt == 1) {
				if (mac_char[0] > 0x60) {
					mac_char[0] = mac_char[0] - 0x57;
				} else if (mac_char[0] > 0x40) {
					mac_char[0] = mac_char[0] - 0x37;
				} else {
					mac_char[0] = mac_char[0] - 0x30;
				}

				umac[i] = mac_char[0];
			} else {
				umac[i] = 0xFF;
			}

			i++;
			char_cnt = 0;
		} else {
			mac_char[char_cnt] = *pos_ptr;
			char_cnt += 1;
		}
	} while (*pos_ptr != 0x0 && pos_ptr++);
}  


static void __init kf_ap143_setup(void)
{
	u8 *ee = (u8 *) KSEG1ADDR(0x1f071000);
	u8 tmpmac[ETH_ALEN] = {-1};
	char mac_string[32] = {0};
	u8 ethmac[6] = {-1};
	u8 radio_0_mac[6] = {-1};
	
	/* gukq 20160510 modified for supporting other qca routers */
//	ath79_register_m25p80(NULL);
	ath79_register_m25p80_32M(NULL);
	/* gukq added over */

	ath79_setup_ar933x_phy4_switch(false, false);

	ath79_register_mdio(0, 0x0);

	/* jack tseng 01/07/2016 add */
	/* use the mac from the command "./mfg-info --set --mac-addr-main 00:01:6C:10:00:A3 2"  */
	memcpy(mac_string, (const void *)MAC_ADDR, 32);
	ath79_get_mac(mac_string, ethmac);
	pr_err("eth mac address: %02x:%02x:%02x:%02x:%02x:%02x \n",
		ethmac[0], ethmac[1], ethmac[2], ethmac[3], ethmac[4], ethmac[5]);

	memset(mac_string, 0, 32);
	memcpy(mac_string, (const void *)RADIO_0_MAC_ADDR, 32);
	ath79_get_mac(mac_string, radio_0_mac);

	/* LAN */
	ath79_init_mac(ath79_eth1_data.mac_addr, ethmac, 0);

	ath79_register_eth(1);

	/* WAN */
	ath79_switch_data.phy4_mii_en = 1;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;

	ath79_init_mac(ath79_eth0_data.mac_addr, ethmac, 1);

	ath79_register_eth(0);

	/* set wlan mac */
	pr_err("radio_0 mac address: %02x:%02x:%02x:%02x:%02x:%02x \n",
		radio_0_mac[0], radio_0_mac[1], radio_0_mac[2], radio_0_mac[3], radio_0_mac[4], radio_0_mac[5]);
	ath79_init_mac(tmpmac, radio_0_mac, 0);

	ath79_register_wmac(ee, tmpmac);
}

static void __init kt9661_setup(void)
{
	kf_ap143_setup();

	ath79_register_leds_gpio(-1, ARRAY_SIZE(kt9661_leds_gpio),
				 kt9661_leds_gpio);

	ath79_register_gpio_keys_polled(1, KT9661_KEYS_POLL_INTERVAL,
					ARRAY_SIZE(kt9661_gpio_keys),
					kt9661_gpio_keys);
}

MIPS_MACHINE(ATH79_MACH_KT9661, "KT9661", "KUNTENG KT9661",
	     kt9661_setup);
