#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "util.h"			//gukq added 20160509

#define MIB0 "/dev/mtd2"
#define ART "/dev/mtd3"
//#define ART "/dev/mtd3"
//#define MIB0 "/home/zcc/openwrt/application/mib.bin"
//#define ART "/home/zcc/openwrt/application/art.bin"

/* gukq added 20160509 */
#define MAC_LEN 17
#define MIB0_TMP_PATH "/tmp/mtd2.bin"

static char **sys_argv;
static int sys_argc;
/* gukq added over */

void mac_cov(unsigned char *ascii_mac, unsigned char *mac)
{
	int i, j;
	for( j = i = 0; j < 17; j++ )
	{
		unsigned char temp;
		if( isdigit(ascii_mac[j]) )
		{
			temp = (ascii_mac[j] - '0') * 16;
		}else if( islower(ascii_mac[j]) )
		{
			temp = (ascii_mac[j] - 'a' + 10) * 16;
		}else if( isupper(ascii_mac[j]) )
		{
			temp = (ascii_mac[j] - 'A' + 10) * 16;
		}

		if( isdigit(ascii_mac[j+1]) )
		{
			temp += (ascii_mac[++j] - '0');
		}else if( islower(ascii_mac[j+1]) )
		{
			temp += (ascii_mac[++j] - 'a' + 10);
		}else if( isupper(ascii_mac[j+1]) )
		{
			temp += (ascii_mac[++j] - 'A' + 10);
		}

		++j;
		mac[i++] = temp;
	}
}

void get_mac_from_mib0(unsigned char *mac)
{
	unsigned char ascii_mac[18] = {"\0"};
	int mib_fd, i;
	mib_fd = open(MIB0, O_RDONLY);
	if(mib_fd < 0)
	{
		printf("open mib_fd !\n");
		exit(0);
	}
	lseek(mib_fd, 0x0110, SEEK_SET);
	read(mib_fd, ascii_mac, 17);
	mac_cov(ascii_mac, mac);
	close(mib_fd);
}

void get_mac_from_art(unsigned char *mac)
{
	int art_fd, i;
	art_fd = open(ART, O_RDONLY);
	if(art_fd < 0)
	{
		printf("open art_fd !\n");
		exit(0);
	}
	lseek(art_fd, 0x1002, SEEK_SET);	//gukq rewrite 4098 to 0x1002
	read(art_fd, mac, 6);
	close(art_fd);
}

/*gukq added 20160509 */
void modiy_mac() {
	if(sys_argc > 2) {
		if(strlen(sys_argv[1]) < 17) {
			printf("MAC-address %s format error!\n", sys_argv[1]);
			exit(0);
		}
		if(strlen(sys_argv[2]) < 17) {
			printf("MAC-address %s format error!\n", sys_argv[2]);
			exit(0);
		}

		char new_mac[2][MAC_LEN] = {0};
		strncpy(new_mac[0], sys_argv[1], MAC_LEN);
		strncpy(new_mac[1], sys_argv[2], MAC_LEN);

		char cmd[64] = {0};
		sprintf(cmd, "/bin/cat /dev/mtd2 > %s", MIB0_TMP_PATH);
		system(cmd);

		if (! IsPathExist(MIB0_TMP_PATH)) {
			printf("tmp mtb0 binary file created error!\n");
			exit(0);
		}

		char *mib0_tmp = MIB0_TMP_PATH;
		int mib_fd;
		mib_fd = open(mib0_tmp, O_RDWR);
		if(mib_fd < 0)
		{
			printf("open mib_fd !\n");
			exit(0);
		}
		lseek(mib_fd, 0x000000d0, SEEK_SET);
		write(mib_fd, new_mac[0], 17);
		lseek(mib_fd, 272, SEEK_SET);
		write(mib_fd, new_mac[1], 17);
		close(mib_fd);

		memset(cmd, 0, 64);
		sprintf(cmd, "/sbin/mtd write %s mib0", MIB0_TMP_PATH);
		system(cmd);
	}
}
/* gukq added over */

void compare_and_fix_mac()
{
	int i,equ = 1;
	unsigned char art_mac[7] = {"\0"}, mib_mac[7] = {"\0"};
	get_mac_from_mib0(mib_mac);
	get_mac_from_art(art_mac);
	for(i = 0; i < 6 ; i++ )
	{
		printf("mac[%d]:%x\t %x\n", i, mib_mac[i], art_mac[i]);
		if(mib_mac[i] != art_mac[i])
		{
			printf("the mac not same!fix...\n");
			equ = 0;
		}
	}
	if( ! equ )
	{
		system("/bin/dd if=/dev/mtd3 of=/bin/art.bin");
		int art_fd;
		art_fd = open("/bin/art.bin", O_RDWR);
		if(art_fd < 0 )
		{
			printf("open art_fd !\n");
		}

		lseek(art_fd, 4098, SEEK_SET);
		write(art_fd, mib_mac, 6);
		close(art_fd);
		
		system("/sbin/mtd write /bin/art.bin /dev/mtd3;/bin/rm /bin/art.bin");
	}
}

int main(int argc, char** argv)
{
	/*gukq added 20160509 */
	sys_argc = argc;
	sys_argv = argv;
	modiy_mac();
	/*gukq added over */
	compare_and_fix_mac();
}

