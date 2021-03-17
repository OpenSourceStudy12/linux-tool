#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <linux/mii.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/sockios.h>
#include <linux/types.h>
#include <netinet/in.h>
#include <stdbool.h>

#define reteck(ret) \
if(ret < 0){ \
printf("%m! \"%s\" : line: %d\n", __func__, __LINE__); \
goto lab; \
}

#define help() \
printf("mii_phy usage:\n\
support options:\n\
	-I	specify interface name\n\
	-w	write/read, default is read\n\
	-r	specify the start addr of phy\n\
	-n	set the number of phy reg to I/O\n\
	-v	set the value to write when -w is set\n");	\
exit(0);

void format_value(char *argv, uint16_t *value, int num)
{
	char *ch;
	int i = 0;

	ch = strtok(argv, " ");
	while (ch != NULL) {
		value[i++] = (uint16_t)strtoul(ch, NULL, 0);
		if (i >= num)
			return;
		ch = strtok(NULL, " ");
	}
}

bool phy_io_operator(char *interface, bool write, uint16_t *phy_id, uint16_t addr, uint8_t num, uint16_t *value)
{
	struct ifreq ifr;
	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr.ifr_data;
	int sockfd, ret = true;

	strncpy(ifr.ifr_name, interface, IFNAMSIZ -1);
	sockfd = socket(PF_LOCAL, SOCK_DGRAM, 0);
	reteck(sockfd);

	/* get phy address in smi bus */
	ret = ioctl(sockfd, SIOCGMIIPHY, &ifr);
	reteck(ret);

	*phy_id = mii->phy_id;

	for(int i = 0; i < num; i++) {
		mii->reg_num = addr + i;
		if (write) {
			mii->val_in = value[i];
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			reteck(ret);
		} else {
			ret = ioctl(sockfd, SIOCGMIIREG, &ifr);
			reteck(ret);
			value[i] = mii->val_out;
		}
	}
lab:
	close(sockfd);
	return ret < 0 ? false : true;
}

int main(int argc, char *argv[])
{
	char *interface = NULL;
	bool write = false;
	uint16_t reg_start = 0;
	uint number = 1;
	uint16_t reg_value[128];
	uint16_t phy_id;
	char out_str[1024] = "\0";

	int ch;

	if(argc == 1 || !strcmp(argv[1], "-h")){
		help();
	}

	while ((ch = getopt(argc, argv, "I:wr:n:v:")) != -1) {
		switch (ch) {
			case 'I':
				interface = optarg;
				break;
			case 'w':
				write = true;
				break;
			case 'r':
				reg_start = (uint16_t)strtoul(optarg, NULL, 0);
				break;
			case 'n':
				number = (uint)strtoul(optarg, NULL, 0);
				break;
			case 'v':
				strncpy(out_str, optarg, strlen(optarg));
				break;
			default:
				help();
		}
	}

	if (write)
		format_value(out_str, reg_value, number);

	bzero(out_str, sizeof out_str);
	if (phy_io_operator(interface, write, &phy_id, reg_start, number, reg_value)) {
		if (!write) {
			sprintf(out_str, 
"interface : %s, PHY id : %#x\n\
reg	value:\n\
------------\n", interface, phy_id);
			for (int i = 0; i < number; i++)
				sprintf(out_str + strlen(out_str), "0x%02x ---- 0x%04x\n", reg_start + i, reg_value[i]);
			printf(out_str);
		}
	} else
		printf("operator error\n");

	return 0;
}
