#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <errno.h>
#include <sys/ioctl.h>
//#include <sys/types.h>
#include <sys/signal.h>
#include <semaphore.h>
#include <termios.h>

#define ERROR_PARAM 1
#define ERROR_OPENDEV 2
#define ERROR_PIN_CHECK 3
#define ERROR_GET_ATTR 4
#define ERROR_SET_ATTR 5
#define ERROR_TX_RX 6

#define RANDROM(x) (rand() % x)

speed_t speed[] = {B110, B300, B600, B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800,
B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000,
B2500000, B3000000, B3500000, B4000000 };

char* speed_name[] = {"110", "300", "600", "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200",
		"230400", "460800", "500000", "576000", "921600", "1000000", "1152000",
		"1500000", "2000000", "2500000", "3000000", "3500000", "4000000" };

enum com_dir {
	SERIAL_IN,
	SERIAL_OUT
};

typedef enum {
	M_S,
	S_M
} com_mode_t;

typedef enum {
	SERIAL_HALF,
	SERIAL_FULL
} serial_t;

typedef enum {
	SINGLE,
	DUAL
}test_mode_t;

struct thread_parameter {
	serial_t serial;
	test_mode_t test_mode;
	int com_mode;
};

int fd[2] = {-1, -1}, timeout;
pthread_t pth[2] = {-1, -1};
speed_t current_speed;
sem_t sem[2];

void signal_handle()
{
	exit(1);
}

int uart_send(int fd, void *buf, int len) {
	int ret = 0;
	int count = 0;

	tcflush(fd, TCIFLUSH);
	while (len > 0) {
		ret = write(fd, (char*) buf + count, len);
		if (ret < 1)
			break;
		count += ret;
		len = len - ret;
	}
	return count;
}

int uart_recv_timeout(int fd, void *buf, int len, int timeout_ms) {
	int ret = 0;
	size_t rsum = 0;
	fd_set rset;
	struct timeval timeout;

	while (rsum < len) {
		timeout.tv_sec = timeout_ms / 1000;
		timeout.tv_usec = timeout_ms % 1000 * 1000;
		FD_ZERO(&rset);
		FD_SET(fd, &rset);

		ret = select(fd + 1, &rset, NULL, NULL, &timeout);
		if (ret <= 0) {
			if (ret == 0)
				return -1;
		} else {
			ret = read(fd, (char*) buf + rsum, len - rsum);
			if (ret < 0)
				return ret;
			else
				rsum += ret;
		}
	}
	return rsum;
}

bool check_rts(int fd, bool set) {
	u_int32_t controlbits, value;

	ioctl(fd, TIOCMGET, &controlbits);
	value = controlbits & TIOCM_RTS;
	if (set ? value : !value) {
		printf("error at : RTS pin 7\r\n");
		return false;
	}
	value = controlbits & TIOCM_CTS;
	if (set ? value : !value) {
		printf("error at : CTS pin 8\r\n");
		return false;
	}
	return true;
}

bool check_dtr(int fd, bool set) {
	u_int32_t controlbits, value;

	ioctl(fd, TIOCMGET, &controlbits);
	value = controlbits & TIOCM_DTR;
	if (set ? value : !value) {
		printf("error at : DTR pin 4\r\n");
		return false;
	}
	value = controlbits & TIOCM_CD;
	if (set ? value : !value) {
		printf("error at : CD pin 1\r\n");
		return false;
	}
	value = controlbits & TIOCM_DSR;
	if (set ? value : !value) {
		printf("error at : DSR pin 6\r\n");
		return false;
	}
	value = controlbits & TIOCM_RI;
	if (set ? value : !value) {
		printf("error at : RI pin 9\r\n");
		return false;
	}
	return true;
}

bool test_pin(int fd) {
	u_int32_t controlbits, controlbits_back;
	bool res = true;

	ioctl(fd, TIOCMGET, &controlbits);
	controlbits_back = controlbits;

	// set RTS AND DTR
//	printf("set RTS DTR: ");
	controlbits |= TIOCM_RTS;
	controlbits |= TIOCM_DTR;
	ioctl(fd, TIOCMSET, &controlbits);
	if (!check_rts(fd, !(controlbits & TIOCM_RTS))
			|| !check_dtr(fd, !(controlbits & TIOCM_DTR))) {
		res = false;
		goto exit;
	}

	// set RTS and remove DTR
//	printf("\r\nset RTS Remove DTR: ");
	controlbits |= TIOCM_RTS;
	controlbits &= ~TIOCM_DTR;
	ioctl(fd, TIOCMSET, &controlbits);
	if (!check_rts(fd, !(controlbits & TIOCM_RTS))
			|| !check_dtr(fd, !(controlbits & TIOCM_DTR))) {
		res = false;
		goto exit;
	}

	// remove RTS and DTR
//	printf("\r\nremove RTS DTR: ");
	controlbits &= ~TIOCM_RTS;
	controlbits &= ~TIOCM_DTR;
	ioctl(fd, TIOCMSET, &controlbits);
	if (!check_rts(fd, !(controlbits & TIOCM_RTS))
			|| !check_dtr(fd, !(controlbits & TIOCM_DTR))) {
		res = false;
		goto exit;
	}

	// remove RTS and set DTR
//	printf("\r\nremove RTS and set DTR: ");
	controlbits &= ~TIOCM_RTS;
	controlbits |= TIOCM_DTR;
	ioctl(fd, TIOCMSET, &controlbits);
	if (!check_rts(fd, !(controlbits & TIOCM_RTS))
			|| !check_dtr(fd, !(controlbits & TIOCM_DTR))) {
		res = false;
		goto exit;
	}

	exit:
	// restore back setting
	ioctl(fd, TIOCMSET, &controlbits_back);
	return res;
}

int serial_init(char** dev_name, int dev_num, char* mode, speed_t speed)
{
	struct termios newtio;
	int index;

	for(index = 0; index < dev_num; index++) {
		fd[index] = open(dev_name[index], O_RDWR | O_NOCTTY);
		assert(fd[index] >= 0);
		if (!strcmp(mode, "232") && !test_pin(fd[index]))
			return ERROR_PIN_CHECK;

		if (tcgetattr(fd[index], &newtio) != 0) {
			return ERROR_GET_ATTR;
		}
		/* 串口特性设置 */
		tcflush(fd[index], TCIOFLUSH); //刷请输入输出缓存
		newtio.c_cflag &= ~(CSIZE | CSTOPB | PARENB | HUPCL); //1bit stop ,no parity check,清除校验位
		newtio.c_cflag |= CS8 | CREAD | CLOCAL; //8bit data
		newtio.c_iflag &= ~(INPCK | IXON | IXOFF | IXANY); //无需软件控制流
		newtio.c_iflag &= ~(INLCR | ICRNL); //无需回车和换行转换
		newtio.c_oflag &= ~(OPOST | ONLCR | OCRNL);
		newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN | ECHOK); //原始模式 //处理字符模式

		cfsetispeed(&newtio, speed); //输入波特率
		cfsetospeed(&newtio, speed); //输出波特率
		if (tcsetattr(fd[index], TCSANOW, &newtio) < 0) {
			printf("set param fail\r\n");
			return ERROR_GET_ATTR;
		}
		tcflush(fd[index], TCIOFLUSH);
	}
//	printf("Setup serial...\n");
	current_speed = speed;
	return 0;
}

void serial_exit()
{
	pthread_join(pth[0], NULL);
	if (pth[1] != -1) {
		pthread_join(pth[1], NULL);
		sem_destroy(sem);
		sem_destroy(sem + 1);
	}

	close(fd[0]);
	if (fd[1] > 0)
		close(fd[1]);
}

void half_direction_switch(int fd, enum com_dir dir)
{
	u_int32_t controlbits;

	ioctl(fd, TIOCMGET, &controlbits);
	if (dir == SERIAL_IN)
		controlbits &= ~TIOCM_RTS;
	else
		controlbits |= TIOCM_RTS;
	controlbits &= ~TIOCM_CTS;
	ioctl(fd, TIOCMSET, &controlbits);

	usleep(100000);
}

void* serial_test_handle(void *argv) {
	char cmd_receive[26];
	char cmd_send_command[26];
	struct thread_parameter para = *(struct thread_parameter*)argv;
	int fd_index = para.com_mode;
	struct termios newtio;
	com_mode_t mode = (fd_index == 0) ? M_S : S_M;
	char result[512] = {'\0'};

	srand((unsigned int)time(0));
	for (int i = 0; i < sizeof(cmd_send_command); i++)
		cmd_send_command[i] = 'A' + RANDROM(26);

	if (para.test_mode == SINGLE) {
		int res = ERROR_TX_RX;
		result[0] = '\0';
		for (int x = 0; x < sizeof(speed) / sizeof(speed_t); x++) {
			tcgetattr(fd[0], &newtio);
			cfsetispeed(&newtio, speed[x]); //输入波特率
			cfsetospeed(&newtio, speed[x]); //输出波特率
			tcsetattr(fd[0], TCSANOW, &newtio);
			tcflush(fd[0], TCIOFLUSH);
			usleep(100000);

			if (sizeof(cmd_send_command)
					!= uart_send(fd[0], cmd_send_command, sizeof(cmd_send_command))) {
				// printf(" send data fail\r\n");
				continue;
			}

			bzero(cmd_receive, sizeof(cmd_receive));
			if (sizeof(cmd_send_command)
					!= uart_recv_timeout(fd[0], cmd_receive, sizeof(cmd_send_command),
							timeout)) {
				// printf(" read data length not equal send data length\r\n");
				continue;
			}

			if (memcmp(cmd_send_command, cmd_receive, sizeof(cmd_send_command))) {
				// printf(" read data not equal send data\r\n");
				continue;
			}

			if (x > 0)
				strcat(result, " ");
			strcat(result, speed_name[x]);
			res = 0;
		}
		if (ERROR_TX_RX == res)
			printf("test tx rx failed\n");
		else
			printf("%s\n", result);
	} else {
		for (int i = 0; i < 2; i++) {
			if (mode == M_S) {
				result[0] = '\0';
				for (int x = 0; x < sizeof(speed) / sizeof(speed_t); x++) {
					if (para.serial == SERIAL_HALF)
						half_direction_switch(fd[fd_index], SERIAL_OUT);
					current_speed = speed[x];

					tcgetattr(fd[fd_index], &newtio);
					cfsetispeed(&newtio, speed[x]); //输入波特率
					cfsetospeed(&newtio, speed[x]); //输出波特率
					tcsetattr(fd[fd_index], TCSANOW, &newtio);
					tcflush(fd[fd_index], TCIOFLUSH);
					// usleep(500000);

					sem_post(sem + !fd_index);
					sem_wait(sem + fd_index);
					// usleep(100000);

					bzero(cmd_receive, sizeof(cmd_receive));
					uart_send(fd[fd_index], cmd_send_command, sizeof(cmd_send_command));

					if (para.serial == SERIAL_HALF) {
						sem_wait(sem + fd_index);
						if (para.serial == SERIAL_HALF)
						half_direction_switch(fd[fd_index], SERIAL_IN);
					}
					sem_post(sem + !fd_index);

					uart_recv_timeout(fd[fd_index], cmd_receive, sizeof(cmd_send_command), timeout);

					if (!memcmp(cmd_send_command, cmd_receive, sizeof(cmd_receive))) {
						if (x > 0)
							strcat(result, " ");
						strcat(result, speed_name[x]);
					}
				}
				mode = S_M;
				printf("%s\n", result);
			} else {
				for (;;) {
					sem_wait(sem + fd_index);
					if (para.serial == SERIAL_HALF)
						half_direction_switch(fd[fd_index], SERIAL_IN);
					tcgetattr(fd[fd_index], &newtio);
					cfsetispeed(&newtio, current_speed); //输入波特率
					cfsetospeed(&newtio, current_speed); //输出波特率
					tcsetattr(fd[fd_index], TCSANOW, &newtio);
					tcflush(fd[fd_index], TCIOFLUSH);
					// usleep(500000);
					sem_post(sem + !fd_index);

					bzero(cmd_receive, sizeof(cmd_receive));
					if (uart_recv_timeout(fd[fd_index], cmd_receive, 26, timeout)
							> 0) {
						if (para.serial == SERIAL_HALF) {
							half_direction_switch(fd[fd_index], SERIAL_OUT);
							sem_post(sem + !fd_index);
						}
						sem_wait(sem + fd_index);
						// usleep(100000);
						uart_send(fd[fd_index], cmd_receive, sizeof(cmd_receive));
					} else {
						if (para.serial == SERIAL_HALF)
							sem_post(sem + !fd_index);
						sem_wait(sem + fd_index);
					}
					if (current_speed == speed[sizeof(speed) / sizeof(speed[0]) - 1])
						break;
				}
				mode = M_S;
			}
			sleep(1);
		}
	}
	return NULL;
}

/*
 * argv[1] test mode(single/dual)
 * argv[2] serial mode(232/485/422)
 * argv[3] timeout
 * argv[4-5] devices
 * */
int main(int argc, char* argv[]) {
	int res = 0;
	char* dev_name[2] = {NULL, NULL};
	char* test_mode = NULL;
	char* mode = NULL;
	int dev_num = 1;
	struct thread_parameter para[2];

	if (argc < 5) {
		printf("serial tool usage:\n\n"
				"serial test_mode(single|dual) mode(232|485|422) timeout dev1 [dev2] \n\n");
		return ERROR_PARAM;
	}

	if (strcmp(argv[1], "single") && strcmp(argv[1], "dual")) {
		printf("no support test mode\n");
		return ERROR_PARAM;
	}
	test_mode = argv[1];
	mode = argv[2];
	timeout = atoi(argv[3]);
	dev_name[0] = argv[4];
	if (!strcmp(test_mode, "dual") && argc > 5) {
		dev_name[1] = argv[5];
		dev_num = 2;
		sem_init(sem, 0, 0);
		sem_init(sem + 1, 0, 0);
	}

	res = serial_init(dev_name, dev_num, mode, B110);
	if (res) {
		printf("serial init failed \n");
		return res;
	}

	// singal register
	signal(SIGINT, signal_handle);

	for (int i = 0; i < dev_num; i++) {
		para[i].com_mode = i;
		if (!strcmp(mode, "485")) {
			para[i].serial = SERIAL_HALF;
			para[i].test_mode = DUAL;
		} else {
			para[i].serial = SERIAL_FULL;
			if (!strcmp(test_mode, "dual"))
				para[i].test_mode = DUAL;
			else
				para[i].test_mode = SINGLE;
		}
		res = pthread_create(pth + i, NULL, serial_test_handle, para + i);
		assert(res == 0);
	}

	serial_exit();

	return res;
}
