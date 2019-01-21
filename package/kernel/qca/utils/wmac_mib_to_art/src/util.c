/* created by gukq.20160420 gukaiqiang@kunteng.org
 *
 */

#include <stdlib.h>
#include <sys/file.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "util.h"

#define LOG_BUFFER_SIZE 4096
#define CMD_LEN 512

// exist 1, not exist return 0
int IsPathExist(const char *path_name) {
	if (access(path_name, F_OK) != -1) {
		return 1;
	}else{
		return 0;
	}
}

// victor add
// print log to serial
// gukq modified it to support Variable Arguments and more
void WriteLog2Serial(const char *format, ...)
{
	char log[LOG_BUFFER_SIZE] = {0};
	char print_cmd[LOG_BUFFER_SIZE + 32] = {0};
	char buf[32] = {0};

	va_list args;
	va_start(args, format);
	vsprintf(log, format, args);
	va_end(args);

	SafeWrite2File("/dev/ttyS0", log, LOG_BUFFER_SIZE);
}

// ExecuateShellCMD()
// victortang@20160114
// Input:	shellCMD	-	shell CMD
// r_buffer	-	result buffer
// return:bit count -exe shell CMD success. -1 -error
int ExecuateShellCmd(char *shellCMD, char *r_buffer, size_t r_buffer_len)
{
	FILE *fstream=NULL;

	if( NULL == (fstream=popen(shellCMD, "r"))){
		WriteLog2Serial("execute command failed: %s", shellCMD);
		return -1;
	}

	char c = 0;
	int i = 0;
	if (r_buffer) {		//if r_buffer is NULL, do never write cmd output
		while((c = fgetc(fstream)) != EOF) {
			if (!(isdigit(c) || isalpha(c) || isspace(c) || ispunct(c))) {
				break;
			}

			if ( i<r_buffer_len ) {
				r_buffer[i] = c;
				i++;
			} else {
				break;
			}
		}
	}
	pclose(fstream);

	return i;
}

// gukq created
// if cannot open file fn, return -1, or return written size when sccessed
int SafeWrite2File(const char *fn, const char *buf, size_t buflen) {
	int fd;
	int written_size = 0;

	fd = open(fn, O_WRONLY|O_CREAT);
	if ( fd == -1 ) {
		return -1;
	}

	flock(fd, LOCK_EX);
	written_size = write(fd, buf, buflen);
	close(fd);
	flock(fd, LOCK_UN);
	return (int) written_size;
}

// Note: This function returns a pointer to a substring of the original string.
// If the given string was allocated dynamically, the caller must not overwrite
// that pointer with the returned value, since the original pointer must be
// deallocated using the same allocator with which it was allocated.  The return
// value must NOT be deallocated using free() etc.
char *Trim(char *str) {
	char *end;

	// Trim leading space
	while(isspace(*str)) str++;

	if(*str == 0)  // All spaces?
		return str;

	// Trim trailing space
	end = str + strlen(str) - 1;
	while(end > str && isspace(*end)) end--;

	// Write new null terminator
	*(end+1) = 0;

	return str;
}

// get configuration field by uci.
// return -1 when executed error and succeed return result bits
// count(upto ExecuateShellCmd func and usually >=0 )
int GetUciField(const char *uci_show_cmd, char *field_buf, size_t field_buf_len) {
	char cmd_buf[CMD_LEN] = {0};
	memset(cmd_buf, 0, sizeof(cmd_buf));
	sprintf(cmd_buf, "%s", uci_show_cmd);

	int exec_r = 0;
	if ((exec_r = ExecuateShellCmd(cmd_buf, field_buf, field_buf_len)) < 0) {
		return -1;
	}
	Trim(field_buf);

	return exec_r;
}

u16 Str2U16(char *str)
{
	int len = 2;
	short int ivalue = 0;
	int i;
	for (i = 0; i < len; i++)
	{
		if ((str[i] <= '9' && str[i] >= '0'))
		{
			ivalue = ivalue * 16 + (str[i] - '0');
		}
		else if ((str[i] >= 'a' && str[i] <= 'f'))
		{
			ivalue = ivalue * 16 + (str[i] - 'a') + 10;
		}
		else if ((str[i] >= 'A' && str[i] <= 'F'))
		{
			ivalue = ivalue * 16 + (str[i] - 'A') + 10;
		}

	}
	return ivalue;
}
