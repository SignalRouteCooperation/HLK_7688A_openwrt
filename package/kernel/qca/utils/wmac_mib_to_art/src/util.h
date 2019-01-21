#ifndef UTIL_H
#define UTIL_H

typedef unsigned short u16;

int IsPathExist(const char *path_name);
void WriteLog2Serial(const char *format, ...);
int ExecuateShellCmd(char * shellCMD, char * r_buffer, size_t r_buffer_len);
int SafeWrite2File(const char *fn, const char *buf, size_t buflen);
char *Trim(char *str);
int GetUciField(const char *uci_show_cmd, char *field_buf, size_t field_buf_len);
u16 Str2U16(char *str);
#endif // UTIL_H
