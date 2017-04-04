

#ifndef _LINUX_EXECD_TEMP_H
#define _LINUX_EXECD_TEMP_H
asmlinkage long sys_setpriority(int which, int who, int niceval);
asmlinkage long sys_getpriority(int which, int who);

#endif


