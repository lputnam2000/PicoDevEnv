#ifndef _LWIPOPTS_EXAMPLE_COMMONH_H
#define _LWIPOPTS_EXAMPLE_COMMONH_H

// Enable socket support
#define LWIP_SOCKET 1

// Use system support (NO_SYS=0 means we're using an OS like FreeRTOS)
#define NO_SYS 0

// Prevent timeval redefinition conflict
#define LWIP_TIMEVAL_PRIVATE 0

// Configure memory and buffer sizes
#define MEM_SIZE 1600
#define MEMP_NUM_TCP_PCB 5
#define MEMP_NUM_UDP_PCB 5

#define TCP_SND_BUF 8192
#define TCP_WND 8192

#endif /* __LWIPOPTS_H__ */
