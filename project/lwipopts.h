#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Enable OS support and socket API
#define NO_SYS 0
#define LWIP_SOCKET 1

#include "lwipopts_examples_common.h"

// Since NO_SYS is now 0, the following settings will be included
#if !NO_SYS
#define TCPIP_THREAD_STACKSIZE 4096 // Increase stack size if necessary
#define DEFAULT_THREAD_STACKSIZE 4096
#define DEFAULT_RAW_RECVMBOX_SIZE 12
#define TCPIP_MBOX_SIZE 12
#define LWIP_TIMEVAL_PRIVATE 0

// Keep MEM_SIZE increased for sufficient memory allocation
#define MEM_SIZE 16384 // 16 KB or higher if needed

// Increase other relevant buffer sizes
#define MEMP_NUM_TCP_PCB 10
#define MEMP_NUM_TCP_PCB_LISTEN 10
#define MEMP_NUM_TCP_SEG 32
#define PBUF_POOL_SIZE 24

// Enable required features
#define SYS_LIGHTWEIGHT_PROT 1
#define LWIP_NETCONN 1

// Enable debugging if needed
#define LWIP_DEBUG 0
#define TCP_DEBUG LWIP_DBG_OFF
#define SOCKETS_DEBUG LWIP_DBG_OFF
#define MEM_DEBUG LWIP_DBG_ON
#define MEMP_DEBUG LWIP_DBG_ON

#define DEFAULT_UDP_RECVMBOX_SIZE 12 // Probably wont need this because im using TCP but havent test
#define DEFAULT_TCP_RECVMBOX_SIZE 12 // To get rid of ***PANIC*** size > 0 error
#define DEFAULT_ACCEPTMBOX_SIZE TCPIP_MBOX_SIZE

#define LWIP_SO_RCVBUF 1
#define RECV_BUFSIZE_DEFAULT 256
#define TCPIP_THREAD_PRIO 2

// #define TCP_SYNMAXRTX 12                   // Increase the maximum SYN retries (default is 6)
// #define TCP_SND_QUEUELEN (8 * TCP_SND_BUF) // Ensure the send queue is large enough

// Enable input core locking if desired
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

// Enable socket receive timeout
#define LWIP_SO_RCVTIMEO 1
#endif

#endif /* _LWIPOPTS_H */
