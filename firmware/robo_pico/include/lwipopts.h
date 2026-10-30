#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Basic lwIP settings
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    20000     // ← INCREASE from 8000 to 20KB
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define TCP_WND                     (4 * TCP_MSS)   // ← REDUCE from 8 to 4
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (4 * TCP_MSS)   // ← REDUCE from 8 to 4
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETCONN                0
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_DHCP                   1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0

// Timeout/timer support (CRITICAL FOR MQTT)
#define LWIP_TIMERS                 1
#define MEMP_NUM_SYS_TIMEOUT        16        // ← INCREASE from 8 to 16

// MQTT-specific settings
#define LWIP_MQTT                   1
#define LWIP_CALLBACK_API           1
#define MQTT_VAR_HEADER_BUFFER_LEN  256       // ← INCREASE from 128 to 256
#define MQTT_REQ_MAX_IN_FLIGHT      16        // Shared by subscriptions, status and telemetry.

// MQTT output ring buffer (important for publishing)
#define MQTT_OUTPUT_RINGBUF_SIZE    8192       // ← ADD THIS LINE

#endif /* __LWIPOPTS_H__ */
