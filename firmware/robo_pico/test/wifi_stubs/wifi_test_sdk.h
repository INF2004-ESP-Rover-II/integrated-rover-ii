#ifndef WIFI_TEST_SDK_H
#define WIFI_TEST_SDK_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int err_t;
typedef uint8_t u8_t;typedef uint16_t u16_t;typedef uint32_t u32_t;
#define ERR_OK 0
#define ERR_MEM -1
#define ERR_VAL -6
#define ERR_ISCONN -10
#define MQTT_DATA_FLAG_LAST 1u
#define MQTT_REQ_MAX_IN_FLIGHT 16u
#define MQTT_OUTPUT_RINGBUF_SIZE 16384u
#define CYW43_ITF_STA 0
#define CYW43_AUTH_WPA2_AES_PSK 1u
#define CYW43_NO_POWERSAVE_MODE 0u
#define CYW43_LINK_DOWN 0
#define CYW43_LINK_JOIN 1
#define CYW43_LINK_NOIP 2
#define CYW43_LINK_UP 3
#define CYW43_LINK_FAIL -1
#define CYW43_LINK_NONET -2
#define CYW43_LINK_BADAUTH -3
typedef struct {uint32_t addr;} ip_addr_t;
struct netif {ip_addr_t address;};
#define netif_ip4_addr(n) (&(n)->address)
typedef struct {struct netif netif[2];} cyw43_t;
extern cyw43_t cyw43_state;
extern struct netif *netif_list;
typedef struct {bool connected;} mqtt_client_t;
typedef enum {
    MQTT_CONNECT_ACCEPTED=0,MQTT_CONNECT_REFUSED_PROTOCOL_VERSION,
    MQTT_CONNECT_REFUSED_IDENTIFIER,MQTT_CONNECT_REFUSED_SERVER,
    MQTT_CONNECT_REFUSED_USERNAME_PASS,MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_,
    MQTT_CONNECT_DISCONNECTED=256,MQTT_CONNECT_TIMEOUT=257
} mqtt_connection_status_t;
struct mqtt_connect_client_info_t {const char *client_id;uint16_t keep_alive;};
typedef void (*mqtt_request_cb_t)(void *,err_t);
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *,void *,mqtt_connection_status_t);
typedef void (*mqtt_incoming_publish_cb_t)(void *,const char *,u32_t);
typedef void (*mqtt_incoming_data_cb_t)(void *,const u8_t *,u16_t,u8_t);
int cyw43_arch_init(void);
void cyw43_arch_enable_sta_mode(void);
int cyw43_arch_wifi_connect_timeout_ms(const char *,const char *,uint32_t,uint32_t);
int cyw43_arch_wifi_connect_async(const char *,const char *,uint32_t);
int cyw43_wifi_link_status(cyw43_t *,int);
int cyw43_tcpip_link_status(cyw43_t *,int);
void cyw43_arch_poll(void);
int cyw43_wifi_pm(cyw43_t *,uint32_t);
uint32_t cyw43_pm_value(unsigned,unsigned,unsigned,unsigned,unsigned);
uint32_t get_absolute_time(void);
uint32_t to_ms_since_boot(uint32_t);
int ip4addr_aton(const char *,ip_addr_t *);
const char *ip4addr_ntoa(const ip_addr_t *);
mqtt_client_t *mqtt_client_new(void);
int mqtt_client_is_connected(mqtt_client_t *);
err_t mqtt_client_connect(mqtt_client_t *,const ip_addr_t *,uint16_t,mqtt_connection_cb_t,void *,const struct mqtt_connect_client_info_t *);
err_t mqtt_publish(mqtt_client_t *,const char *,const void *,u16_t,u8_t,u8_t,mqtt_request_cb_t,void *);
err_t mqtt_subscribe(mqtt_client_t *,const char *,u8_t,mqtt_request_cb_t,void *);
void mqtt_set_inpub_callback(mqtt_client_t *,mqtt_incoming_publish_cb_t,mqtt_incoming_data_cb_t,void *);
#endif
