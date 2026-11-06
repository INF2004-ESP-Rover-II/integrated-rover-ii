/* Production wifi_mqtt.c with a fake SDK: no real network or credentials. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "wifi_mqtt.h"
#include "wifi_test_sdk.h"
cyw43_t cyw43_state;
struct netif *netif_list=&cyw43_state.netif[0];
static int tcp_state=CYW43_LINK_DOWN,join_calls,publish_calls;
static mqtt_client_t client;
static char published_topic[32];
int cyw43_arch_init(void){return 0;}
void cyw43_arch_enable_sta_mode(void){}
int cyw43_arch_wifi_connect_async(const char *ssid,const char *pw,uint32_t auth){
    (void)ssid;(void)pw;(void)auth;++join_calls;return 0;
}
int cyw43_arch_wifi_connect_timeout_ms(const char *s,const char *p,uint32_t a,uint32_t t){(void)t;return cyw43_arch_wifi_connect_async(s,p,a);}
int cyw43_wifi_link_status(cyw43_t *s,int itf){(void)s;(void)itf;return CYW43_LINK_JOIN;}
int cyw43_tcpip_link_status(cyw43_t *s,int itf){(void)s;(void)itf;return tcp_state;}
void cyw43_arch_poll(void){}
int cyw43_wifi_pm(cyw43_t *s,uint32_t p){(void)s;(void)p;return 0;}
uint32_t cyw43_pm_value(unsigned a,unsigned b,unsigned c,unsigned d,unsigned e){(void)a;(void)b;(void)c;(void)d;(void)e;return 0;}
uint32_t get_absolute_time(void){return 1000;}
uint32_t to_ms_since_boot(uint32_t t){return t;}
int ip4addr_aton(const char *text,ip_addr_t *ip){(void)text;ip->addr=1;return 1;}
const char *ip4addr_ntoa(const ip_addr_t *ip){(void)ip;return "192.0.2.10";}
mqtt_client_t *mqtt_client_new(void){return &client;}
int mqtt_client_is_connected(mqtt_client_t *c){return c->connected;}
err_t mqtt_client_connect(mqtt_client_t *c,const ip_addr_t *ip,uint16_t port,mqtt_connection_cb_t callback,void *arg,const struct mqtt_connect_client_info_t *info){
    (void)ip;(void)port;(void)info;c->connected=true;callback(c,arg,MQTT_CONNECT_ACCEPTED);return ERR_OK;
}
err_t mqtt_publish(mqtt_client_t *c,const char *topic,const void *data,u16_t length,u8_t qos,u8_t retain,mqtt_request_cb_t callback,void *arg){
    (void)c;(void)qos;(void)retain;assert(data&&length>0);
    snprintf(published_topic,sizeof(published_topic),"%s",topic);++publish_calls;
    callback(arg,ERR_OK);return ERR_OK;
}
err_t mqtt_subscribe(mqtt_client_t *c,const char *t,u8_t qos,mqtt_request_cb_t cb,void *a){(void)c;(void)t;(void)qos;(void)cb;(void)a;return ERR_OK;}
void mqtt_set_inpub_callback(mqtt_client_t *c,mqtt_incoming_publish_cb_t p,mqtt_incoming_data_cb_t d,void *a){(void)c;(void)p;(void)d;(void)a;}
int main(void){
    assert(!wifi_is_connected()&&wifi_link_status()==CYW43_LINK_DOWN);
    assert(wifi_init()==WIFI_OK);
    assert(wifi_connect_async("test-network","test-password")==WIFI_OK&&join_calls==1);
    const int not_ready[]={CYW43_LINK_DOWN,CYW43_LINK_JOIN,CYW43_LINK_NOIP,CYW43_LINK_FAIL,CYW43_LINK_NONET,CYW43_LINK_BADAUTH};
    for(unsigned i=0;i<sizeof(not_ready)/sizeof(not_ready[0]);++i){
        tcp_state=not_ready[i];assert(!wifi_is_connected());
    }
    tcp_state=CYW43_LINK_NOIP;assert(mqtt_connect()==WIFI_ERR_NOT_CONNECTED);
    tcp_state=CYW43_LINK_UP;
    /* Regression: raw Wi-Fi stays JOIN while the overall link becomes UP. */
    assert(cyw43_wifi_link_status(&cyw43_state,0)==CYW43_LINK_JOIN);
    assert(wifi_is_connected());wifi_mqtt_poll();
    assert(strcmp(wifi_get_ip_address(),"192.0.2.10")==0);
    assert(mqtt_init("192.0.2.10",1883,"test-client")==WIFI_OK);
    assert(mqtt_connect()==WIFI_OK&&mqtt_is_connected());
    assert(mqtt_publish_map("{\"state\":\"IDLE\",\"markers\":[]}")==WIFI_OK);
    assert(publish_calls==1&&!strcmp(published_topic,"robot/map"));
    assert(mqtt_publish_telemetry("{}")==WIFI_OK&&publish_calls==2);
    tcp_state=CYW43_LINK_DOWN;assert(!wifi_is_connected());wifi_mqtt_poll();
    puts("PASS: joining/DHCP readiness -> MQTT connection -> robot/map; link loss detected");
    return 0;
}
