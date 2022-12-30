/******************************************************************************
* File Name:   cy_ppp_netif.h
*
* Description: This file is the public interface of cy_ppp_netif.c
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef SOURCE_CY_PPP_NETIF_H_
#define SOURCE_CY_PPP_NETIF_H_

#include <stdint.h>
#include <stdbool.h>

//#include "cy_lwip.h"
#include "netif/ppp/pppapi.h"
#include "cy_pcm_user.h"

#ifdef __cplusplus
extern "C" {
#endif


/*-- Public Definitions -------------------------------------------------*/

#define IPV4_FORMAT "%d.%d.%d.%d"

// ip4_addr1_16 defined in lwip/src/include/lwip/ip4_addr.h
#define IPV4_TO_STR(ipaddr) \
  ip4_addr1_16(ipaddr),     \
  ip4_addr2_16(ipaddr),     \
  ip4_addr3_16(ipaddr),     \
  ip4_addr4_16(ipaddr)

#define IPV6_FORMAT "%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x"

/* IP6_ADDR_BLOCK1(ip6addr) defined in lwip/src/include/lwip/ip6_addr.h */
#define IPV6_TO_STR(ipaddr)   \
  IP6_ADDR_BLOCK1(&(ipaddr)), \
  IP6_ADDR_BLOCK2(&(ipaddr)), \
  IP6_ADDR_BLOCK3(&(ipaddr)), \
  IP6_ADDR_BLOCK4(&(ipaddr)), \
  IP6_ADDR_BLOCK5(&(ipaddr)), \
  IP6_ADDR_BLOCK6(&(ipaddr)), \
  IP6_ADDR_BLOCK7(&(ipaddr)), \
  IP6_ADDR_BLOCK8(&(ipaddr))

typedef struct {
    ip_addr_t ip;
} cy_ppp_netif_dns_info_t;

typedef enum {
    IP_EVENT_PPP_GOT_IPV6,
    IP_EVENT_PPP_GOT_IPV4,
    IP_EVENT_PPP_LOST_IP,
} ip_event_t;

typedef struct {
    ip4_addr_t ip;
    ip4_addr_t netmask;
    ip4_addr_t gw;
} cy_ppp_netif_ip_info_t;

typedef struct {
    ip6_addr_t ip;
} cy_ppp_netif_ip6_info_t;


typedef void (*on_error_event_fn)(int32_t event_id, void *event_data_p);
typedef void (*on_ip_event_fn)(int32_t event_id, void *event_data_p);

struct cy_ppp_netif_s {
    struct netif *lwip_netif_p;
    ppp_pcb *ppp_pcb_p;

    void *modem_handle;
    bool (*modem_transmit)(void *handle, void *buffer_p, size_t size);

    on_error_event_fn error_event_fn;
    on_ip_event_fn ip_event_fn;

    /* user-supplied callback to be invoked when IP is lost */
    on_user_ip_lost_fn user_ip_lost_fn;
};
typedef struct cy_ppp_netif_s cy_ppp_netif_t;


typedef struct {
    cy_ppp_netif_t *ppp_netif_p;
    cy_ppp_netif_ip_info_t ip_info;
    bool ip_changed;
} ip_event_got_ip_t;


typedef struct {
    cy_ppp_netif_t *ppp_netif_p;
    cy_ppp_netif_ip6_info_t ip6_info;
} ip_event_got_ip6_t;


typedef struct cy_ppp_netif_config {
    on_error_event_fn error_event_fn;
    on_ip_event_fn ip_event_fn;
    on_user_ip_lost_fn user_ip_lost_fn;
} cy_ppp_netif_config_t;


/*-- Public Functions -------------------------------------------------*/

bool cy_ppp_netif_init(bool wifi_started);

void cy_ppp_netif_deinit(void);

cy_ppp_netif_t *cy_ppp_netif_new(const cy_ppp_netif_config_t *ppp_netif_config);

void cy_ppp_netif_delete(cy_ppp_netif_t *ppp_netif_p);

bool cy_ppp_netif_start(cy_ppp_netif_t *ppp_netif_p);

bool cy_ppp_netif_stop(cy_ppp_netif_t *ppp_netif_p);

/* passes data from modem to network stack */
bool cy_ppp_netif_receive(cy_ppp_netif_t *ppp_netif_p,
                          void *buffer_p,
                          size_t size);

bool cy_ppp_netif_is_netif_up(cy_ppp_netif_t *ppp_netif_p);

bool cy_ppp_netif_get_dns_info( cy_ppp_netif_t *ppp_netif_p,
                                uint8_t numdns,
                                cy_ppp_netif_dns_info_t *dns);

void cy_netif_ppp_set_auth( cy_ppp_netif_t *ppp_netif_p,
                            uint8_t auth_type,
                            const char *username_p,
                            const char *password_p);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CY_PPP_NETIF_H_ */
