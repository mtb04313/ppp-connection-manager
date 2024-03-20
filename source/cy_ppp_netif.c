/******************************************************************************
* File Name:   cy_ppp_netif.c
*
* Description: This file implements the network interface used by PPP.
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

#include "feature_config.h" // for FEATURE_WIFI
#include "cy_ppp_netif.h"

#include "cy_debug.h"
#include "cy_string.h"
#include "cy_memtrack.h"

#include <lwipopts.h>
#include <lwip/dns.h>
#include <lwip/netifapi.h>


/*-- Local Definitions -------------------------------------------------*/


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "ppp_netif";

#if (FEATURE_WIFI != ENABLE_FEATURE)
static bool s_tcpip_initialized = false;
#endif


/*-- Local Functions -------------------------------------------------*/

/* interface with LWIP */
static void on_ppp_status_changed(ppp_pcb *pcb_p,
                                  int err_code,
                                  void *ctx)
{
    struct netif *pppif = ppp_netif(pcb_p);
    cy_ppp_netif_t *ppp_netif_p = ctx;

    ip_event_got_ip_t evt = {
        .ppp_netif_p = ppp_netif_p,
    };

    switch (err_code) {
    case PPPERR_NONE:
        CY_LOGI(TAG, "Connected");

        if (pcb_p->if4_up && !ip_addr_isany(&pppif->ip_addr)) {
#if PPP_IPV6_SUPPORT
            evt.ip_info.ip.addr = pppif->ip_addr.u_addr.ip4.addr;
            evt.ip_info.gw.addr = pppif->gw.u_addr.ip4.addr;
            evt.ip_info.netmask.addr = pppif->netmask.u_addr.ip4.addr;
#else
            // only IPv4 supported
            evt.ip_info.ip.addr = pppif->ip_addr.addr;
            evt.ip_info.gw.addr = pppif->gw.addr;
            evt.ip_info.netmask.addr = pppif->netmask.addr;
#endif

            if (ppp_netif_p->ip_event_fn != NULL) {
                ppp_netif_p->ip_event_fn( IP_EVENT_PPP_GOT_IPV4,
                                          &evt);
            }
            return;

#if PPP_IPV6_SUPPORT
        } else if (pcb_p->if6_up && !ip_addr_isany(&pppif->ip6_addr[0])) {
            cy_ppp_netif_ip6_info_t ip6_info;
            ip6_addr_t lwip_ip6_info;

            ip_event_got_ip6_t ip6_event = {
                .ppp_netif_p = pppif->state,
            };

            ip6_addr_set(&lwip_ip6_info, ip_2_ip6(&pppif->ip6_addr[0]));
#if LWIP_IPV6_SCOPES
            memcpy(&ip6_info.ip, &lwip_ip6_info, sizeof(ip6_addr_t));

#else
            memcpy(&ip6_info.ip, &lwip_ip6_info, sizeof(ip6_addr_t));
            ip6_info.ip.zone = 0;   // zero out zone, as not used in lwip
#endif /* LWIP_IPV6_SCOPES */

            memcpy(&ip6_event.ip6_info, &ip6_info, sizeof(cy_ppp_netif_ip6_info_t));

            if (ppp_netif_p->ip_event_fn != NULL) {
                ppp_netif_p->ip_event_fn( IP_EVENT_PPP_GOT_IPV6,
                                          &ip6_event);
            }
            return;
#endif /* PPP_IPV6_SUPPORT */

        } else {
            CY_LOGE(TAG, "Unexpected connected event");
            return;
        }
        break;

    case PPPERR_PARAM:
        CY_LOGE(TAG, "Invalid parameter");
        break;
    case PPPERR_OPEN:
        CY_LOGE(TAG, "Unable to open PPP session");
        break;
    case PPPERR_DEVICE:
        CY_LOGE(TAG, "Invalid I/O device for PPP");
        break;
    case PPPERR_ALLOC:
        CY_LOGE(TAG, "Unable to allocate resources");
        break;
    case PPPERR_USER:
        CY_LOGI(TAG, "User interrupt");
        break;

    case PPPERR_CONNECT:
        CY_LOGI(TAG, "Connection lost");

        if (ppp_netif_p->ip_event_fn != NULL) {
            ppp_netif_p->ip_event_fn( IP_EVENT_PPP_LOST_IP,
                                      &evt);
        }

        if (ppp_netif_p->user_ip_lost_fn != NULL) {
            ppp_netif_p->user_ip_lost_fn();
        }
        return;

    case PPPERR_AUTHFAIL:
        CY_LOGE(TAG, "Failed authentication challenge");
        break;
    case PPPERR_PROTOCOL:
        CY_LOGE(TAG, "Failed to meet protocol");
        break;
    case PPPERR_PEERDEAD:
        CY_LOGE(TAG, "Connection timeout");
        break;
    case PPPERR_IDLETIMEOUT:
        CY_LOGE(TAG, "Idle Timeout");
        break;
    case PPPERR_CONNECTTIME:
        CY_LOGE(TAG, "Max connect time reached");
        break;
    case PPPERR_LOOPBACK:
        CY_LOGE(TAG, "Loopback detected");
        break;

    default:
        CY_LOGE(TAG, "Unknown error code %d", err_code);
        break;
    }

    if (ppp_netif_p->error_event_fn != NULL) {
        ppp_netif_p->error_event_fn(err_code,
                                    ppp_netif_p);
    }
}

/* interface with LWIP */
#if PPP_NOTIFY_PHASE
static void on_ppp_notify_phase(ppp_pcb *pcb,
                                u8_t phase,
                                void *ctx)
{
    switch (phase) {
    case PPP_PHASE_DEAD:
        CY_LOGD(TAG, "PHASE_DEAD");
        break;
    case PPP_PHASE_INITIALIZE:
        CY_LOGD(TAG, "PHASE_INITIALIZE");
        break;
    case PPP_PHASE_ESTABLISH:
        CY_LOGD(TAG, "PHASE_ESTABLISH");
        break;
    case PPP_PHASE_AUTHENTICATE:
        CY_LOGD(TAG, "PHASE_AUTHENTICATE");
        break;
    case PPP_PHASE_NETWORK:
        CY_LOGD(TAG, "PHASE_NETWORK");
        break;
    case PPP_PHASE_RUNNING:
        CY_LOGD(TAG, "PHASE_RUNNING");
        break;
    case PPP_PHASE_TERMINATE:
        CY_LOGD(TAG, "PHASE_TERMINATE");
        break;
    case PPP_PHASE_DISCONNECT:
        CY_LOGD(TAG, "PHASE_DISCONNECT");
        break;
    default:
        CY_LOGW(TAG, "PHASE %u (UNKNOWN)", phase);
        break;
    }
}
#endif /* PPP_NOTIFY_PHASE */


/* interface with LWIP */
static uint32_t ppp_low_level_output( ppp_pcb *pcb_p,
                                      uint8_t *data_p,
                                      uint32_t size,
                                      void *netif_p)
{
    cy_ppp_netif_t *ppp_netif_p = netif_p;
    if ((ppp_netif_p->modem_transmit)(ppp_netif_p->modem_handle,
                                      data_p,
                                      size)) {
        return size;
    }

    return 0;
}

/* interface with LWIP */
static ppp_pcb *ppp_netif_new_pcb(cy_ppp_netif_t *ppp_netif_p)
{
    ppp_netif_p->ppp_pcb_p = pppapi_pppos_create( ppp_netif_p->lwip_netif_p,
                             ppp_low_level_output,
                             on_ppp_status_changed,
                             ppp_netif_p);

    if (ppp_netif_p->ppp_pcb_p == NULL) {
        CY_LOGE(TAG, "%s [%d]: pppapi_pppos_create failed",
                __FUNCTION__, __LINE__);
        return NULL;
    }

#if PPP_NOTIFY_PHASE
    ppp_set_notify_phase_callback(ppp_netif_p->ppp_pcb_p,
                                  on_ppp_notify_phase);
#endif

    ppp_set_usepeerdns(ppp_netif_p->ppp_pcb_p, 1);

    return (ppp_netif_p->ppp_pcb_p);
}

static void ppp_netif_lwip_remove(cy_ppp_netif_t *ppp_netif_p)
{
    if (ppp_netif_p->lwip_netif_p != NULL) {
        if (netif_is_up(ppp_netif_p->lwip_netif_p)) {
            netif_set_down(ppp_netif_p->lwip_netif_p);
        }
        netif_remove(ppp_netif_p->lwip_netif_p);
    }
}


/*-- Public Functions -------------------------------------------------*/

bool cy_ppp_netif_init(bool wifi_started)
{
#if (FEATURE_WIFI != ENABLE_FEATURE)
    if (!wifi_started) {
        if (!s_tcpip_initialized) {
            tcpip_init(NULL, NULL);
            s_tcpip_initialized = true;
            CY_LOGD(TAG, "%s [%d]: tcpip_init", __FUNCTION__, __LINE__);
        }
    }
#endif

    CY_LOGD(TAG, "%s [%d]: done", __FUNCTION__, __LINE__);
    return true;
}

void cy_ppp_netif_deinit(void)
{
    /* do nothing */
}

cy_ppp_netif_t *cy_ppp_netif_new(const cy_ppp_netif_config_t *ppp_netif_config_p)
{
    ReturnAssert(ppp_netif_config_p != NULL, NULL);

    cy_ppp_netif_t *ppp_netif_p = CY_MEMTRACK_CALLOC(1, sizeof(*ppp_netif_p));
    if (ppp_netif_p == NULL) {
        CY_LOGE(TAG, "ppp_netif_p calloc failed");
        return NULL;
    }

    do {
        struct netif *lwip_netif_p = CY_MEMTRACK_CALLOC(1, sizeof(*lwip_netif_p));
        if (lwip_netif_p == NULL) {
            CY_LOGE(TAG, "lwip_netif_p calloc failed");
            break;
        }

        lwip_netif_p->state = ppp_netif_p;
        ppp_netif_p->lwip_netif_p = lwip_netif_p;

        ppp_netif_p->error_event_fn = ppp_netif_config_p->error_event_fn;
        ppp_netif_p->ip_event_fn = ppp_netif_config_p->ip_event_fn;
        ppp_netif_p->user_ip_lost_fn = ppp_netif_config_p->user_ip_lost_fn;

        ppp_netif_p->ppp_pcb_p = ppp_netif_new_pcb(ppp_netif_p);
        if (ppp_netif_p->ppp_pcb_p == NULL) {
            CY_LOGE(TAG, "ppp_netif_new_pcb failed!");
            break;
        }

        return ppp_netif_p;

    } while (false);

    cy_ppp_netif_delete(ppp_netif_p);
    return NULL;
}

void cy_ppp_netif_delete(cy_ppp_netif_t *ppp_netif_p)
{
    if (ppp_netif_p != NULL) {
        ppp_netif_lwip_remove(ppp_netif_p);

        if (ppp_netif_p->ppp_pcb_p != NULL) {
            pppapi_free(ppp_netif_p->ppp_pcb_p);
        }

        CY_MEMTRACK_FREE(ppp_netif_p->lwip_netif_p);
        CY_MEMTRACK_FREE(ppp_netif_p);
    }
}

bool cy_ppp_netif_start(cy_ppp_netif_t *ppp_netif_p)
{
    bool result;
    err_t err_ret;

    ReturnAssert(ppp_netif_p != NULL, false);
    ReturnAssert(ppp_netif_p->ppp_pcb_p != NULL, false);

    CY_LOGD(TAG, "%s [%d]: starting PPP", __FUNCTION__, __LINE__);
    err_ret = pppapi_connect(ppp_netif_p->ppp_pcb_p, 0);
    result = (err_ret == ERR_OK)? true : false;

    if (result) {
        netifapi_netif_set_default(ppp_netif_p->lwip_netif_p);
    }

    return result;
}

bool cy_ppp_netif_stop(cy_ppp_netif_t *ppp_netif_p)
{
    bool result;
    err_t err_ret;

    ReturnAssert(ppp_netif_p != NULL, false);
    ReturnAssert(ppp_netif_p->ppp_pcb_p != NULL, false);

    CY_LOGD(TAG, "%s [%d]: stopping PPP", __FUNCTION__, __LINE__);
    err_ret = pppapi_close(ppp_netif_p->ppp_pcb_p, 0);
    result = (err_ret == ERR_OK)? true : false;

    if (result) {
        netifapi_netif_set_default(NULL);
    }

    return result;
}


bool cy_ppp_netif_receive(cy_ppp_netif_t *ppp_netif_p,
                          void *buffer_p,
                          size_t size)
{
    err_t err_ret;

    ReturnAssert(ppp_netif_p != NULL, false);
    ReturnAssert(ppp_netif_p->ppp_pcb_p != NULL, false);

    err_ret = pppos_input_tcpip(ppp_netif_p->ppp_pcb_p, buffer_p, size);
    if (err_ret != ERR_OK) {
        CY_LOGE(TAG, "%s [%d]: pppos_input_tcpip failed with %d",
                __FUNCTION__, __LINE__, err_ret);
        return false;
    }

    return true;
}

bool cy_ppp_netif_is_netif_up(cy_ppp_netif_t *ppp_netif_p)
{
    if ((ppp_netif_p != NULL) &&
            (ppp_netif_p->lwip_netif_p != NULL)) {
        ReturnAssert(ppp_netif_p->ppp_pcb_p != NULL, false);

        return netif_is_link_up(ppp_netif_p->lwip_netif_p);
    }

    return false;
}


bool cy_ppp_netif_get_dns_info( cy_ppp_netif_t *ppp_netif_p,
                                uint8_t numdns,
                                cy_ppp_netif_dns_info_t *dns_p)
{
    ReturnAssert(ppp_netif_p != NULL, false);
    ReturnAssert(ppp_netif_p->ppp_pcb_p != NULL, false);

    const ip_addr_t *addr_p = dns_getserver(numdns); /* LWIP dns.c */

    if (addr_p == IP_ADDR_ANY) {
        return false;
    }

    memcpy( &dns_p->ip,
            addr_p,
            sizeof(ip_addr_t));

    return true;
}


void cy_netif_ppp_set_auth( cy_ppp_netif_t *ppp_netif_p,
                            uint8_t auth_type,
                            const char *username_p,
                            const char *password_p)
{
    VoidAssert(ppp_netif_p != NULL);
    VoidAssert(ppp_netif_p->ppp_pcb_p != NULL);

    // okay to call this during PHASE_DEAD
    ppp_set_auth( ppp_netif_p->ppp_pcb_p,
                  auth_type,
                  username_p,
                  password_p);
}
