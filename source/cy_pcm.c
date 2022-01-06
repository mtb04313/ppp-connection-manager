/******************************************************************************
* File Name:   cy_pcm.c
*
* Description: This file implements the PPP Connection Manager
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

#include "feature_config.h"
#include <stdlib.h>
#include <string.h>

#include "cyabs_rtos.h"
#include "cy_pcm.h"
#include "cy_lwip.h"  /* WIFI LwIP interface */
#include "lwip/netifapi.h"

#include "cy_debug.h"
#include "cy_ppp_netif.h"
#include "cy_modem.h"
#include "cy_modem_netif.h"


/*-- Local Definitions -------------------------------------------------*/

#define LINE_BUFFER_SIZE    512

#define BIT1     0x00000002
#define BIT0     0x00000001

#define WAIT_INTERVAL_FOR_PPP_CONNECT   90000 // 90sec for BG96 //was 20000 (SimCom)
#define WAIT_INTERVAL_FOR_IP_ADDRESS    1000

#define WAIT_INTERVAL_FOR_PPP_STOP      3000
#define WAIT_INTERVAL_FOR_PPP_ABORT     20000


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "pcm";

static cy_event_t s_event_group = NULL;
static const int CONNECT_BIT = BIT0;
static const int ABORT_BIT = BIT1;

static cy_modem_t *s_modem_p = NULL;
static cy_ppp_netif_t *s_ppp_netif_p = NULL;
static bool s_is_pcm_initialized = false;
static bool s_is_ppp_connected = false;

static connectivity_t s_default_connectivity = CELLULAR_CONNECTIVITY;
static cy_mutex_t s_operationMutex;  /* mutex prevents concurrent operations */


/*-- Local Functions -------------------------------------------------*/

static void ppp_client_error_event( int32_t event_id,
                                    void *event_data_p)
{
    CY_LOGI(TAG, "PPP error event %ld", event_id);
    if (event_id == PPPERR_USER) {
        cy_ppp_netif_t *ppp_netif_p = event_data_p;
        cy_time_t now = 0;

        cy_rtos_get_time(&now);
        CY_LOGI(TAG, "{%ld} User interrupted event from netif %p", now, ppp_netif_p);

        if (s_event_group != NULL) {
            cy_rslt_t result = cy_rtos_setbits_event(&s_event_group,
                               ABORT_BIT,
                               false);
            (void)result;
        }
    }
}

static void ppp_client_ip_event(int32_t event_id,
                                void *event_data_p)
{
    cy_rslt_t result;

    CY_LOGD(TAG, "IP event! %ld", event_id);

    if (event_id == IP_EVENT_PPP_GOT_IPV4) {
        cy_ppp_netif_dns_info_t dns_info;
        ip_event_got_ip_t *event_p = (ip_event_got_ip_t *)event_data_p;
        cy_ppp_netif_t *ppp_netif_p = event_p->ppp_netif_p;
        cy_time_t now = 0;

        cy_rtos_get_time(&now);
        CY_LOGI(TAG, "{%ld} PPP Connected: IPv4", now);

        CY_LOGI(TAG, "============================");
        CY_LOGI(TAG, "IP      : " IPV4_FORMAT, IPV4_TO_STR(&event_p->ip_info.ip));
        CY_LOGI(TAG, "Netmask : " IPV4_FORMAT, IPV4_TO_STR(&event_p->ip_info.netmask));
        CY_LOGI(TAG, "Gateway : " IPV4_FORMAT, IPV4_TO_STR(&event_p->ip_info.gw));

        cy_ppp_netif_get_dns_info(ppp_netif_p,
                                  0,
                                  &dns_info);
        CY_LOGI(TAG, "Name Server1: " IPV4_FORMAT, IPV4_TO_STR(&dns_info.ip.u_addr.ip4));

        cy_ppp_netif_get_dns_info(ppp_netif_p,
                                  1,
                                  &dns_info);
        CY_LOGI(TAG, "Name Server2: " IPV4_FORMAT, IPV4_TO_STR(&dns_info.ip.u_addr.ip4));
        CY_LOGI(TAG, "============================");

        s_is_ppp_connected = true;

        if (s_event_group != NULL) {
            result = cy_rtos_setbits_event(&s_event_group,
                                           CONNECT_BIT,
                                           false);
            (void)result;
        }

    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        cy_time_t now = 0;

        cy_rtos_get_time(&now);
        CY_LOGI(TAG, "{%ld} PPP Disconnected", now);
        s_is_ppp_connected = false;

        if (s_event_group != NULL) {
            result = cy_rtos_setbits_event(&s_event_group,
                                           ABORT_BIT,
                                           false);
        }

    } else if (event_id == IP_EVENT_PPP_GOT_IPV6) {
        ip_event_got_ip6_t *event_p = (ip_event_got_ip6_t *)event_data_p;
        cy_time_t now = 0;

        cy_rtos_get_time(&now);

        CY_LOGI(TAG, "{%ld} PPP Connected: IPv6", now);
        CY_LOGI(TAG, "============================");
        CY_LOGI(TAG, "IPv6 : " IPV6_FORMAT, IPV6_TO_STR(event_p->ip6_info.ip));
        CY_LOGI(TAG, "============================");

        s_is_ppp_connected = true;

        if (s_event_group != NULL) {
            result = cy_rtos_setbits_event(&s_event_group,
                                           CONNECT_BIT,
                                           false);
            (void)result;
        }
    }
}


/*-- Public Functions -------------------------------------------------*/

cy_rslt_t cy_pcm_init(  cy_pcm_config_t *config_p,
                        bool is_wcm_initialized)
{
    cy_rslt_t rslt;
    bool result;
    ReturnAssert(config_p != NULL, CY_RSLT_PCM_FAILED);

    result = cy_ppp_netif_init(is_wcm_initialized);
    ReturnAssert(result, CY_RSLT_PCM_FAILED);

    rslt = cy_rtos_init_event(&s_event_group);
    DEBUG_ASSERT(rslt == CY_RSLT_SUCCESS);
    ReturnAssert(s_event_group != NULL, CY_RSLT_PCM_FAILED);

    rslt = cy_rtos_init_mutex(&s_operationMutex);
    ReturnAssert(rslt == CY_RSLT_SUCCESS, CY_RSLT_PCM_FAILED);

    s_is_pcm_initialized = true;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_pcm_deinit(void)
{
    cy_rslt_t rslt;
    cy_ppp_netif_deinit();

    if (s_event_group != NULL) {
        rslt = cy_rtos_deinit_event(&s_event_group);
        DEBUG_ASSERT(rslt == CY_RSLT_SUCCESS);
        s_event_group = NULL;
    }

    rslt = cy_rtos_deinit_mutex(&s_operationMutex);
    DEBUG_ASSERT(rslt == CY_RSLT_SUCCESS);

    s_is_pcm_initialized = false;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_pcm_connect_modem( const cy_pcm_connect_params_t *connect_params_p,
                                cy_wcm_ip_address_t *ip_addr_p,
                                uint32_t timeout_msec)
{
    cy_rslt_t result;
    cy_modem_t *modem_p = NULL;
    cy_ppp_netif_t *ppp_netif_p = NULL;

    CY_LOGI(TAG, "%s [%d]", __FUNCTION__, __LINE__);

    ReturnAssert(s_is_pcm_initialized, CY_RSLT_PCM_FAILED);
    ReturnAssert(connect_params_p != NULL, CY_RSLT_PCM_FAILED);
    ReturnAssert(s_ppp_netif_p == NULL, CY_RSLT_PCM_FAILED);

    // acquire the mutex
    result = cy_rtos_get_mutex(&s_operationMutex, timeout_msec);

    if (result == CY_RTOS_TIMEOUT) {
        return CY_RSLT_PCM_TIMEOUT;

    } else if (result != CY_RSLT_SUCCESS) {
        return CY_RSLT_PCM_FAILED;
    }

    // abort if modem is already connected
    if (s_modem_p != NULL) {
        // release the mutex
        cy_rtos_set_mutex(&s_operationMutex);

        return CY_RSLT_PCM_MODEM_IN_USE;
    }

    do {
        uint32_t uxBits;

        const cy_ppp_netif_config_t ppp_netif_config = {
            .error_event_fn = ppp_client_error_event,
            .ip_event_fn = ppp_client_ip_event,
            .user_ip_lost_fn = connect_params_p->user_ip_lost_fn,
        };

        modem_p = cy_modem_new(connect_params_p->connect_ppp);
        if (modem_p == NULL) {
            CY_LOGE(TAG, "%s [%d]: cy_modem_new failed", __FUNCTION__, __LINE__);
            break;
        }

        if (!cy_modem_powerup(modem_p, connect_params_p->connect_ppp)) {
            CY_LOGE(TAG, "%s [%d]: cy_modem_powerup failed", __FUNCTION__, __LINE__);
            break;
        }

        if (connect_params_p->connect_ppp) {

            // reset the IP address
            ReturnAssert(ip_addr_p != NULL, CY_RSLT_PCM_FAILED);
            memset(ip_addr_p, 0, sizeof(*ip_addr_p));

            // reset the event bits
            if (s_event_group != NULL) {
                result = cy_rtos_clearbits_event(&s_event_group,
                                                 CONNECT_BIT | ABORT_BIT,
                                                 false);
            }

            ppp_netif_p = cy_ppp_netif_new(&ppp_netif_config);
            if (ppp_netif_p == NULL) {
                CY_LOGE(TAG, "%s [%d]: cy_ppp_netif_new failed", __FUNCTION__, __LINE__);
                break;
            }

            cy_netif_ppp_set_auth(ppp_netif_p,
                                  connect_params_p->credentials.security,
                                  (const char*)connect_params_p->credentials.username,
                                  (const char*)connect_params_p->credentials.password);

            /* attach the modem to the network interface (and start ppp) */
            if (!cy_modem_netif_start(modem_p,
                                      ppp_netif_p,
                                      (const char*)connect_params_p->apn)) {
                CY_LOGE(TAG, "%s [%d]: cy_modem_netif_start failed",
                        __FUNCTION__, __LINE__);
                break;
            }

            /* Wait for IP address */
            CY_LOGD(TAG, "%s [%d]: Wait %d ms for CONNECT",
                    __FUNCTION__, __LINE__, WAIT_INTERVAL_FOR_PPP_CONNECT);

            uxBits = CONNECT_BIT | ABORT_BIT;
            result = cy_rtos_waitbits_event(   &s_event_group,
                                               &uxBits,
                                               true, //bool clear,
                                               false, //bool all,
                                               WAIT_INTERVAL_FOR_PPP_CONNECT);


            (void)result;

            if ((uxBits & CONNECT_BIT) == 0) {
                CY_LOGE(TAG, "%s [%d]: timeout while waiting for CONNECT",
                        __FUNCTION__, __LINE__);

                /* tell LwIP to abort PPP */
                if (!cy_modem_netif_stop(modem_p, ppp_netif_p)) {
                    CY_LOGE(TAG, "%s [%d]: cy_modem_netif_stop failed",
                            __FUNCTION__, __LINE__);
                }

                /* Wait for ABORT */
                CY_LOGD(TAG, "%s [%d]: Wait %d ms for PPP to abort",
                        __FUNCTION__, __LINE__, WAIT_INTERVAL_FOR_PPP_ABORT);

                uxBits = ABORT_BIT;
                result = cy_rtos_waitbits_event(   &s_event_group,
                                                   &uxBits,
                                                   true, //bool clear,
                                                   true, //bool all,
                                                   WAIT_INTERVAL_FOR_PPP_ABORT);
                (void)result;

                if ((uxBits & ABORT_BIT) == 0) {
                    CY_LOGE(TAG, "%s [%d]: timeout while waiting PPP to abort",
                            __FUNCTION__, __LINE__);

                    /* try increasing WAIT_INTERVAL_FOR_PPP_ABORT */
                    DEBUG_ASSERT(0);
                }

                CY_LOGD(TAG, "%s [%d]: Received ABORT", __FUNCTION__, __LINE__);
                break;
            }

            CY_LOGD(TAG, "%s [%d]: Received CONNECT", __FUNCTION__, __LINE__);

            // sometimes an ipv6 event, is followed by a ipv4 event
            // wait a while for both events to be received, then
            // read the ip address
            cy_rtos_delay_milliseconds(WAIT_INTERVAL_FOR_IP_ADDRESS);

            if (ppp_netif_p->lwip_netif_p->ip_addr.u_addr.ip4.addr != 0) {
                ip_addr_p->version = CY_WCM_IP_VER_V4;
                ip_addr_p->ip.v4 = ppp_netif_p->lwip_netif_p->ip_addr.u_addr.ip4.addr;

            } else {
                ip_addr_p->version = CY_WCM_IP_VER_V6;
                memcpy( ip_addr_p->ip.v6,
                        ppp_netif_p->lwip_netif_p->ip_addr.u_addr.ip6.addr,
                        sizeof(ip_addr_p->ip.v6));
            }

            s_ppp_netif_p = ppp_netif_p;
        }

        s_modem_p = modem_p;

        // release the mutex
        cy_rtos_set_mutex(&s_operationMutex);

        return CY_RSLT_SUCCESS;

    } while (false);

    cy_modem_delete(modem_p, true);
    cy_ppp_netif_delete(ppp_netif_p);

    // release the mutex
    cy_rtos_set_mutex(&s_operationMutex);

    return CY_RSLT_PCM_FAILED;
}

cy_rslt_t cy_pcm_disconnect_modem(uint32_t timeout_msec, bool power_off_modem)
{
    cy_rslt_t result;

    ReturnAssert(s_is_pcm_initialized, CY_RSLT_PCM_FAILED);
    ReturnAssert(s_modem_p != NULL, CY_RSLT_PCM_FAILED);

    result = cy_rtos_get_mutex(&s_operationMutex, timeout_msec);

    if (result == CY_RTOS_TIMEOUT) {
        return CY_RSLT_PCM_TIMEOUT;

    } else if (result != CY_RSLT_SUCCESS) {
        return CY_RSLT_PCM_FAILED;
    }

    result = CY_RSLT_PCM_FAILED;

    do {
        s_is_ppp_connected = false;

        if (s_ppp_netif_p != NULL) {
            /* Exit PPP mode */
            if (!cy_modem_netif_stop(s_modem_p, s_ppp_netif_p)) {
                CY_LOGE(TAG, "%s [%d]: modem_netif_stop failed", __FUNCTION__, __LINE__);
                break;
            }
        }

        if (power_off_modem) {
            if (!cy_modem_powerdown(s_modem_p)) {
                CY_LOGE(TAG, "%s [%d]: modem_powerdown failed", __FUNCTION__, __LINE__);
                break;
            }
        }

        result = CY_RSLT_SUCCESS;

    } while (false);

    cy_modem_delete(s_modem_p, power_off_modem);
    s_modem_p = NULL;

    cy_ppp_netif_delete(s_ppp_netif_p);
    s_ppp_netif_p = NULL;

    cy_rtos_set_mutex(&s_operationMutex);
    return result;
}

bool cy_pcm_is_initialized(void)
{
    return s_is_pcm_initialized;
}

bool cy_pcm_is_ppp_connected(void)
{
    return s_is_ppp_connected;
}


cy_rslt_t cy_pcm_set_default_connectivity(connectivity_t type)
{
    cy_rslt_t result;

    ReturnAssert(s_is_pcm_initialized, CY_RSLT_PCM_FAILED);

    result = cy_rtos_get_mutex(&s_operationMutex, CY_RTOS_NEVER_TIMEOUT);

    if (result != CY_RSLT_SUCCESS) {
        return CY_RSLT_PCM_FAILED;
    }

    result = CY_RSLT_PCM_FAILED;

    if (type == WIFI_STA_CONNECTIVITY) {
        struct netif *wifi_net_interface = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);

        if (wifi_net_interface != NULL) {
            netifapi_netif_set_default(wifi_net_interface);

            s_default_connectivity = WIFI_STA_CONNECTIVITY;
            result = CY_RSLT_SUCCESS;
        }

    } else if (type == CELLULAR_CONNECTIVITY) {
        if ((s_ppp_netif_p != NULL) &&
                (s_ppp_netif_p->lwip_netif_p != NULL)) {

            netifapi_netif_set_default(s_ppp_netif_p->lwip_netif_p);

            s_default_connectivity = CELLULAR_CONNECTIVITY;
            result = CY_RSLT_SUCCESS;
        }
    }

    cy_rtos_set_mutex(&s_operationMutex);
    return result;
}


connectivity_t cy_pcm_get_default_connectivity(void)
{
    return s_default_connectivity;
}

cy_rslt_t cy_pcm_update_gps_location(void)
{
    cy_rslt_t result;
    ReturnAssert(s_is_pcm_initialized, CY_RSLT_PCM_FAILED);

    result = cy_rtos_get_mutex(&s_operationMutex, CY_RTOS_NEVER_TIMEOUT);

    if (result != CY_RSLT_SUCCESS) {
        return CY_RSLT_PCM_FAILED;
    }

    if (s_modem_p != NULL) {
        if (!cy_modem_update_gps_location(s_modem_p)) {
            result = CY_RSLT_PCM_FAILED;
        }
    }

    cy_rtos_set_mutex(&s_operationMutex);
    return result;
}


cy_rslt_t cy_pcm_get_modem_mode(cy_modem_mode_t *mode_p)
{
    ReturnAssert(mode_p != NULL, CY_RSLT_MW_BADARG);

    if (s_modem_p == NULL) {
        return CY_RSLT_PCM_MODEM_IS_NULL;
    }

    *mode_p = s_modem_p->mode;
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_pcm_change_modem_mode(cy_modem_mode_t new_mode)
{
    cy_rslt_t result;
    ReturnAssert(s_is_pcm_initialized, CY_RSLT_PCM_FAILED);

    result = cy_rtos_get_mutex(&s_operationMutex, CY_RTOS_NEVER_TIMEOUT);

    if (result == CY_RSLT_SUCCESS) {
        if (s_modem_p != NULL) {
            if (!cy_modem_change_mode(s_modem_p, new_mode)) {
                result = CY_RSLT_PCM_FAILED;
            }

        } else {
            result = CY_RSLT_PCM_MODEM_IS_NULL;
        }

        cy_rtos_set_mutex(&s_operationMutex);
    }
    return result;
}
