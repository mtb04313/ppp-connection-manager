/******************************************************************************
* File Name:   cy_pcm.h
*
* Description: This file is the public interface of cy_pcm.c
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

#ifndef SOURCE_CY_PCM_H_
#define SOURCE_CY_PCM_H_

#include <stdbool.h>
#include "cy_result.h"
#include "cy_result_mw.h"
#include "cy_wcm.h"
#include "cy_pcm_user.h"
#include "cy_modem_mode.h"

#ifdef __cplusplus
extern "C" {
#endif


/*-- Public Definitions -------------------------------------------------*/

#define REAL_PPP_CONNECTION        1   // 1:enable, 0:disable (for testing only)

#define CY_PCM_MAX_PASSWORD_LEN   64
#define CY_PCM_MAX_USERNAME_LEN   253
#define CY_PCM_MAX_APN_LEN        63


#define PCM_CONNECT_MODEM_TIMEOUT_MSEC       1000


// use generic middleware error codes
#define CY_RSLT_PCM_FAILED              CY_RSLT_MW_ERROR
#define CY_RSLT_PCM_TIMEOUT             CY_RSLT_MW_TIMEOUT
#define CY_RSLT_PCM_BADARG              CY_RSLT_MW_BADARG
#define CY_RSLT_PCM_MODEM_IS_NULL       CY_RSLT_MW_UNSUPPORTED
#define CY_RSLT_PCM_MODEM_IN_USE        CY_RSLT_MW_PENDNG

typedef uint8_t cy_pcm_username_t[CY_PCM_MAX_USERNAME_LEN + 1];  /**< User Name in null-terminated string format. */
typedef uint8_t cy_pcm_password_t[CY_PCM_MAX_PASSWORD_LEN + 1];  /**< Password in null-terminated string format. */
typedef uint8_t cy_pcm_security_t;
typedef uint8_t cy_pcm_apn_t[CY_PCM_MAX_APN_LEN + 1]; /**< Access Point Name in null-terminated string format. */

typedef enum {
    NO_CONNECTIVITY,
    CELLULAR_CONNECTIVITY,
    WIFI_STA_CONNECTIVITY,
} connectivity_t;

typedef struct {
    connectivity_t default_type;
    cy_wcm_interface_t wifi_interface_type;
} cy_pcm_config_t;

typedef struct {
    cy_pcm_username_t username;  /**< Username of the PPP network to join; should be a null-terminated string. */
    cy_pcm_password_t password;  /**< Password needed to join the PPP network; should be a null-terminated string. */
    cy_pcm_security_t security;  /**< PPP security protocol. */
} cy_pcm_credentials_t;

typedef struct {
    cy_pcm_credentials_t credentials;   /**< PPP network credentials. */
    cy_pcm_apn_t apn;                   /**< Access Point Name. */
    on_user_ip_lost_fn user_ip_lost_fn; /**< Callback when User IP address is lost. */
    bool connect_ppp;                   /**< Whether to connect PPP. If false, only connect the modem */
} cy_pcm_connect_params_t;


/*-- Public Functions -------------------------------------------------*/

cy_rslt_t cy_pcm_init(  cy_pcm_config_t *config_p,
                        bool is_wcm_initialized);

cy_rslt_t cy_pcm_deinit(void);

cy_rslt_t cy_pcm_connect_modem( const cy_pcm_connect_params_t *connect_params_p,
                                cy_wcm_ip_address_t *ip_addr_p,
                                uint32_t timeout_msec);

cy_rslt_t cy_pcm_disconnect_modem(uint32_t timeout_msec);

cy_rslt_t cy_pcm_set_default_connectivity(connectivity_t type);

connectivity_t cy_pcm_get_default_connectivity(void);

bool cy_pcm_is_initialized(void);

bool cy_pcm_is_ppp_connected(void);

cy_rslt_t cy_pcm_update_gps_location(void);

cy_rslt_t cy_pcm_get_modem_mode(cy_modem_mode_t *mode_p);

cy_rslt_t cy_pcm_change_modem_mode(cy_modem_mode_t new_mode);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CY_PCM_H_ */
