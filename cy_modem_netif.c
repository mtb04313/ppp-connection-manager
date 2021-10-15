/******************************************************************************
* File Name:   cy_modem_netif.c
*
* Description: This file implements the glue code between modem and ppp_netif.
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
#include <sys/param.h>

#include "cy_modem_netif.h"
#include "cy_memtrack.h"
#include "cy_debug.h"
#include "cy_string.h"


/*-- Local Definitions -------------------------------------------------*/
#define MODEM_WRITE_IN_CHUNKS       0 /* 1:enable, 0:disable */


/*-- Local Functions -------------------------------------------------*/

#if MODEM_WRITE_IN_CHUNKS
static bool modem_write_in_chunks(void *modem_p,
                                  void *buffer,
                                  size_t size)
{
#define MAX_CHUNK_SIZE   128

    bool result = false;

    while (size > 0) {
        size_t chunk_size;

        if (size > MAX_CHUNK_SIZE) {
            chunk_size = MAX_CHUNK_SIZE;
        } else {
            chunk_size = size;
        }

        result = Modem_WriteCommand(((cy_modem_t*)modem_p)->handle,
                                    (const uint8_t*)buffer,
                                    chunk_size);
        if (result) {
            size -= chunk_size;
        } else {
            break;
        }
    }
    return result;

#undef MAX_CHUNK_SIZE
}
#endif


/* sends data from network stack to the modem */
static bool modem_transmit( void *modem_p,
                            void *buffer_p,
                            size_t size)
{
    //CY_LOGD(TAG, "%s [%d]: size=%u", __FUNCTION__, __LINE__, size);
    //print_bytes("data:", (const uint8_t *)buffer_p, size);

    ReturnAssert(buffer_p != NULL, false);

#if MODEM_WRITE_IN_CHUNKS
    return modem_write_in_chunks( modem_p,
                                  buffer_p,
                                  size);

#else
    return Modem_WriteCommand(((cy_modem_t*)modem_p)->handle,
                              (const uint8_t*)buffer_p,
                              size);
#endif
}


/* receives data from modem, intended for the network stack */
static bool modem_netif_receive_cb( void *buffer_p,
                                    size_t size,
                                    void *context)
{
    cy_modem_t *modem_p = (cy_modem_t*)context;

    ReturnAssert(modem_p != NULL, false);
    ReturnAssert(modem_p->mode == CY_MODEM_PPP_MODE, false);

    return cy_ppp_netif_receive(modem_p->receive_cb_ctx,
                                buffer_p,
                                size);
}


static void modem_netif_setup(cy_modem_t *modem_p,
                              cy_ppp_netif_t *ppp_netif_p)
{
    VoidAssert(modem_p != NULL);
    VoidAssert(ppp_netif_p != NULL);

    modem_p->read_cb_p->read_callback = modem_netif_receive_cb;
    modem_p->read_cb_p->read_callback_ctx = modem_p;
    modem_p->receive_cb_ctx = ppp_netif_p;

    ppp_netif_p->modem_handle = modem_p;
    ppp_netif_p->modem_transmit = modem_transmit;
}


/*-- Public Functions -------------------------------------------------*/

bool cy_modem_netif_start(cy_modem_t *modem_p,
                          cy_ppp_netif_t *ppp_netif_p,
                          const char *apn_p)
{
    modem_netif_setup(modem_p, ppp_netif_p);

    bool result = modem_start_ppp(modem_p,
                                  apn_p);

    if (result) {
        result = cy_ppp_netif_start(ppp_netif_p);
    }

    return true;
}

bool cy_modem_netif_stop( cy_modem_t *modem_p,
                          cy_ppp_netif_t *ppp_netif_p)
{
    bool result;

    result = cy_ppp_netif_stop(ppp_netif_p);
    ReturnAssert(result, false);

    result = modem_stop_ppp(modem_p);

    return result;
}
