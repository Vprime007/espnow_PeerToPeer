#ifndef _ESPNOW_PEER_TO_PEER_H
#define _ESPNOW_PEER_TO_PEER_H

#include <stdint.h>
#include "esp_now.h"
#include "esp_err.h"

/******************************************************************************
*   Public Definitions
*******************************************************************************/
#define ESPNOW_MAX_PAYLOAD_LEN_BYTE             (1466)//Bytes

/******************************************************************************
*   Public Data Types
*******************************************************************************/
typedef enum ESPNOW_Status_e{
    ESPNOW_STATUS_NOT_PAIRED,
    ESPNOW_STATUS_PAIRED,
    ESPNOW_STATUS_PAIRING,

    ESPNOW_STATUS_INVALID,
}ESPNOW_Status_t;

/******************************************************************************
*   Public Macros
*******************************************************************************/
typedef void(*ESPNOW_MsgRecv_Callback_t)(const uint8_t *pMac_source,
                                         const uint8_t *pData,
                                         const uint16_t len);

typedef void(*ESPNOW_Pairing_Callback_t)(const uint8_t *pMac_peer,
                                         const ESPNOW_Status_t status);

/******************************************************************************
*   Public Variables
*******************************************************************************/


/******************************************************************************
*   Error Check
*******************************************************************************/


/******************************************************************************
*   Public Functions
*******************************************************************************/
/***************************************************************************//*!
*  \brief Init ESPNOW driver.
*
*   Initialize the ESPNOW driver and optionnaly register the data
*   reception and pairing callbacks. 
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]  task_priority           ESPNOW task priority
*   \param[in]  rf_channel              RF channel to use.
*   \param[in]  recv_callback           Data reception callback.
*   \param[in]  pairing_callback        Pairing callback.
*
*   \return     Operation status
*
*******************************************************************************/
esp_err_t ESPNOW_InitDriver(uint8_t task_priority,
                            uint8_t rf_channel,
                            ESPNOW_MsgRecv_Callback_t recv_callback,
                            ESPNOW_Pairing_Callback_t pairing_callback);

/***************************************************************************//*!
*  \brief Start pairing.
*
*   Start the pairing process. A pairing request will be sent every 100ms for the
*   specified number of attemps.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]  nb_attemps              Number of pairing attemps.
*
*   \return     Operation status
*
*******************************************************************************/
esp_err_t ESPNOW_StartPairing(uint32_t nb_attemps);

/***************************************************************************//*!
*  \brief Flush ESPNOW peer.
*
*   Remove the ESPNOW peer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \return     Operation status
*
*******************************************************************************/
esp_err_t ESPNOW_FlushPeer(void);

/***************************************************************************//*!
*  \brief Get Pair status.
*
*   Return the current pair status.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[out] pStatus         Pointer to store the pair status.
*
*   \return     Operation status
*
*******************************************************************************/
esp_err_t ESPNOW_GetPairingStatus(ESPNOW_Status_t *pStatus);

/***************************************************************************//*!
*  \brief Send data via ESPNOW.
*
*   Send data to the paired peer via ESPNOW. The device sending data must be
*   paired before calling this function.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]  pData                   Tx data buffer.
*   \param[in]  len                     Data buffer len (in Byte).
*
*   \return     Operation status
*
*******************************************************************************/
esp_err_t ESPNOW_SendData(const uint8_t *pData, uint16_t len);

#endif//_ESPNOW_PEER_TO_PEER_H