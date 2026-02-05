/******************************************************************************
*   Includes
*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "espnow_PeerToPeer.h"

/******************************************************************************
*   Private Definitions
*******************************************************************************/
#define ESPNOW_PMK                      "pmk1234567890123"
#define ESPNOW_NVS_NS                   "espnow"
#define ESPNOW_NVS_KEY                  "peer_mac"

#define ESPNOW_TX_QUEUE_SIZE            (8)
#define ESPNOW_RX_QUEUE_SIZE            (8)

#define ESPNOW_CMD_TYPE_SIZE            (sizeof(ESPNOW_Cmd_Type_t))//In bytes
#define ESPNOW_CRC16_SIZE               (2)//In bytes

#define ESPNOW_PAIR_REQ_PERIOD_MS       (200)//ms

/******************************************************************************
*   Private Macros
*******************************************************************************/


/******************************************************************************
*   Private Data Types
*******************************************************************************/
typedef enum ESPNOW_Cmd_Type_e{
    ESPNOW_CMD_PAIR_REQ,
    ESPNOW_CMD_PAIR_RESP,
    ESPNOW_CMD_FLUSH_PEER,
    ESPNOW_CMD_DATA,

    ESPNOW_CMD_INVALID,
}ESPNOW_Cmd_Type_t;

typedef struct ESPNOW_Packet_s{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *pMsg;
    uint16_t len;
}ESPNOW_Packet_t;

/******************************************************************************
*   Private Functions Declaration
*******************************************************************************/
static void save_peer_infos(const uint8_t *pMac_addr);
static bool load_peer_infos(uint8_t *pMac_addr);
static void erase_peer_infos(void);

static bool postPairingRequest(void);
static bool postFlushPeer(void);

static uint16_t crc16(uint8_t *pBuffer, uint16_t len);

static void espnow_recv_callback(const esp_now_recv_info_t *pInfo,
                                 const uint8_t *pData,
                                 int len);

static void espnow_send_callback(const uint8_t *pMac_dest,
                                 esp_now_send_status_t status);

static void pairingTimerCallback(TimerHandle_t xTimer);

static void tEspnowTask(void *pvParameters);

/******************************************************************************
*   Public Variables
*******************************************************************************/


/******************************************************************************
*   Private Variables
*******************************************************************************/
static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = 
                                    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0};
static ESPNOW_Status_t current_status = ESPNOW_STATUS_NOT_PAIRED;

static ESPNOW_MsgRecv_Callback_t app_msg_recv_callback = NULL;
static ESPNOW_Pairing_Callback_t app_pairing_callback = NULL;

static TaskHandle_t espnow_task_handle = NULL;
static QueueHandle_t espnow_tx_queue_handle = NULL;
static QueueHandle_t espnow_rx_queue_handle = NULL;
static TimerHandle_t espnow_timer_handle = NULL;

static uint32_t pairing_attemps = 0;

/******************************************************************************
*   Error Check
*******************************************************************************/


/******************************************************************************
*   Private Functions Definitions
*******************************************************************************/
/***************************************************************************//*!
*  \brief Save peer in NVS.
*
*   Save peer MAC address in NVS.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]  pMac_addr           Peer MAC address (6 bytes).
*
*******************************************************************************/
static void save_peer_infos(const uint8_t *pMac_addr){

    nvs_handle_t nvs;
    if(ESP_OK == nvs_open(ESPNOW_NVS_NS, NVS_READWRITE, &nvs)){

        nvs_set_blob(nvs, ESPNOW_NVS_KEY, pMac_addr, ESP_NOW_ETH_ALEN);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

/***************************************************************************//*!
*  \brief Load peer
*
*   Retrieve previously paired peer MAC address form NVS. 
*   Return true if peer address was found / False otherwise.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[out]  pMac_addr           Peer MAC address (6 bytes).
*
*   \return      (True -> Peer found / False no Peer)
*
*******************************************************************************/
static bool load_peer_infos(uint8_t *pMac_addr){

    nvs_handle_t nvs;
    size_t size = ESP_NOW_ETH_ALEN;

    if(ESP_OK == nvs_open(ESPNOW_NVS_NS, NVS_READONLY, &nvs)){

        if(ESP_OK == nvs_get_blob(nvs, ESPNOW_NVS_KEY, pMac_addr, &size)){

            nvs_close(nvs);
            return true;
        }

        nvs_close(nvs);
    }

    return false;
}

/***************************************************************************//*!
*  \brief Erase peer infos
*
*   Erase peer infos stored in NVS.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \return     None.
*
*******************************************************************************/
static void erase_peer_infos(void){

    nvs_handle_t nvs;
    if(ESP_OK == nvs_open(ESPNOW_NVS_NS, NVS_READWRITE, &nvs)){

        nvs_erase_key(nvs, ESPNOW_NVS_KEY);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

/***************************************************************************//*!
*  \brief Post a pairing request.
*
*   Post a pairing request to be sent via espnow.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \return     Operation status.       (true -> OK, false -> not OK)
*
*******************************************************************************/
static bool postPairingRequest(void){

    ESPNOW_Cmd_Type_t cmd_type = ESPNOW_CMD_PAIR_REQ;
    uint16_t msg_crc = crc16((uint8_t*)&cmd_type, 1);

    //Allocate buffer for tx msg
    uint8_t *pTx_msg = malloc((ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE)*sizeof(uint8_t));
    if(pTx_msg == NULL)     return false;

    //Fill tx msg
    memcpy(pTx_msg, &cmd_type, sizeof(cmd_type));
    memcpy(pTx_msg+sizeof(cmd_type), &msg_crc, sizeof(msg_crc));

    ESPNOW_Packet_t packet_to_send = {0};
    memcpy(packet_to_send.mac_addr, broadcast_mac, sizeof(broadcast_mac));
    packet_to_send.len = ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE;
    packet_to_send.pMsg = pTx_msg;

    //Post tx packet to the queue
    if(pdTRUE != xQueueSend(espnow_tx_queue_handle, &packet_to_send, 0)){
        //Free allocated ressources
        free(pTx_msg);
        pTx_msg = NULL;

        return false;
    }

    return true;
}

/***************************************************************************//*!
*  \brief Post a flush peer.
*
*   Post a flush peer to be sent via espnow.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \return     Operation status.       (true -> OK, false -> not OK)
*
*******************************************************************************/
static bool postFlushPeer(void){

    ESPNOW_Cmd_Type_t cmd_type = ESPNOW_CMD_FLUSH_PEER;
    uint16_t msg_crc = crc16((uint8_t*)&cmd_type, 1);

    //Allocate buffer for tx msg
    uint8_t *pTx_msg = malloc((ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE)*sizeof(uint8_t));
    if(pTx_msg == NULL)     return false;

    //Fill tx msg
    memcpy(pTx_msg, &cmd_type, sizeof(cmd_type));
    memcpy(pTx_msg+sizeof(cmd_type), &msg_crc, sizeof(msg_crc));

    ESPNOW_Packet_t packet_to_send = {0};
    memcpy(packet_to_send.mac_addr, peer_mac, sizeof(peer_mac));
    packet_to_send.len = ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE;
    packet_to_send.pMsg = pTx_msg;

    //Post tx packet to queue
    if(pdTRUE != xQueueSend(espnow_tx_queue_handle, &packet_to_send, 0)){
        //Free allocated ressources
        free(pTx_msg);
        pTx_msg = NULL;

        return false;
    }

    return true;
}

/***************************************************************************//*!
*  \brief CRC 16 bits.
*
*   Calculates the DRD-16-ANSI checksum for a data buffer.
*   Polynomial: 0x8005 (or 0xA001 reflected)
*   Initial value: 0xFFFF
*   Final XOR: 0x0000
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]  pBuffer             Data buffer.
*   \param[in]  len                 Data buffer len (in Bytes).
*
*   \return     crc-16 value.
*
*******************************************************************************/
static uint16_t crc16(uint8_t *pBuffer, uint16_t len){

    uint16_t crc = 0xFFFF;

    for(uint16_t i=0; i<len; i++){

        crc ^= pBuffer[i];
        for(uint8_t j=0; j<8; j++){

            if(crc & 0x0001){
                //Right shift is used for reflected algorithms
                crc = (crc >> 1) ^ 0xA001;
            }
            else{
                crc >>= 1;
            }
        }
    }

    return crc;//Final XOR is 0x0000
}

/***************************************************************************//*!
*  \brief ESPNOW recv callback
*
*   Mandatory espnow recv callback.
*
*******************************************************************************/
static void espnow_recv_callback(const esp_now_recv_info_t *pInfo,
                                 const uint8_t *pData,
                                 int len){

    //Validate buffer crc
    uint16_t calc_crc = crc16((uint8_t*)pData, len-ESPNOW_CRC16_SIZE);
    uint16_t msg_crc = (pData[len-ESPNOW_CRC16_SIZE] << 8) + (pData[len-1]);
    if(calc_crc != msg_crc){
        //Invalid msg... do nothing...
    }

    ESPNOW_Cmd_Type_t cmd_type = (ESPNOW_Cmd_Type_t)(pData[0]);

    switch(cmd_type){
        case ESPNOW_CMD_PAIR_REQ:
        {   
            //check if if we are acivel pairing
            if(current_status != ESPNOW_STATUS_PAIRING){
                //Already paired... do nothing...
                return;
            }

            //Add initiator as peer
            esp_now_peer_info_t peer = {0};
            memcpy(peer.peer_addr, pInfo->src_addr, ESP_NOW_ETH_ALEN);
            peer.channel = 0;
            peer.ifidx = ESP_IF_WIFI_STA;
            peer.encrypt = false;

            //Check if peer is already registered
            if(!esp_now_is_peer_exist(peer.peer_addr)){
                if(ESP_OK != esp_now_add_peer(&peer)){
                    
                    //Failed to add peer...
                    return;
                }

                memcpy(peer_mac, peer.peer_addr, ESP_NOW_ETH_ALEN);
                current_status = ESPNOW_STATUS_PAIRED;
                save_peer_infos(peer.peer_addr);

                //Send back a pairing response
                uint8_t cmd_type = ESPNOW_CMD_PAIR_RESP;
                uint16_t resp_crc = crc16(&cmd_type, 1);
                uint8_t *pResp = malloc((ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE)*sizeof(uint8_t));
                memcpy(pResp, &cmd_type, sizeof(cmd_type));
                memcpy(pResp+sizeof(cmd_type), &resp_crc, sizeof(resp_crc));

                esp_now_send(peer.peer_addr, pResp, ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE);

                if(app_pairing_callback != NULL)    app_pairing_callback(peer.peer_addr, ESPNOW_STATUS_PAIRED);
            }
        }
        break;

        case ESPNOW_CMD_PAIR_RESP:
        {   
            //Check if we are currently paired
            if((current_status != ESPNOW_STATUS_NOT_PAIRED) && (current_status != ESPNOW_STATUS_PAIRING)){
                //Already paired... do nothing...
                return;
            }

            xTimerStop(espnow_timer_handle, 10/portTICK_PERIOD_MS);
            pairing_attemps = 0;//Reset pairing attemps


            //Add responder as peer
            esp_now_peer_info_t peer = {0};
            memcpy(peer.peer_addr, pInfo->src_addr, ESP_NOW_ETH_ALEN);
            peer.channel = 0;
            peer.ifidx = ESP_IF_WIFI_STA;
            peer.encrypt = false;

            //Check if peer is already registered
            if(!esp_now_is_peer_exist(peer.peer_addr)){
                if(ESP_OK != esp_now_add_peer(&peer)){
                    
                    //Failed to add peer...
                    return;
                }

                memcpy(peer_mac, peer.peer_addr, ESP_NOW_ETH_ALEN);
                current_status = ESPNOW_STATUS_PAIRED;
                save_peer_infos(peer.peer_addr);

                if(app_pairing_callback != NULL)    app_pairing_callback(peer.peer_addr, ESPNOW_STATUS_PAIRED);
            }
        }
        break;

        case ESPNOW_CMD_FLUSH_PEER:
        {   
            //Check if we are currently paired
            if(current_status != ESPNOW_STATUS_PAIRED){
                //Not currently paired... do nothing...
                return;
            }

            //Check if the message come from our peer
            for(uint8_t i=0; i<ESP_NOW_ETH_ALEN; i++){
                if(peer_mac[i] != pInfo->src_addr[i]){
                    //Invalid mac source
                    return;
                }
            }

            //delete peer
            esp_now_del_peer(peer_mac);
            memset(peer_mac, 0, ESP_NOW_ETH_ALEN);
            current_status = ESPNOW_STATUS_NOT_PAIRED;
            
            //Remove peer infos in NVS
            erase_peer_infos();

            //App callback
            if(app_pairing_callback != NULL)    app_pairing_callback(pInfo->src_addr, ESPNOW_STATUS_NOT_PAIRED);
        }
        break;

        case ESPNOW_CMD_DATA:
        {
            if(current_status != ESPNOW_STATUS_PAIRED){
                //We are not currently paired... reject all data packet
                return;
            }

            //Check if the data is comming from our peer
            for(uint8_t i=0; i<ESP_NOW_ETH_ALEN; i++){
                if(peer_mac[i] != pInfo->src_addr[i]){   
                    //Invalid mac source
                    return;
                } 
            }

            //Allocate buffer for incoming msg
            uint8_t *pRx_msg = malloc((len-ESPNOW_CMD_TYPE_SIZE-ESPNOW_CRC16_SIZE)*sizeof(uint8_t));
            if(pRx_msg == NULL)     return;

            memcpy(pRx_msg, pData+ESPNOW_CMD_TYPE_SIZE, len-ESPNOW_CMD_TYPE_SIZE-ESPNOW_CRC16_SIZE);

            ESPNOW_Packet_t recv_packet = {0};
            memcpy(recv_packet.mac_addr, peer_mac, sizeof(peer_mac));
            recv_packet.len = len-ESPNOW_CMD_TYPE_SIZE-ESPNOW_CRC16_SIZE;
            recv_packet.pMsg = pRx_msg;

            if(pdTRUE != xQueueSend(espnow_rx_queue_handle, &recv_packet, 0)){
                //Free allocated ressources
                free(pRx_msg);
                pRx_msg = NULL;
                return;
            }
        }
        break;

        case ESPNOW_CMD_INVALID:
        default:
        {
            //Invalid cmd... do nothing...S
        }
        break;
    }

}

/***************************************************************************//*!
*  \brief ESPNOW send callback
*
*   Optional espnow recv callback.
*
*******************************************************************************/
static void espnow_send_callback(const uint8_t *pMac_dest,
                                 esp_now_send_status_t status){

    //Do nothing for the moment...
}

static void pairingTimerCallback(TimerHandle_t xTimer){

    if(pairing_attemps != 0)    pairing_attemps--;

    if(pairing_attemps == 0){

        if(app_pairing_callback != NULL)    app_pairing_callback(peer_mac, ESPNOW_STATUS_NOT_PAIRED);
    }
    else{
        //Send pairing request
        postPairingRequest();

        //Re-schedule the pairing timeout
        xTimerStart(espnow_timer_handle, 10/portTICK_PERIOD_MS);
    }
}

/***************************************************************************//*!
*  \brief ESPNOW task.
*
*   ESPNOW task is used to process queued TX messages and RX responses.
*
*   \param[in]  pvParameters    User parameters.
*
*******************************************************************************/
static void tEspnowTask(void *pvParameters){

    ESPNOW_Packet_t rx_packet = {0};
    ESPNOW_Packet_t tx_packet = {0};

    for(;;){

        //Retrieve all queued rx packets
        if(pdTRUE == xQueueReceive(espnow_rx_queue_handle, &rx_packet, 100/portTICK_PERIOD_MS)){

            //Pass the RX packe to the application
            if(app_msg_recv_callback != NULL){
                app_msg_recv_callback(rx_packet.mac_addr, rx_packet.pMsg, rx_packet.len);
            }

            //Free allocated resources
            free(rx_packet.pMsg);
            rx_packet.pMsg = NULL;
        }

        //Send all queued tx packets
        if(pdTRUE == xQueueReceive(espnow_tx_queue_handle, &tx_packet, 100/portTICK_PERIOD_MS)){

            //Send TX packet to peer via espnow
            esp_now_send(tx_packet.mac_addr, tx_packet.pMsg, tx_packet.len);

            //Free allocated resources
            free(tx_packet.pMsg);
            tx_packet.pMsg = NULL;
        }
    }
    vTaskDelete(NULL);
}

/******************************************************************************
*   Public Functions Definitions
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
                            ESPNOW_Pairing_Callback_t pairing_callback){

    //Register application callbacks
    if(recv_callback != NULL)       app_msg_recv_callback = recv_callback;
    if(pairing_callback != NULL)    app_pairing_callback = pairing_callback;

    //Init espnow
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESPNOW_PMK));
    esp_now_register_recv_cb(espnow_recv_callback);
    esp_now_register_send_cb(espnow_send_callback);

    //Add broadcast peer
    esp_now_peer_info_t broadcast_peer = {0};
    memcpy(broadcast_peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    broadcast_peer.channel = 0;
    broadcast_peer.ifidx = ESP_IF_WIFI_STA;
    broadcast_peer.encrypt = false;

    if(!esp_now_is_peer_exist(broadcast_mac)){
        ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));
    }

    //Try to restore previously paired peer
    if(load_peer_infos(peer_mac)){
        
        esp_now_peer_info_t peer_info = {0};
        memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
        peer_info.channel = 0;
        peer_info.ifidx = ESP_IF_WIFI_STA;
        peer_info.encrypt = false;

        if(!esp_now_is_peer_exist(peer_mac)){
            esp_now_add_peer(&peer_info);
        }

        current_status = ESPNOW_STATUS_PAIRED;

        if(app_pairing_callback != NULL)    app_pairing_callback(peer_mac, ESPNOW_STATUS_PAIRED);
    }

    //Create espnow tx queue
    espnow_tx_queue_handle = xQueueCreate(ESPNOW_RX_QUEUE_SIZE, sizeof(ESPNOW_Packet_t));
    if(espnow_tx_queue_handle == NULL){
        //Failed to create espnow tx queue
        return ESP_FAIL;
    }

    //Create espnow rx queue
    espnow_rx_queue_handle = xQueueCreate(ESPNOW_RX_QUEUE_SIZE, sizeof(ESPNOW_Packet_t));
    if(espnow_rx_queue_handle == NULL){
        //Failed to create espnow rx queue
        return ESP_FAIL;
    }

    //Create timer
    espnow_timer_handle = xTimerCreate("espnow timer",
                                       ESPNOW_PAIR_REQ_PERIOD_MS,
                                       pdFALSE,
                                       (void*)0,
                                       pairingTimerCallback);

    if(espnow_timer_handle == NULL){
        //Failed to create espnow timer
        return ESP_FAIL;
    }

    //Create espnow task
    if(pdTRUE != xTaskCreate(tEspnowTask,
                             "Espnow Task",
                             2048,
                             NULL,
                             task_priority,
                             &espnow_task_handle)){

        //Failed to create the espnow task
        return ESP_FAIL;
    }

    return ESP_OK;
}

/***************************************************************************//*!
*  \brief Start pairing.
*
*   Start the pairing process. A pairing request will be sent at 50ms for the
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
esp_err_t ESPNOW_StartPairing(uint32_t nb_attemps){

    if((espnow_tx_queue_handle == NULL) || 
       (current_status != ESPNOW_STATUS_NOT_PAIRED)){

        return ESP_FAIL;
    }

    if(postPairingRequest()){
        
        current_status = ESPNOW_STATUS_PAIRING;//Update status to pairing

        pairing_attemps = nb_attemps;
        if(pairing_attemps > 0){
            //Start pairing timer
            xTimerStart(espnow_timer_handle, 10/portTICK_PERIOD_MS);
        }
    }
    else{
        return ESP_FAIL;
    }

    return ESP_OK;
}

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
esp_err_t ESPNOW_FlushPeer(void){

    //Check if currently paired
    if(current_status == ESPNOW_STATUS_PAIRED){

        //Send a flush cmd to peer
        if(!postFlushPeer()){
            return ESP_FAIL;
        }

        //Remove peer infos from nvs
        esp_now_del_peer(peer_mac);
        memset(peer_mac, 0, ESP_NOW_ETH_ALEN);
        current_status = ESPNOW_STATUS_NOT_PAIRED;

        nvs_handle_t nvs;
        if(ESP_OK == nvs_open(ESPNOW_NVS_NS, NVS_READWRITE, &nvs)){

            nvs_erase_key(nvs, ESPNOW_NVS_KEY);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
    }
    else{
        //Not currently paired.... Cannot flush peer...
        return ESP_FAIL;
    }

    return ESP_OK;
}

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
esp_err_t ESPNOW_GetPairingStatus(ESPNOW_Status_t *pStatus){

    if(pStatus == NULL)     return ESP_FAIL;

    *pStatus = current_status;

    return ESP_OK;
}

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
esp_err_t ESPNOW_SendData(const uint8_t *pData, uint16_t len){

    if((pData == NULL) || 
       (len > ESPNOW_MAX_PAYLOAD_LEN_BYTE) || 
       (espnow_tx_queue_handle == NULL)){

        return ESP_FAIL;
    }

    //Allocate buffer for tx msg
    uint8_t *pTx_msg = malloc((ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE+len)*sizeof(uint8_t));
    if(pTx_msg == NULL)     return ESP_ERR_NO_MEM;

    //Fill tx msg
    ESPNOW_Cmd_Type_t cmd_type = ESPNOW_CMD_DATA;
    uint16_t msg_crc = 0;
    memcpy(pTx_msg, &cmd_type, sizeof(cmd_type));
    memcpy(pTx_msg+sizeof(cmd_type), pData, len);

    msg_crc = crc16(pTx_msg, len+ESPNOW_CMD_TYPE_SIZE);
    memcpy(pTx_msg+sizeof(cmd_type)+len, &msg_crc, sizeof(msg_crc));
    
    ESPNOW_Packet_t packet_to_send = {0};
    memcpy(packet_to_send.mac_addr, peer_mac, sizeof(peer_mac));
    packet_to_send.len = len+ESPNOW_CMD_TYPE_SIZE+ESPNOW_CRC16_SIZE;
    packet_to_send.pMsg = pTx_msg;

    //Post tx packet to the queue
    if(pdTRUE != xQueueSend(espnow_tx_queue_handle, &packet_to_send, 0)){

        //Free allocated ressources
        free(pTx_msg);
        pTx_msg = NULL;

        return ESP_FAIL;
    }

    return ESP_OK;
}

/******************************************************************************
*   Interrupts
*******************************************************************************/

