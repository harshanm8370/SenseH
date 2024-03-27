
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "API_Flash_org.h"

//Macros
#define EXAMPLE_ESP_WIFI_SSID      ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       ESP_MAX_STA_CONN

//Function declarations
void wifi_start_access_point(void);
void API_TCP_Server(void);
void disconnect_wifi(void);
void wifi_restart(void);
void wifi_send_data(uint8_t* data, size_t length);
bool wait_for_ack(int socket);
uint32_t API_Wifi_Receive(uint8_t *data_buff);
void EVAL_REQ_VITAL_CMD(int *recived, char *buff);

