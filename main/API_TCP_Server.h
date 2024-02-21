
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "API_Flash_org.h"
#define EXAMPLE_ESP_WIFI_SSID      ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       ESP_MAX_STA_CONN

//void wifi_driver(void);
void wifi_start_access_point(void);
void API_TCP_Server(void);
void disconnect_wifi(void);
void wifi_controller_jump(VITAL_TYPE_t vital, uint16_t one_record_len);
void wifi_send_data(uint8_t* data, size_t length);
void setHoldSocketTask(bool hold);
void socket_close();
bool wait_for_ack(int socket);
bool wait_for_endack(int socket);
uint32_t API_Wifi_Receive(uint8_t *data_buff);
