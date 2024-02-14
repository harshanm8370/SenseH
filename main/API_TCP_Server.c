
#include <lwip/sockets.h>
#include <esp_log.h>
#include <string.h>
#include <errno.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "API_TCP_Server.h"
#include "API_Flash_org.h"
#include <stdbool.h>
#include "bluetooth.h"
int variouble;
#define FALSE 0
#define PORT_NUMBER 5002
static char tag[] = "socket_server";
static int END_Var;
int clientSock ;
extern int globalFlag;
int terminate;
extern int 	Empty_Record;
static int sock;
#define BUF_SIZE      600
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BUF_SIZE 600

enum {
	Wifi_BUF_EMPTY,
	Wifi_BUF_FULL,
	wifi_TX_RX_FAILED,
	wifi_TX_RX_SUCCESS,
};

typedef struct {
	uint8_t wifi_status;
	uint16_t wifi_len;
	uint8_t wifi_buf[BUF_SIZE];
} wifi_BUF_t;
static wifi_BUF_t wifi_buf;

uint32_t API_Wifi_Receive(uint8_t *data_buf)
{
	if (wifi_buf.wifi_status == Wifi_BUF_FULL) {
		memcpy(data_buf, wifi_buf.wifi_buf, wifi_buf.wifi_len);

		printf("copying the Received Bytes: ");
		for (size_t i = 0; i < wifi_buf.wifi_len; ++i) {
			printf("%02X ", data_buf[i]);
		}
		printf("\n");

		memset(wifi_buf.wifi_buf, 0, sizeof(wifi_buf.wifi_buf));
		wifi_buf.wifi_status = Wifi_BUF_EMPTY;

		return wifi_buf.wifi_len;
	} else {
		return Wifi_BUF_EMPTY;
	}
}



void disconnect_wifi()
{
	printf("\n %s",__func__);
	ESP_LOGW(tag, "End of socket_server_task");
	close(sock);
	esp_wifi_stop();
	esp_wifi_deinit();
	vTaskDelete(NULL);
}
void wifi_send_data(uint8_t* data, size_t length)
{
	if (clientSock != -1 && clientSock > 0 )
	{
		ESP_LOGI(tag, "Sending data...");

		ESP_LOGI(tag, "Raw data:");

		for (size_t i = 0; i < length; i++)
		{
			printf("%02X ", data[i]);
		}
		printf("\n");

		int bytes_sent = send(clientSock, (uint8_t*)data, length, 0);
		if (bytes_sent < 0)
		{
			ESP_LOGE(tag, "send failed: errno %d", errno);
		}
		else
		{
			ESP_LOGI(tag, "Sent %d bytes", bytes_sent);
			char endbuffer[10];
			memset(endbuffer, 0, sizeof(endbuffer));
			while(1)
			{
				int len = recv(clientSock, endbuffer, sizeof(endbuffer) - 1, 0);
				if (len > 0)
				{
					//printf("\n Received String (Debug): %s", endbuffer);
					if (strcmp(endbuffer, "END") == 0)
					{
						printf("\n Received String (Debug): %s", endbuffer);
						//	terminate = 1;
						break;
					}
					else if (strcmp(endbuffer, "ACK") == 0)
					{
						printf("\n Received String (Debug): %s", endbuffer);
						break;
					}
				}
			}
			printf("\n iam closing client socket");
		}
	}
	else
	{
		printf("\n clientSock == -1 \n");
	}
	printf("\n iam going out from send func \n");
}
bool wait_for_ack(int socket)
{

	memset(&wifi_buf, 0, sizeof(wifi_buf));
	// C0 10 03 C0 00 00

	int len = recv(clientSock, wifi_buf.wifi_buf, sizeof(wifi_buf.wifi_buf) - 1, 0);


	printf("\n ++++++++++++ Received String (Debug): ");
	for (size_t i = 0; i < len; i++)
	{
		printf("%02X ",  wifi_buf.wifi_buf[i]);
	}
	printf("\n");

	if (len == 0 && len < 0)
	{
		ESP_LOGI(tag, "Connection closed");
		return false;
	}
	else
	{
		BT_process_requests();
	    return (wifi_buf.wifi_buf[1] == 20);
		printf("\N one record sent");
	}

	return false;
}



void wifi_start_access_point() {
	wifi_config_t wifi_config = {
			.ap = {
					.ssid = "SALMAN_WIFI",
					.password = "MYPASSWORD",
					.channel = 1,
					.authmode = WIFI_AUTH_WPA2_PSK,
					.ssid_hidden = 0,
					.max_connection = 5,
					.beacon_interval = 100
			}
	};

	esp_netif_init();//network interface
	ESP_ERROR_CHECK(esp_event_loop_create_default());//default event loop that will be used to handle WiFi-related events
	esp_netif_t* wifiAP = esp_netif_create_default_wifi_ap();//This line creates a default WiFi access point interface
	esp_netif_ip_info_t ip_info;//ip info
	IP4_ADDR(&ip_info.ip, 192, 168, 1, 1);
	IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
	IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
	esp_netif_dhcps_stop(wifiAP);//This line stops the DHCP server associated with the access point network interface
	esp_netif_set_ip_info(wifiAP, &ip_info);//This sets the previously configured IP information (IP address, gateway, and netmask) for the access point network interface.
	esp_netif_dhcps_start(wifiAP);

	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&wifi_init_config);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_AP);
	esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
	esp_wifi_start();
}


void socket_server_task(void* pvParameters)
{
	struct sockaddr_in clientAddress;
	struct sockaddr_in serverAddress;
	//IPv4 AF_INET
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//socket system call
	if (sock < 0)
	{
		ESP_LOGE(tag, "socket: %d %s", sock, strerror(errno));
		goto END;
	}
	////family as IPv4.
	serverAddress.sin_family = AF_INET;
	//Binds the server to all available network interfaces.
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	// 	: Sets the port number for the server.
	serverAddress.sin_port = htons(PORT_NUMBER);

	int rc = bind(sock, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
	if (rc < 0)
	{
		ESP_LOGE(tag, "bind: %d %s", rc, strerror(errno));
		goto END;
	}
	else
	{
		printf("\n binded");
	}

	rc = listen(sock, 5);
	if (rc < 0)
	{
		ESP_LOGE(tag, "listen: %d %s", rc, strerror(errno));
		goto END;
	}
	else
	{
		printf("\n listen");
	}

	while (1)
	{

		printf("\n while iam sarching for new socet");
		socklen_t clientAddressLength = sizeof(clientAddress);
		clientSock = accept(sock, (struct sockaddr*)&clientAddress, &clientAddressLength);

		if (clientSock < 0)
		{
			ESP_LOGE(tag, "accept: %d %s", clientSock, strerror(errno));
			goto END;
		}
		else
		{
			printf("\n **************************client connected to wifi %d", clientSock);

			// Wait for acknowledgment before proceeding
			while (1)
			{
				if(wait_for_ack(clientSock))
				{
					disconnect_wifi();
				}
			}

		}
	}

	END:
	ESP_LOGW(tag, "end of socket_server_task");
	close(sock);
	vTaskDelete(NULL);
}

void API_TCP_Server(void)
{
	printf("\n Wifi configarations initiated \n");
	nvs_flash_init();
	wifi_start_access_point();
	xTaskCreate(socket_server_task, "socket_server_task", 8192, NULL, 5, NULL);

}

