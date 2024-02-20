
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

int temp=0;
typedef struct {
	uint8_t wifi_status;
	uint16_t wifi_len;
	uint16_t wifi_buf[BUF_SIZE];
} wifi_BUF_t;
static wifi_BUF_t wifi_buf_rx;

uint32_t API_Wifi_Receive(uint8_t *data_buf)
{
	printf("\n %s",__func__);
	if (wifi_buf_rx.wifi_status == Wifi_BUF_FULL)
	{
		for(int i=0;i<wifi_buf_rx.wifi_len;i++)
		{
			data_buf[i]=wifi_buf_rx.wifi_buf[i];
		}
		printf("copying the Received Bytes: ");
		for (size_t i = 0; i < wifi_buf_rx.wifi_len; ++i) {
			printf("%02X ", data_buf[i]);
		}
		printf("\n");

		memset(wifi_buf_rx.wifi_buf, 0, sizeof(wifi_buf_rx.wifi_buf));
		wifi_buf_rx.wifi_status = Wifi_BUF_EMPTY;

		return wifi_buf_rx.wifi_len;
	}
	else
	{
		return Wifi_BUF_EMPTY;
	}
	return Wifi_BUF_EMPTY;
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


bool wait_for_ack(int socket)
{
	printf("%s",__func__);
	int buff[60];
	memset(&wifi_buf_rx.wifi_buf, 0, sizeof(wifi_buf_rx.wifi_buf));
	printf("\n reciving the frame format");
	wifi_buf_rx.wifi_len = recv(socket, buff, sizeof(buff) - 1, 0);
	printf("\n recieved");
	if (wifi_buf_rx.wifi_len == 0 || wifi_buf_rx.wifi_len < 0)
	{
		printf("\n iam in if");
		ESP_LOGI(tag, "Connection closed");
		return false;
	}
	else
	{
		printf("\n iam in else ");
		wifi_buf_rx.wifi_len=6;
		wifi_buf_rx.wifi_buf[0]=192;
		wifi_buf_rx.wifi_buf[2]=03;
		wifi_buf_rx.wifi_buf[3]=192;
		wifi_buf_rx.wifi_buf[4]=00;
		wifi_buf_rx.wifi_buf[5]=00;
		int recv=0x14;
		wifi_buf_rx.wifi_buf[1]=recv;
		printf("\n ++++++++++++ Received String (Debug): ");
		for(int i=0;i< wifi_buf_rx.wifi_len;i++)
		{
			printf("%d ",wifi_buf_rx.wifi_buf[i]);
		}

		if(wifi_buf_rx.wifi_buf[1] == 0x20)
		{
			temp=1;
		}
		else
		{
			temp=0;
		}

		wifi_buf_rx.wifi_status = Wifi_BUF_FULL;
		int flag = BT_process_requests();
		printf("\n temp:%d",temp);
		printf("\n %d ",wifi_buf_rx.wifi_buf[1]);
		printf("\n 1 record sent");
		// return (wifi_buf_rx.wifi_buf[1] == 20);
		return temp;

	}
	printf("\n no last return");
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
			while(1)
			{
				printf("\n iam in while tcp");
				if(wait_for_ack(clientSock))
				{
					printf("\n iam disconnecting");
					disconnect_wifi();
					break;
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

