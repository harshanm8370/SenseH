
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
//#include "protocol_examples_common.h"
#include "API_TCP_Server.h"
#include "API_Flash_org.h"
#include <stdbool.h>
int variouble;
#define FALSE 0
#define PORT_NUMBER 5002
static char tag[] = "socket_server";

int clientSock ;
extern int globalFlag;

void wifi_send_data(uint8_t* data, size_t length)
{

	if (clientSock != -1 && clientSock > 0 )
	{
		ESP_LOGI(tag, "Sending data...");

		ESP_LOGI(tag, "Raw data:");

		for (size_t i = 0; i < length; i++)
		{
			printf("%02X ",data[i]);
		}
		printf("\n");

		int bytes_sent = send(clientSock, (uint8_t*)data, length, 0);
		vTaskDelay(pdMS_TO_TICKS(500));

		if (bytes_sent < 0)
		{
			ESP_LOGE(tag, "send failed: errno %d", errno);
		}
		else
		{
			ESP_LOGI(tag, "Sent %d bytes", bytes_sent);
			if(data[1]== 0x03)
			{
				close(clientSock);
				printf("\n hahahahhahahahhaahahha iam closing client socket");
				/*while (wait_for_ack(clientSock))
				{
					//BT_process_requests();
					break; // waiting till I get the ack
				}

				if (!wait_for_ack(clientSock))
				{
					ESP_LOGI(tag, "No acknowledgment received");
				}*/
			}

		}
	}
	else
	{
		printf("\n clientSock == -1 \n");
	}
}

bool wait_for_ack(int socket)
{
	char ack_buffer[10];

	memset(ack_buffer, 0, sizeof(ack_buffer));

	int len = recv(socket, ack_buffer, sizeof(ack_buffer) - 1, 0);

	// Print information about each received character
	printf("\n ++++++++++++ Received String (Debug): ");
	for (int i = 0; i < len; i++)
	{
		printf("%c ", ack_buffer[i]);
	}
	printf("\n");

	if (len < 0)
	{
		ESP_LOGE(tag, "recv failed: errno %d", errno);
		return false;
	}
	else if (len == 0)
	{
		ESP_LOGI(tag, "Connection closed");
		return false;
	}
	else
	{
		ESP_LOGI(tag, "Received String: %s", ack_buffer);

		// Check the content of the string for acknowledgment
		if (strcmp(ack_buffer, "START") == 0)
		{
			variouble = 1;
			printf("\n variouble value is changed");
			return true; // Break out of the loop
		}
		else if (strcmp(ack_buffer, "END") == 0)
		{
			ESP_LOGI(tag, "Received 'END'. Closing connection.");
			//BT_process_requests();
				//	close(clientSock);
			return true; // Break out of the loop
		}

	}

	// Return false if no specific condition matched
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
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//socket system call
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
        printf("\n while");
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
            while (wait_for_ack(clientSock))
            {
                break; // Waiting till I get the ack
            }

            if (!wait_for_ack(clientSock))
            {
            	 ESP_LOGI(tag, "No acknowledgment received");
            }
            else
            {
                printf("\n Acknowledgment received. Proceeding...");

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

