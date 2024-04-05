
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
#include <stdint.h>
#include "API_timer.h"
#include <stdio.h>
#include <string.h>


//Global and Static varioubles
#define FALSE 0
#define PORT_NUMBER 5002
static char tag[] = "socket_server";
extern bool task_close,start;
int clientSock ;
static int sock;
int recived;
int temp_flag=0;
TaskHandle_t Handle = NULL;
//Macros
#define BUF_SIZE 600
//Structure and enums
enum {
	Wifi_BUF_EMPTY,
	Wifi_BUF_FULL,
	wifi_TX_RX_FAILED,
	wifi_TX_RX_SUCCESS,
};


typedef struct {
	uint8_t wifi_status;
	uint16_t wifi_len;
	uint16_t wifi_buf[BUF_SIZE];
} wifi_BUF_t;
static wifi_BUF_t wifi_buf_rx;


uint32_t API_Wifi_Receive(uint8_t *data_buf)
{
	if (wifi_buf_rx.wifi_status == Wifi_BUF_FULL)
	{
		for(int i=0;i<wifi_buf_rx.wifi_len;i++)
		{
			data_buf[i]=wifi_buf_rx.wifi_buf[i];
		}
		/*printf("copying the Received Bytes: ");
		for (size_t i = 0; i < wifi_buf_rx.wifi_len; ++i) {
			printf("%02X ", data_buf[i]);
		}
		printf("\n"); */

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

void wifi_restart(void)
{
	// esp_wifi_set_mode(WIFI_MODE_AP);
	// wifi_start_access_point();
	esp_wifi_start();
	start = 1;

}


void disconnect_wifi()
{
	ESP_LOGW(tag, "End of socket_server_task");
	//close(sock);
	esp_wifi_set_mode(WIFI_MODE_NULL);
	task_close = 1;

	//esp_wifi_stop();
//	esp_wifi_deinit();
	//vTaskDelete(NULL);
}

void EVAL_REQ_VITAL_CMD(int *recived, char *buff) {
    if (!strcmp(buff, "BP")) {
        printf("\nBP selected");
        *recived = 0x10;
    } else if (!strcmp(buff, "SPO2")) {
        printf("\nSPO2 selected");
        *recived = 0x14;
    } else if (!strcmp(buff, "ECG6")) {
        printf("\nECG6 selected");
        *recived = 0x13;
    } else if (!strcmp(buff, "ECG1")) {
        printf("\nECG1 selected");
        *recived = 0x12;
    } else if (!strcmp(buff, "ECG12")) {
        printf("\nECG12 selected");
        *recived = 0x16;
    } else if (!strcmp(buff, "OFF")) {
        printf("\nOFF selected");
        *recived = 0x20;
    }
}


bool wait_for_ack(int socket)
{
	char Vital_buff[10];
//	printf("\n reciving the frame format");
	memset(&wifi_buf_rx.wifi_buf, 0, sizeof(wifi_buf_rx.wifi_buf));
	memset(&Vital_buff, 0, sizeof(Vital_buff));

	wifi_buf_rx.wifi_len = recv(socket, Vital_buff, sizeof(Vital_buff) - 1, 0);
	int len =wifi_buf_rx.wifi_len;

	if(0)
	{
		ESP_LOGI(tag, "Connection closed");
		return false;
	}
	else
	{
		printf("\n iam in else ");
		wifi_buf_rx.wifi_len=6;
		wifi_buf_rx.wifi_buf[0]=0XC0;
		wifi_buf_rx.wifi_buf[2]=3;
		wifi_buf_rx.wifi_buf[3]=0XC0;
		wifi_buf_rx.wifi_buf[4]=00;
		wifi_buf_rx.wifi_buf[5]=00;

		//the recived value based upon the client request vital commmand
		EVAL_REQ_VITAL_CMD(&recived, Vital_buff);
		wifi_buf_rx.wifi_buf[1]=recived;

	/*	printf("\n ++++++++++++ Received String (Debug): ");
		for(int i=0;i< wifi_buf_rx.wifi_len;i++)
		{
			printf("%d ",wifi_buf_rx.wifi_buf[i]);
		} */

		if(wifi_buf_rx.wifi_buf[1] == 0x20)
		{
			temp_flag=1;
		}
		else
		{
			temp_flag=0;
		}

		wifi_buf_rx.wifi_status = Wifi_BUF_FULL;
		int flag = BT_process_requests();

		// return (wifi_buf_rx.wifi_buf[1] == 20);
		return temp_flag;

	}

	return false;
}

void wifi_start_access_point() {
	wifi_config_t wifi_config = {
			.ap = {
					.ssid = "SenseH",
					.password = "SenseSemi",
					.channel = 1,
					.authmode = WIFI_AUTH_OPEN,
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


void socket_server_task(void *pvParametrs)
{
	struct sockaddr_in clientAddress;
	int ret=0;
	struct sockaddr_in serverAddress;
	goto START;

	//IPv4 AF_INET
	START:
//	for(int i=0;i<4;i++)
//	{
//		ret = close(sock);
//		printf("\n\t close-%d = %d",i+1,ret);
//	}

//	Delay_ms(1000);
	printf("\n\tstarting socket........");
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//socket system call

	if (sock < 0)
	{
		ESP_LOGE(tag, "socket: %d %s", sock, strerror(errno));
		goto END;
	}
	else
	{
		printf("\n socket created successfully with socket ID : %d",sock);
	}
//	Delay_ms(10000);
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
//		if(ret < 4)
//		goto BIND;

		ret++;
	}
	else
	{
		printf("\nBinded succesfully with socket if %d" ,rc);
	}


	rc = listen(sock, 100);
	if (rc < 0)
	{
		ESP_LOGE(tag, "listen: %d %s", rc, strerror(errno));
		goto END;
	}
	else
	{
		printf("\n Listen success rc : %d",sock);
	}

	while (1)
	{
		if(task_close == 1)
		{
			task_close = 0;
			printf("\n\tsuspending the tassk");
			goto END;
		}
		Delay_ms(1000);

		socklen_t clientAddressLength = sizeof(clientAddress);
//		API_TIMER_Register_Timer((TIMER_t)TIMER_30SEC);
//		do
//		{

			clientSock = accept(sock, (struct sockaddr*)&clientAddress, &clientAddressLength);
//			if(API_TIMER_Get_Timeout_Flag((TIMER_t)TIMER_30SEC)) // call get_time_out function
//			{
//				goto END;
//			}
//		}
//		while(clientSock < 0);


		if (clientSock < 0)
		{
			if(!task_close)
			ESP_LOGE(tag, "accept: %d %s", clientSock, strerror(errno));
			//goto END;
		}
		else
		{
			printf("accepted client socket successfully with client socket id : %d",clientSock);
			if(wait_for_ack(clientSock))
			{
				disconnect_wifi();
				break;
			}

		}
	}

	END:
	ESP_LOGW(tag, " Failed End of socket_server_task");
	//shutdown(sock, 0);
	        //close(listen_sock);
	close(sock);
	vTaskDelete(NULL);
}

bool flag_init;

void API_TCP_Server(void)
{
    if(flag_init)
    {

//    	wifi_config_t wifi_config = {
//    				.ap = {
//    						.ssid = "SenseSemi_Device",
//    						.password = "SenseSemi",
//    						.channel = 1,
//    						.authmode = WIFI_AUTH_OPEN,
//    						.ssid_hidden = 0,
//    						.max_connection = 5,
//    						.beacon_interval = 100
//    				}
//    		};
//
////    		esp_netif_init();//network interface
//    		//ESP_ERROR_CHECK(esp_event_loop_create_default());//default event loop that will be used to handle WiFi-related events
//    		//esp_netif_t* wifiAP = esp_netif_create_default_wifi_ap();//This line creates a default WiFi access point interface
//    		//esp_netif_ip_info_t ip_info;//ip info
//    		printf("\n line - 1");
//    		//IP4_ADDR(&ip_info.ip, 192, 168, 1, 1);
//    	//	IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
//    		printf("\n line - 2");
//    		//IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
//    		//esp_netif_dhcps_stop(wifiAP);//This line stops the DHCP server associated with the access point network interface
//    		//printf("\n line - 3");
//    		//esp_netif_set_ip_info(wifiAP, &ip_info);//This sets the previously configured IP information (IP address, gateway, and netmask) for the access point network interface.
//    		printf("\n line - 4");
//    		//esp_netif_dhcps_start(wifiAP);
//    		printf("\n line - 5");
//    		wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
//    		printf("\n line - 6");
//    		esp_wifi_init(&wifi_init_config);
//    		printf("\n line - 7");
//    		esp_wifi_set_storage(WIFI_STORAGE_RAM);
//    		printf("\n line - 8");
//    		esp_wifi_set_mode(WIFI_MODE_AP);
//    		printf("\n line - 9");
//    		esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
//    		printf("\n line - 10");
    		//esp_wifi_start();
    	esp_wifi_set_mode(WIFI_MODE_AP);


    		//esp_wifi_start();
    	//wifi_restart();
    	//vTaskResume( Handle );
    	task_close = 0;
    	 xTaskCreate(socket_server_task, "socket_server_task", 12288, NULL, 5, NULL);
    }
    else
    {
	    nvs_flash_init();
	    wifi_start_access_point();
	    xTaskCreate(socket_server_task, "socket_server_task", 12288, NULL, 5, &Handle);
	    //socket_server_task();
	    flag_init = 1;
    }

}

