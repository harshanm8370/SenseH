
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
#include "OTA_Upgrade.h"

#include <esp_ota_ops.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "errno.h"
#include "esp_log.h"
#include "API_utility.h"
#include "API_IO_Exp.h"
#include "API_Display.h"

#define BUFFSIZE 			1024
bool ota_data_avlbl =0;
extern uint16_t bt_total_received_bytes;
uint8_t ota_write_data[BUFFSIZE + 1];
FIRMWARE_UPGRADE_STATE_t Upgrade_state;
static const char *TAG = "ota";
#define HASH_LEN 			32 /* SHA-256 digest length */
//typedef enum { FALSE = 0, TRUE = 1 } Boolean;
#define TIME_OUT_DELAY		60   //wait delay in sec
#define BUFFSIZE 			1024
//Global and Static varioubles
bool is_OTA_request_arrived;
 uint32_t FW_buff_index;
 uint64_t Application_len;
#define FALSE 0
#define PORT_NUMBER 5002
static char tag[] = "socket_server";
extern bool task_close,start;
int clientSock ;
static int sock;
int recived;
int temp_flag=0;
TaskHandle_t Handle = NULL;
uint16_t received;
extern uint32_t FW_data_len;
bool ota_f;
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
	uint8_t wifi_buf[BUF_SIZE];
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

void EVAL_REQ_VITAL_CMD(uint16_t *received, char *buff) {
    if (!strcmp(buff, "BP")) {
        printf("\nBP selected");
        *received = 0x10;
    } else if (!strcmp(buff, "SPO2")) {
        printf("\nSPO2 selected");
        *received = 0x14;
    } else if (!strcmp(buff, "ECG6")) {
        printf("\nECG6 selected");
        *received = 0x13;
    } else if (!strcmp(buff, "ECG1")) {
        printf("\nECG1 selected");
        *received = 0x12;
    } else if (!strcmp(buff, "ECG12")) {
        printf("\nECG12 selected");
        *received = 0x16;
    } else if (!strcmp(buff, "OFF")) {
        printf("\nOFF selected");
        *received = 0x20;
    }else if (!strcmp(buff, "OTA")) {
        printf("\nOTA selected");
        *received = 0x07;
    }
}
uint8_t OTA_UPGRADE(int socket)
{
	ota_data_avlbl = 1;
	ESP_LOGI(TAG, "OTA example app_main start");

			    uint8_t sha_256[HASH_LEN] = { 0 };
			    esp_partition_t partition;

			    // get sha256 digest for the partition table
			    partition.address   = ESP_PARTITION_TABLE_OFFSET;
			    partition.size      = ESP_PARTITION_TABLE_MAX_LEN;
			    partition.type      = ESP_PARTITION_TYPE_DATA;
			    esp_partition_get_sha256(&partition, sha_256);
			    print_sha256(sha_256, "SHA-256 for the partition table: ");

			    // get sha256 digest for bootloader
			    partition.address   = ESP_BOOTLOADER_OFFSET;
			    partition.size      = ESP_PARTITION_TABLE_OFFSET;
			    partition.type      = ESP_PARTITION_TYPE_APP;
			    esp_partition_get_sha256(&partition, sha_256);
			    print_sha256(sha_256, "SHA-256 for bootloader: ");

			    // get sha256 digest for running partition
			    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
			    print_sha256(sha_256, "SHA-256 for current firmware: ");

			    const esp_partition_t *running = esp_ota_get_running_partition();
			    esp_ota_img_states_t ota_state;
			    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
			        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
			            // run diagnostic function ...
			            bool diagnostic_is_ok = diagnostic();
			            if (diagnostic_is_ok) {
			                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
			                esp_ota_mark_app_valid_cancel_rollback();
			            } else {
			                ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
			                esp_ota_mark_app_invalid_rollback_and_reboot();
			            }
			        }
			    }

			    // Initialize NVS.
			    esp_err_t err = nvs_flash_init();
			    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			        // OTA app partition table has a smaller NVS partition size than the non-OTA
			        // partition table. This size mismatch may cause NVS initialization to fail.
			        // If this happens, we erase NVS partition and initialize NVS again.
			        ESP_ERROR_CHECK(nvs_flash_erase());
			        err = nvs_flash_init();
			    }
			    ESP_ERROR_CHECK( err );

	//OTA_firmwareupdate function init
			    int timer;
			    	int data_read;
			       // esp_err_t err;
//			    	API_DISP_Display_Screen(DISP_DEVICE_UPGRADING);
//			    	Delay_ms(5000);
//			    	API_display_backlight_off();
			        /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
			        esp_ota_handle_t update_handle = 0 ;
			        const esp_partition_t *update_partition = NULL;

			        ESP_LOGI(TAG, "Starting OTA firmware update");

			    	Upgrade_state = DEVICE_UPGRADE_START;
			        const esp_partition_t *configured = esp_ota_get_boot_partition();
			        running = esp_ota_get_running_partition();

			        if (configured != running) {
			            ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08lx, but running from offset 0x%08lx",
			                     configured->address, running->address);
			            ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
			        }
			        ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08lx)",
			                 running->type, running->subtype, running->address);

			        update_partition = esp_ota_get_next_update_partition(NULL);
			        assert(update_partition != NULL);
			        ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx",
			                 update_partition->subtype, update_partition->address);

			        int binary_file_length = 0;
			        /*deal with all received packets*/
			        bool image_header_was_checked = false;

			    	timer = TIME_OUT_DELAY * 2;


			    	FW_complete_data_received = FALSE;
			    	is_OTA_request_arrived = FALSE;
			    	uint16_t len=0;
			    	bool flag =1;
			while(!FW_complete_data_received )
			{

				if(flag)
				{
					//API_display_backlight_off();
					API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,LOW);
					flag = !flag;
				}
				else
				{
					flag = !flag;
					API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
				}
				memset(&wifi_buf_rx.wifi_buf, 0, sizeof(wifi_buf_rx.wifi_buf));
				wifi_buf_rx.wifi_len = recv(socket, (uint8_t*)wifi_buf_rx.wifi_buf, 511/*sizeof(wifi_buf_rx.wifi_buf) - 1*/, 0);
	//			for(int i=0;i<wifi_buf_rx.wifi_len;i++)
	//			{
	//				printf(" %02x, ",wifi_buf_rx.wifi_buf[i]);
	//			}
	//			printf("\n");

				len  = wifi_buf_rx.wifi_buf[2] << 8; //computing length field
							len |= wifi_buf_rx.wifi_buf[3];
							if (wifi_buf_rx.wifi_len - 4 != len){ //comparing with received with payload len
								printf("Incorrect length, received: %d, packet len: %d\n", wifi_buf_rx.wifi_len - 4, len);
								//bt_send_ack_or_nack_response(NACK_INVALID_PACKET);
								break;
							}
							else
							{
								printf("correct length, received: %d, packet len: %d\n", wifi_buf_rx.wifi_len - 4, len);
							}
							//TODO: Check CRC for the received packet

							//bt_send_ack_or_nack_response(NACK_DEVICE_BUSY); //Processing data
							if(is_OTA_request_arrived == FALSE){
								is_OTA_request_arrived = TRUE;
								//BT_ongoing_session     = TRUE;
								Data_sync_in_progress  = TRUE;
								FW_buff_index 	       = 0;
								Application_len        = 0;
								FW_complete_data_received = FALSE;
				//				Upgradation_progress = DISP_DEVICE_UPGRADING;
							}
							bt_total_received_bytes = wifi_buf_rx.wifi_len;
							int ret = BTL_validate_and_copy2buf(wifi_buf_rx.wifi_buf + 4);
							if (ret == FW_PACKET_CURRUPTED) {
								//bt_send_ack_or_nack_response(NACK_INVALID_PACKET);
							}
							else if (ret == END_OF_FILE_RECORD){
								FW_complete_data_received = TRUE;
								is_OTA_request_arrived = FALSE;
								//BT_ongoing_session = FALSE;
							} //ACK response will be sent after reading firmware data in case of recevied packet is good
				//			else if(ret == BT_DISCONNECTED)
				//			{
				//				FW_complete_data_received = FALSE;
				//				is_OTA_request_arrived = FALSE;
				//			}

							//OTA data write section
							data_read = read_firmware_data (ota_write_data, BUFFSIZE);

									if(data_read == 2)
									{
										return 2;
									}

									if(Is_Device_Paired == BT_DISCONNECTED) // Paired condition
									{
									      return FALSE;
									}

							        if (data_read > 0) {
							            if (image_header_was_checked == FALSE) {
							            	image_header_was_checked = TRUE;
											Upgrade_state = DEVICE_UPGRADING;
							                err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
							                if (err != ESP_OK) {
								                ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
							                    //esp_ota_abort(update_handle);
							    	        }
							                ESP_LOGI(TAG, "esp_ota_begin succeeded");
							            }
							         //   esp_log_buffer_hex("OTA", ota_write_data, data_read);
							            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
							            if (err != ESP_OK) {
								            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
							                //esp_ota_abort(update_handle);
							            }
							            binary_file_length += data_read;
							            ESP_LOGI(TAG, "Written image length %d", binary_file_length);
										timer = TIME_OUT_DELAY * 2;
							        } else if (data_read == 0) {
										if (FW_complete_data_received == true){
							          		ESP_LOGI(TAG, "Received complete firmware file..");
											break;
										}
							        	//vTaskDelay(500 / portTICK_PERIOD_MS);
										timer--;
										if (timer <= 0){
							          		ESP_LOGI(TAG, "TIME-OUT, firmware packet not received...");
											break;
										}
							        }
			}

			 ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
			    if (FW_complete_data_received != true) {
			        ESP_LOGE(TAG, "Error in receiving complete file");
			       // esp_ota_abort(update_handle);
					Upgrade_state = DEVICE_UPGRADE_FAIL;
					API_display_backlight_on();
					API_DISP_Display_Screen(DISP_DEVICE_UPGRADATION_FAIL);
					Delay_ms(5000);
					esp_restart();
					return false;
			    }
			    err = esp_ota_end(update_handle);
			    if (err != ESP_OK) {
			        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
			            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
			        } else {
			            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
			        }
					Upgrade_state = DEVICE_UPGRADE_FAIL;
					 API_display_backlight_on();
					API_DISP_Display_Screen(DISP_DEVICE_UPGRADATION_FAIL);
					Delay_ms(5000);
					esp_restart();
					return false;
			    }
			    err = esp_ota_set_boot_partition(update_partition);
			    if (err != ESP_OK) {
			        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
					Upgrade_state = DEVICE_UPGRADE_FAIL;
					API_display_backlight_on();
					API_DISP_Display_Screen(DISP_DEVICE_UPGRADATION_FAIL);
					Delay_ms(5000);
					esp_restart();
					return false;
			    }
			    ESP_LOGI(TAG, "Prepare to restart system!");
				Upgrade_state = DEVICE_UPGRADE_SUCCESS;
				API_display_backlight_on();
				API_DISP_Display_Screen(DISP_DEVICE_UPGRADED);
				Delay_ms(5000);
				esp_restart();

}


bool wait_for_ack(int socket)
{
	char Vital_buff[100];
//	printf("\n reciving the frame format");
	memset(&wifi_buf_rx.wifi_buf, 0, sizeof(wifi_buf_rx.wifi_buf));
	memset(&Vital_buff, 0, sizeof(Vital_buff));

	wifi_buf_rx.wifi_len = recv(socket, Vital_buff, sizeof(Vital_buff) - 1, 0);
	if(Is_Test_In_Progress)
		{
			printf("\nTest is in progress, So can not sync the data");
			return false;
		}
	int len =wifi_buf_rx.wifi_len;
	for(int i=0;i<len;i++)
	{
		printf("\n %d %c",i,Vital_buff[i]);
	}

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
		EVAL_REQ_VITAL_CMD(&received, Vital_buff);
		wifi_buf_rx.wifi_buf[1]=received;
		if(received == 0x07)
		{
			printf("GOing OTA function\n");
			OTA_UPGRADE(socket);
		}
		else
		{
			Data_sync_in_progress = TRUE;
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
					.max_connection = 1,
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
int rc;
bool start_sock;
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
	if(!start_sock)
	{
		printf("\n\tstarting socket........");
		sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//socket system call
		start_sock =1;
	}

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
	    rc = bind(sock, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
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
	//ESP_LOGW(tag, " Failed End of socket_server_task");
	//shutdown(sock, 0);
	        //close(listen_sock);
	//close(sock);
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

