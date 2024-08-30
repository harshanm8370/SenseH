#include "MainFlow.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "esp_ota_ops.h"
#include "max30101.h"
#include "API_Display.h"
#include "API_adc.h"
#include "API_IO_Exp.h"
#include "API_timer.h"
#include "rtc.h"
#include "API_Battery_monitor.h"
#include "API_Flash.h"
#include "push_button.h"
#include "API_ADS.h"
#include "Quick_Test.h"
#include "bluetooth.h"
#include "OTA_Upgrade.h"
#include "API_Bluetooth.h"
#include "Battery_management.h"
#include "ProjectConfiguration.h"
#include "Hardware.h"
#include "esp_sleep.h"
#include "API_utility.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "API_TCP_Server.h"
#include "ECG_12_Lead.h"

extern uint8_t Device_stat;
extern bool task_close;
static void Interfaces_init(void);
void POR_Init(void);
void Manage_Device_Sleep(void);
bool flag = 1,ota_flag=1;
void TestNbfsTests_PerBatteryFull(void);
void ExecuteComplianceSequence(void);
void HandleDataSync(void);
void Led_Blink(void *pvParameters);
extern BT_STATUS Is_Device_Paired;
uint8_t qv_flag,mv_flag;
bool BLE_DS;
extern bool ota_data_avlbl;
extern uint32_t FW_data_len;
#define test 0



TaskHandle_t myTaskHandle = NULL;



 void Application_Run(void)
{
	 Device_stat = 0;
	 ota_flag = 0;
	// flag = 1;
	static SELECTED_TEST_t state;
	 xTaskCreate(Led_Blink, "Led_Blink", 4096, NULL, 5, &myTaskHandle);

	printf("\nSystem turning on..");


    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ECG_CSn_VCS], PIN_FUNC_GPIO); //Ecg chip select initialising as GPIO

    gpio_set_direction(ECG_CSn_VCS, GPIO_MODE_OUTPUT);  //setting csn as output pin

    gpio_set_level(ECG_CSn_VCS, 1); // initializing as high


	API_TIMER_Run_1MS_Timer(); //timer init

	API_IO_Exp_init(); //IOEXP init

    Interfaces_init(); // max30102,flash,push button,spi,i2c,battery monitor inits
    API_IO_Exp1_P0_write_pin(HIBERNATE,HIGH);//it should be high to  initialize the max30102

	if(API_Flash_Initialize_Data_pointers() == RECORDS_UPDATE_FAILED) // initialising the pointers
	   {
		   printf("\n Fail: API_Flash_Initialize_Data_pointers");
	   }


	  API_DISP_SenseSemi_Logo(STATIC_IMAGE); //displaing SenseH LOGO while bootup
	  API_DISP_Firmware_Version(); //function display current FW version


#if test
	Selected_PID_type = VALID_PID;
#endif

	/***************************************************/
		char datetime[18]; // Assuming you want to store the result in a char array
	    const char *customFormat = "%Y-%m-%d %H:%M:%S"; // Example date-time format

	      API_IO_Exp1_P0_write_pin(EFM_DISP_EN2,LOW); //EN1 &EN2 represents the display brightness for more refer the data sheet
	      API_IO_Exp1_P1_write_pin(EFM_DISP_EN1,HIGH);

    	  API_Check_USB_Charger_Connection_Display_Notification(); //checking charger is connected or not


	while(1)
	{
		if(flash_data.sys_mode == DEVICE_ACTIVE_MODE)
		{
			    if(Detect_low_battery_display_notification()==false) // check and update battery level
				{
#if test
			    	 state = VIEW_SCREEN;
#else
			    	 state = API_Disp_Select_PID_Screen(); //PID screen it will on till user sents PID through APK
			    	 Device_stat = 1;
#endif
			    	    if(state == VIEW_SCREEN)
			    	    {
			    	    	state = API_Display_View_Screen(); // Test screen will be there after PID is pushed into SenseH
			    	    	//ota_flag = 1;
			    	    	//API_RTC_Update_Date_Time(8, 8, 2023, 16, 24, 17);
							API_RTC_Get_Date_Time(datetime, customFormat); //update date and time received from APK
							printf("Formatted Date and Time: %s\n", datetime);
			    	    }



						if(state == DATA_SYNC)  // when data sync is in progress state will be returning DATA_SYNC
						{
							HandleDataSync();  //During data sync this code will be running. It will turn-off display for power saving blink LED to notify the user data sync in progress
							if(BLE_DS)
							{
								Selected_PID_type = 0;
								BLE_DS = 0;
							}
							state = VIEW_SCREEN;
						}

						 if((state == QUICK_VITALS) || (state == MULTI_VITALS))
						 {
							   if(API_Check_USB_Charger_Connection_Display_Notification())
								{
								   state = 0;//clear the selected test option if the charger is connected.
								}
						 }


						if(state)
						{
							API_TIMER_Kill_Timer(USER_INACTIVE_TIMEOUT); //stoping the timer since there is user operation
						}

						switch(state)
						{
							case QUICK_VITALS:{
                                qv_flag = BP_FLAG; //Setting flag for BP(ECGl1&PPG IR)
                                Device_stat = 2;
#if !test
								if(Is_Device_Paired == DC) // Paired condition
								{
									//printf("\n\t%d",Is_Device_Paired);
								   Selected_PID_type = PID_NOT_SELECTED;
								}
#endif
								Run_Quick_Vital(); // getting into Test function
								Device_stat  = 3;
#if !test
								if(Is_Device_Paired == DC) // Paired condition
								{
									Selected_PID_type = PID_NOT_SELECTED;
								}
#endif
								 qv_flag = 0;
								//ota_flag = 0;
								Is_Test_In_Progress = false;
								break;
							}

							case ECG6LEAD:{
								qv_flag = ECG6_FLAG;
								Device_stat = 2;
								//	Is_Test_In_Progress = true;
#if !test
								if(Is_Device_Paired ==DC) // Paired condition
								{
									//printf("\n\t%d",Is_Device_Paired);
									Selected_PID_type = PID_NOT_SELECTED;
								}
#endif
								Run_Quick_Vital();// getting into Test function
								FW_data_len =0;
								Device_stat  = 3;
#if !test
								if(Is_Device_Paired == BT_DISCONNECTED) // Paired condition
								{
									Selected_PID_type = DC;
								}
#endif
								qv_flag = 0;
								//ota_flag = 0;
								Is_Test_In_Progress = false;
								break;
							}

							case MULTI_VITALS:{
								Is_Test_In_Progress = true;
								mv_flag =1 ;
								Device_stat = 2;
								qv_flag = 0;
#if !test
								if(Is_Device_Paired == DC) // Paired condition
								{
									Selected_PID_type = PID_NOT_SELECTED;
							    }
#endif
								Run_Multi_Vital(); //multivital test part
								Device_stat = 3;
#if !test
								if(Is_Device_Paired == DC) // Paired condition
								{
									Selected_PID_type = PID_NOT_SELECTED;
								}
#endif
								mv_flag = 0;
								Is_Test_In_Progress = false;

								break;
							}

							case TEST_EXIT:{
								flash_data.sys_mode = DEVICE_HIBERNET_MODE;
								Is_Test_In_Progress = false;

								break;
							}

							default : break;
						}

						if(state)
							{
								API_TIMER_Register_Timer(USER_INACTIVE_TIMEOUT);
							}

						if (ota_data_avlbl)
						{
							//API_display_backlight_on();
							printf("\n OTA update detected");
							API_DISP_Display_Screen(DISP_DEVICE_UPGRADING);
							Delay_ms(10000);
							API_display_backlight_off();
							API_TIMER_Kill_Timer(USER_INACTIVE_TIMEOUT);
							while(ota_data_avlbl);
							/*if(Is_Device_Paired == BT_DISCONNECTED) // Paired condition
						    {
								    API_display_backlight_on();
									API_DISP_Display_Screen(DISP_DEVICE_UPGRADATION_FAIL);
									ota_flag = 0;
									API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
							}*/
						//	ota_flag = 1;

			//Firmware_upgrade();
						}

					}

			    else
			    {
			    	flash_data.sys_mode = DEVICE_HIBERNET_MODE;
			    }
		}

		if(flash_data.sys_mode == DEVICE_SLEEP_MODE)
		{
			Manage_Device_Sleep();
		}

		if(flash_data.sys_mode == DEVICE_HIBERNET_MODE)
		{
			EnterSleepMode(SYSTEM_DEEP_SLEEP);

		}
	}

}





 void API_display_backlight_on(void)
 {
 	API_IO_Exp1_P0_write_pin(EFM_DISP_EN2,HIGH);
 	API_IO_Exp1_P1_write_pin(EFM_DISP_EN1,HIGH);
 }
 void API_display_backlight_off(void)
 {
 	API_IO_Exp1_P0_write_pin(EFM_DISP_EN2,LOW);
 	API_IO_Exp1_P1_write_pin(EFM_DISP_EN1,LOW);
 }




 void Led_Blink(void *pvParameters) {
     while (1) {
     	//printf("\n in task function");
     	vTaskDelay(pdMS_TO_TICKS(2000/portTICK_PERIOD_MS));
     	if(ota_flag)
     	{
         // Your task code here
        // vTaskDelay(pdMS_TO_TICKS(1000/portTICK_PERIOD_MS)); // Delay the task for 1000 milliseconds
     		// vTaskDelay(pdMS_TO_TICKS(2000/portTICK_PERIOD_MS)); // Delay the task for 1000 milliseconds
            if(flag)
            {
         	   API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,LOW);
         	   flag = !flag;
         	   //vTaskDelay(pdMS_TO_TICKS(1000/portTICK_PERIOD_MS)); // Delay the task for 1000 milliseconds
            }
            else
            {
 				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
 				flag = !flag;
 				//vTaskDelay(pdMS_TO_TICKS(1000/portTICK_PERIOD_MS)); // Delay the task for 1000 milliseconds
            }
     	}
     }
     vTaskDelete(NULL);

 }


static void Interfaces_init(void)
{
	/**************** Flash init *********************************************/
    API_FLASH_init();

	/*********** Display ********************************************************/
	API_IO_Exp_Reset();

	Power_Up_All_Modules();
	API_Display_interface_init();
	API_DISP_Clear_Full_Screen_3_Wire(WHITE);

	/*********** MAX86150 ********************************************************/
	API_MAX30101_I2C_Init();

	/*********** Battery Monitor  ***********************************************/
	API_Battery_monitor_Init();

	/****************** RTC init  ***********************************************/
	API_RTC_Update_Date_Time(0,0,0,0,0,0);

	/*********** Push Button ****************************************************/
	API_Push_Btn_init();

	/**************** IR ADC init ***********************************************/
	API_IR_ADC_Init();

	/**************** BLE init ***********************************************/
   API_BLE_Init();
}


void POR_Init(void)
{
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);

	API_Select_PID();

	memset(PatientID.pid,'\0',sizeof(PatientID.pid));

	API_Disp_Reset_Screen();
	//API_DISP_Toggle_Date_Time();
	API_Disp_BT_Icon(WHITE);
	flash_data.sys_mode = DEVICE_ACTIVE_MODE;
	Did_Push_Button_Pressed = false;
	Selected_PID_type =PID_NOT_SELECTED;
	flash_data.device_config.registration_config = HOSPITAL;

	API_Reset_Push_Button_State();
	Selected_PID_type = PID_NOT_SELECTED;
	Is_time_displayed = TRUE;
	 BT_ongoing_session = FALSE;
	 Is_Test_In_Progress = FALSE;
	API_TIMER_Register_Timer(DATE_TIME_FLIP_TIME);
	API_TIMER_Register_Timer(USER_INACTIVE_TIMEOUT);
	IsValidRecordsInFlash = true;

	Disable_Power_Supply();

}

void Manage_Device_Sleep(void)
{
	Delay_ms(3000);
	API_DISP_Clear_Screen_Fast(WHITE);

	//API_DISP_Clear_Full_Screen_3_Wire(WHITE);

	EnterSleepMode(SYSTEM_SLEEP);


	API_TIMER_Register_Timer(DEEP_SLEEP_TIMEOUT);

	Did_Push_Button_Pressed = false;


	while(1)
	{
		if(Did_Push_Button_Pressed)
		{
			if(API_Push_Btn_Get_Buttton_Press())
			 {
				POR_Init();
				 Interfaces_init();
				break;
			 }
		}

		if(API_TIMER_Get_Timeout_Flag(DEEP_SLEEP_TIMEOUT))
		{
			EnterSleepMode(SYSTEM_DEEP_SLEEP);
		}
	}

}



void TestNbfsTests_PerBatteryFull(void)
{/*
	static uint32_t nbf_tests_conducted = 0;

    while(1)
    {
    	QUICK_Test1();
    	QUICK_Test2();
    	Lead12_Test();
    	API_Flash_Initialize_Data_pointers();
    	nbf_tests_conducted++;

    	printf("\nnbf_tests_conducted = %d",nbf_tests_conducted);

    	if(Detect_low_battery_display_notification())
    	{
    		printf("\nTotal Tests Completed = %d",nbf_tests_conducted);
    		printf("\nEnterring Into Sleep Mode");
    		Manage_Device_Sleep();
    	}

    }
*/}

void ExecuteComplianceSequence(void)
{
	uint8_t btn_press=0;
	API_ECG_Init();
	API_MAX30101_Setup();
	API_ECG_Reginit_12Lead();
	API_TIMER_Register_Timer(USER_INACTIVE_TIMEOUT);

	   while(1)
	   {
		   btn_press = API_Push_Btn_Get_Buttton_Press();

		   if((API_TIMER_Get_Timeout_Flag(USER_INACTIVE_TIMEOUT) == true) || btn_press)
		   		{
		   			EnterSleepMode(SYSTEM_DEEP_SLEEP);
		   		}
	   }
}

void HandleDataSync(void)
{
	uint8_t btn_press;
	uint16_t flag=0;
	//int Dflag=20000;
//	if(ota_data_avlbl)
//	{
//		return ;
//	}
	API_DISP_Display_Screen(DISP_DATA_SYNC_IN_PROGRESS);
	//API_Disp_Display_Exit_Bottom_Section();

	Delay_ms(1000);
	//API_display_backlight_off();
	if(Detect_low_battery_display_notification()==false)
	{
		while(1)
		{

			/*if(Dflag-- <= 1)
			{
				API_display_backlight_off();
			}*/
			//ota_flag = 1;
			if(flag++ < 150)
			{
				API_display_backlight_off();
			     API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,LOW);
			}
			else if(flag > 150)
			{
				if(flag == 300)
				flag = 0;
				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
			}
			btn_press = API_Push_Btn_Get_Buttton_Press();

			/*if(API_TIMER_Get_Timeout_Flag(DATA_SYNC_TIMEOUT)) // call get_time_out function
			{
				BT_Sync_Timeout_Init_State();
				API_display_backlight_on();
				API_DISP_Display_Screen(DISP_DATA_SYNC_FAIL);
				Delay_ms(2000);
				 EnterSleepMode(SYSTEM_DEEP_SLEEP);
			}*/

			if(Is_Device_Paired == BT_DISCONNECTED) // Paired condition
			 {
				API_display_backlight_on();
				API_DISP_Display_Screen(BLUETOOTH_DISCONNECTED);
				Delay_ms(1000);
				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
				BLE_DS = 1;
				disconnect_wifi();
				Data_sync_in_progress = 0;
				break;
			  	//API_Disp_BT_Icon(GREEN);
			  	//Is_Device_Paired = DEFAULT;// to avoid Redisplaying the same thing again
			 }


			if(task_close)
			{
				API_display_backlight_on();
				API_DISP_Display_Screen(WIFI_DISABLED);
				Delay_ms(1000);
				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
				break;
			}


//			if(IsValidRecordsInFlash == FALSE)// No records in flash
//			{
//				API_display_backlight_on();
//				API_DISP_Display_Screen(DISP_DATA_SYNC_COMPLETED);
//				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,LOW);
//			    Delay_ms(1000);
//				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
//				break;
//			}

			if((btn_press == 1) || ((btn_press == 2)))
			{
				API_DISP_Display_Screen(DISP_DATA_SYNC_IN_PROGRESS);
				Delay_ms(200);
				API_display_backlight_on();
				Delay_ms(5000);
				API_display_backlight_off();

				//break;
			}

		}
	}
	else
	{
		API_display_backlight_on();
		API_DISP_Display_Screen(DISP_DATA_SYNC_FAIL);
		Delay_ms(2000);
		EnterSleepMode(SYSTEM_DEEP_SLEEP);
	}

}

#ifdef TEST_FLASH
uint8_t DstBuffer[DATA_BUFFER3_LENGTH];

void TestFlashStorage(void)
{
	uint8_t totalRecordsToValidate = 10U;

	RECORD_POINTER_t records_pointer;

	MemSet(BT_flash_buffer,0,sizeof(BT_flash_buffer));
	MemSet(DstBuffer,0,sizeof(DstBuffer));

	for(int i=0;i<DATA_BUFFER3_LENGTH;i++)
	{
		BT_flash_buffer[i] = i%0xFF;
	}

	printf("\n Flash storage validation, Please wait........\n");

   for(int i=0;i<totalRecordsToValidate;i++)
   {
	 API_Flash_Write_Record(SPO2,(uint8_t*) BT_flash_buffer);
	 API_Flash_Read_Record(SPO2, (uint8_t*)DstBuffer);

	 get_record_pointer_details (SPO2, &records_pointer);

	 if(memcmp(BT_flash_buffer,DstBuffer,records_pointer.one_record_len) !=0)
	 {
		 printf("\nSPO2 storage fail");
	 }
	 else
	 {
		 printf("\nSPO2 storage pass");
	 }
   }

   for(int i=0;i<totalRecordsToValidate;i++)
     {
		 API_Flash_Write_Record(BP1,(uint8_t*) BT_flash_buffer);
		 API_Flash_Read_Record(BP1, (uint8_t*)DstBuffer);

		 get_record_pointer_details (BP1, &records_pointer);

		 if(memcmp(BT_flash_buffer,DstBuffer,records_pointer.one_record_len) !=0)
		 {
			 printf("\nBP1 storage fail");
		 }
		 else
			 {
				 printf("\nBP1 storage pass");
			 }
     }

   for(int i=0;i<totalRecordsToValidate;i++)
     {
   		 API_Flash_Write_Record(BP2,(uint8_t*) BT_flash_buffer);
   		 API_Flash_Read_Record(BP2, (uint8_t*)DstBuffer);

   		 get_record_pointer_details (BP2, &records_pointer);

   		 if(memcmp(BT_flash_buffer,DstBuffer,records_pointer.one_record_len) !=0)
   		 {
   			 printf("\nBP2 storage fail");
   		 }
   		 else
   			 {
   				 printf("\nBP2 storage pass");
   			 }
     }

   for(int i=0;i<totalRecordsToValidate;i++)
       {
     		 API_Flash_Write_Record(ECG_1_Lead,(uint8_t*) BT_flash_buffer);
     		 API_Flash_Read_Record(ECG_1_Lead, (uint8_t*)DstBuffer);

     		 get_record_pointer_details (ECG_1_Lead, &records_pointer);

     		 if(memcmp(BT_flash_buffer,DstBuffer,records_pointer.one_record_len) !=0)
     		 {
     			 printf("\ECG_1_Lead storage fail");
     		 }
     		 else
     			 {
     				 printf("\nECG_1_LEAD storage pass");
     			 }
       }

   for(int i=0;i<totalRecordsToValidate;i++)
        {
      		 API_Flash_Write_Record(ECG_6_Lead,(uint8_t*) BT_flash_buffer);
      		 API_Flash_Read_Record(ECG_6_Lead, (uint8_t*)DstBuffer);

      		 get_record_pointer_details (ECG_6_Lead, &records_pointer);

      		 if(memcmp(BT_flash_buffer,DstBuffer,records_pointer.one_record_len) !=0)
      		 {
      			 printf("\nECG_6_Lead storage fail");
      		 }
      		 else
      			 {
      				 printf("\nECG_6_Lead storage pass");
      			 }
        }

   for(int i=0;i<totalRecordsToValidate;i++)
         {
       		 API_Flash_Write_Record(ECG_12_LEAD,(uint8_t*) BT_flash_buffer);
       		 API_Flash_Read_Record(ECG_12_LEAD, (uint8_t*)DstBuffer);

       		 get_record_pointer_details (ECG_12_LEAD, &records_pointer);

       		 if(memcmp(BT_flash_buffer,DstBuffer,records_pointer.one_record_len) !=0)
       		 {
       			 printf("\nECG_12_LEAD storage fail");
       		 }
       		 else
			 {
				 printf("\nECG_12_Lead storage pass");
			 }
         }

   for(int i=0;i<totalRecordsToValidate;i++)
           {
         		 API_Flash_Write_Record(SENSE_TEMP,(uint8_t*) BT_flash_buffer);
         		 API_Flash_Read_Record(SENSE_TEMP, (uint8_t*)DstBuffer);

         		 get_record_pointer_details (SENSE_TEMP, &records_pointer);

         		 if(memcmp(BT_flash_buffer,DstBuffer,records_pointer.one_record_len) !=0)
         		 {
         			 printf("\nSENSE_TEMP storage fail");
         		 }
         		 else
				 {
					 printf("\nSENSE_TEMP storage pass");
				 }
           }

}

#endif
