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
#include "ECG_12_Lead.h"

extern uint8_t Device_stat;

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
#define test 1


TaskHandle_t myTaskHandle = NULL;



 void Application_Run(void)
{
	 Device_stat = 0;
	 ota_flag = 0;
	// flag = 1;
	static SELECTED_TEST_t state;
	 xTaskCreate(Led_Blink, "Led_Blink", 4096, NULL, 5, &myTaskHandle);

	printf("\nSystem turning on..");


	//xTaskCreate(myTask, "My Task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
	// Start the scheduler
   // vTaskStartScheduler();

	/*API_IO_Exp_PowerOnReset_Configuration();

	API_Battery_monitor_Init();

	if(IsUSB_Charger_Connected())
	{
        if(API_ADC_Read_Battery_Voltage() < (FUEL_GUAGE_MIN_VOLTAGE+0.4))
        {
          printf("\nLow Battery Voltage.. Device Shutting Down.");
          EnterSleepMode(SYSTEM_DEEP_SLEEP);
        }
	}*/

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ECG_CSn_VCS], PIN_FUNC_GPIO);

    gpio_set_direction(ECG_CSn_VCS, GPIO_MODE_OUTPUT);
    gpio_set_direction(MAX30101_DRDY_INTR_PIN,GPIO_MODE_INPUT);
    gpio_set_pull_mode(MAX30101_DRDY_INTR_PIN,GPIO_PULLUP_ONLY);
    gpio_set_level(ECG_CSn_VCS, 1);


	API_TIMER_Run_1MS_Timer();

	printf("\n initializing IO EXP");
	API_IO_Exp_init();
	printf("\n IOEXP DONE!");
	//API_IO_Exp1_P0_write_pin(EFM_DISP_EN2,LOW);
    //API_IO_Exp1_P1_write_pin(EFM_DISP_EN1,HIGH);

    Interfaces_init();
    API_IO_Exp1_P0_write_pin(HIBERNATE,HIGH);//it should be high to  initialize the max30102

   // API_RUN_TEMPERATURE_TEST();

	if(API_Flash_Initialize_Data_pointers() == RECORDS_UPDATE_FAILED)
	   {
		   printf("\n Fail: API_Flash_Initialize_Data_pointers");
	   }

		/*** Testing code, need to be removed during the Release */

		/**************************************************************/
	//	TestFlashStorage();

	//	while(1){}
		/**************************************************************/
	 API_IO_Exp_Power_Control(EN_ANALOG,HIGH);
    printf("\n initializing ADS1293");
	API_ECG_Init();
    printf("\n ADS1293 init");
    API_IO_Exp_Power_Control(EN_ANALOG,LOW);

	  API_DISP_SenseSemi_Logo(STATIC_IMAGE);
	  API_DISP_Firmware_Version();

	/* Delay Provided for the User to read the Firmware version */
	Delay_ms(3000);
	POR_Init();
	/** Testing */
	/***************************************************/


#if test
	Selected_PID_type = VALID_PID;
#endif

	/***************************************************/
		char datetime[18]; // Assuming you want to store the result in a char array
	    const char *customFormat = "%Y-%m-%d %H:%M:%S"; // Example date-time format

	      API_IO_Exp1_P0_write_pin(EFM_DISP_EN2,LOW);
	      API_IO_Exp1_P1_write_pin(EFM_DISP_EN1,HIGH);
	  //  uint16_t result[5] ={0};

	    //API_Disp_Quick_Test_Result(result);
	    //Delay_ms(15000);
	     /* API_IO_Exp_Power_Control(EN_VLED,LOW);
	      while(1)
	      {
	      printf("\nDevice ID = 0x%2X.\n",readPartID(0xFF));
	      }*/
#if !onep2
    	  if(API_Check_USB_Charger_Connection_Display_Notification())
    	  {
    		  EnterSleepMode(SYSTEM_DEEP_SLEEP);
    	  }
#endif



	while(1)
	{
		//ota_flag = 1;
		if(flash_data.sys_mode == DEVICE_ACTIVE_MODE)
		{
			    if(Detect_low_battery_display_notification()==false)
				{
#if test
			    	 state = VIEW_SCREEN;
#else
			    	 state = API_Disp_Select_PID_Screen();
			    	 Device_stat = 1;
#endif
			    	    if(state == VIEW_SCREEN)
			    	    {
			    	    	state = API_Display_View_Screen();
			    	    	//ota_flag = 1;
			    	    	//API_RTC_Update_Date_Time(8, 8, 2023, 16, 24, 17);
							API_RTC_Get_Date_Time(datetime, customFormat);
							printf("Formatted Date and Time: %s\n", datetime);
			    	    }



						if(state == DATA_SYNC)
						{
							HandleDataSync();
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
							API_TIMER_Kill_Timer(USER_INACTIVE_TIMEOUT);
						}
				/*	     if(Is_Device_Paired == BT_DISCONNECTED) // Paired condition
						 {
								Selected_PID_type = PID_NOT_SELECTED;
						 }*/

						switch(state)
						{
							case QUICK_VITALS:{
                                qv_flag = 11;
                                Device_stat = 2;
							//	Is_Test_In_Progress = true;
#if !test
								if(Is_Device_Paired == DEFAULT) // Paired condition
								{
									//printf("\n\t%d",Is_Device_Paired);
								   Selected_PID_type = PID_NOT_SELECTED;
								}
#endif
								Run_Quick_Vital();
								Device_stat  = 3;
#if !test
								if(Is_Device_Paired == DEFAULT) // Paired condition
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
								qv_flag = 12;
								Device_stat = 2;
								//	Is_Test_In_Progress = true;
#if !test
								if(Is_Device_Paired == DEFAULT) // Paired condition
								{
									//printf("\n\t%d",Is_Device_Paired);
									Selected_PID_type = PID_NOT_SELECTED;
								}
#endif
								Run_Quick_Vital();
								Device_stat  = 3;
#if !test
								if(Is_Device_Paired == DEFAULT) // Paired condition
								{
									Selected_PID_type = PID_NOT_SELECTED;
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
								if(Is_Device_Paired == DEFAULT) // Paired condition
								{
									Selected_PID_type = PID_NOT_SELECTED;
							    }
#endif
								Run_Multi_Vital();
								Device_stat = 3;
#if !test
								if(Is_Device_Paired == DEFAULT) // Paired condition
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

						if (is_firmware_data_available())
						{
							//API_display_backlight_on();
							/*if(Is_Device_Paired == BT_DISCONNECTED) // Paired condition
						    {
								    API_display_backlight_on();
									API_DISP_Display_Screen(DISP_DEVICE_UPGRADATION_FAIL);
									ota_flag = 0;
									API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
							}*/
							ota_flag = 1;
							API_TIMER_Kill_Timer(USER_INACTIVE_TIMEOUT);
							Firmware_upgrade();
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
	printf("\n initializing FLASH");
    API_FLASH_init();
    printf("\n FLASH init DONE!");

	/*********** Display ********************************************************/
	API_IO_Exp_Reset();

	Power_Up_All_Modules();
	printf("\n initializing FLASH");
	API_Display_interface_init();
	printf("\n Display init done!");
	API_DISP_Clear_Full_Screen_3_Wire(WHITE);

	/*********** MAX86150 ********************************************************/
    API_IO_Exp_Power_Control(EN_VLED,HIGH);
	printf("\n initializing MAX30101");
	API_MAX30101_I2C_Init();
	API_MAX30101_Setup();
	API_IO_Exp_Power_Control(EN_VLED,LOW);
	printf("\n MAX30101 init done!");

	/*********** Battery Monitor  ***********************************************/
	printf("\n initializing BAT monitor");
	API_Battery_monitor_Init();
    printf("\n BAT monitor init done");
	/****************** RTC init  ***********************************************/
    printf("\n RTC initializing");
	API_RTC_Update_Date_Time(0,0,0,0,0,0);
	  printf("\n RTC init done");
	/*********** Push Button ****************************************************/
	  printf("\npush button initialising");
	API_Push_Btn_init();
	  printf("\n push button init done");
	/**************** IR ADC init ***********************************************/

	API_IR_ADC_Init();

	/**************** BLE init ***********************************************/
	printf("\n BLE initialising");
   API_BLE_Init();
   printf("\n BLE init done");
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
			if(flag++ < 250)
			{
				API_display_backlight_off();
			     API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,LOW);
			}
			else if(flag > 250)
			{
				if(flag == 500)
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
				break;
			  	//API_Disp_BT_Icon(GREEN);
			  	//Is_Device_Paired = DEFAULT;// to avoid Redisplaying the same thing again
			 }

			if(IsValidRecordsInFlash == FALSE)// No records in flash
			{
				API_display_backlight_on();
				API_DISP_Display_Screen(DISP_DATA_SYNC_COMPLETED);
				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,LOW);
			    Delay_ms(1000);
				API_IO_Exp1_P1_write_pin(NOTIFICATION_LED,HIGH);
				break;
			}

			if((btn_press == 1) || ((btn_press == 2)))
			{
				API_DISP_Display_Screen(DISP_DATA_SYNC_IN_PROGRESS);
				Delay_ms(200);
				API_display_backlight_on();
				Delay_ms(2000);
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
