#include "Quick_Test.h"
#include "API_utility.h"
#include "ecg_config.h"
#include "quick_test_config.h"
#include "bpf.h"
#include "ProjectConfiguration.h"
#include "API_Buzzer.h"
#include "API_Flash_org.h"
#include "API_Display.h"
#include "push_button.h"
#include "API_timer.h"
#include <math.h>
#include "max30101.h"
#include "API_IO_Exp.h"
#include "API_ADS.h"
#include "Error_Handling.h"
#include "API_Flash_org.h"
#include "ECG_12_Lead.h"
#include "driver/periph_ctrl.h"
#include "Hardware.h"
#include "bluetooth.h"
#include "Battery_management.h"

// MACROS REQUIRED FOR WDOG SpO2

#define SPO2_WAIT_TIME							5
#define SPO2_NUM_SEC_DATA_CAPTURE		      	5
#define SPO2_NUM_SEC_FINGER_DET_DATA_CAPTUR  	(3 * 2)  // 3 x (two time detecting the finger)
#define SPO2_REGISTER_INIT_TIME               	1
#define SPO2_MODULE_INIT_TIME              		(SPO2_REGISTER_INIT_TIME + SPO2_WAIT_TIME)
#define SPO2_DATA_CAPTURE_TIME             		(SPO2_NUM_SEC_DATA_CAPTURE + SPO2_NUM_SEC_FINGER_DET_DATA_CAPTUR)
#define SPO2_DATA_PROCESSING_TIME          		1  // processing time is less than 1sec
#define SPO2_DATA_STORAGE_TIME             		1
//#define SPO2_DISPLAY_DELAY_TIME            		(DISP_RESULT_TIME + (DISP_NOTIFICATION_TIME * 3))// 3 display screens will encounters the delay of  "DISP_NOTIFICATION_TIME"
#define SPO2_WDOG_TIME_FEED                		(SPO2_MODULE_INIT_TIME + SPO2_DATA_CAPTURE_TIME + SPO2_DATA_PROCESSING_TIME + SPO2_DISPLAY_DELAY_TIME + SPO2_DATA_STORAGE_TIME)


// MACROS REQUIRED FOR WDOG BP

#define BP_NUM_SEC_DUMMY_DATA_CAPTURE   		BP_NUM_SEC_DATA_CAPTURE
#define BP_REGISTER_INIT_TIME           		2
#define BP_WAIT_TIME		            		5
#define BP_MODULE_INIT_TIME            			(BP_REGISTER_INIT_TIME + BP_WAIT_TIME)
#define BP_DATA_CAPTURE_TIME           			(BP_NUM_SEC_DATA_CAPTURE * 2)
#define BP_DATA_PROCESSING_TIME         		1
#define BP_DATA_STORAGE_TIME            		(3 * 2) // BP + ECG
//#define BP_DISPLAY_DELAY_TIME         			(DISP_RESULT_TIME + (DISP_NOTIFICATION_TIME * 3))	// 3 display screens will encounters the delay of  "DISP_NOTIFICATION_TIME"
#define BP_WDOG_TIME_FEED              			(BP_MODULE_INIT_TIME + BP_DATA_CAPTURE_TIME + BP_DATA_PROCESSING_TIME + BP_DISPLAY_DELAY_TIME + BP_DATA_STORAGE_TIME)

#define QUICK_TEST_BUZZER_ON_TIME              	(2 + 5)

#define QUICK_TEST_TOTAL_TIME_SPO2              (SPO2_MAX_CAPTURE_ITERATIONS * SPO2_NUM_SEC_DATA_CAPTURE)
#define QUICK_TEST_TOTAL_TIME_BP                (BP_MAX_CAPTURE_ITERATIONS * BP_NUM_SEC_DATA_CAPTURE)

#define QUICK_TEST_SPO2_REG_INIT_STABILIZATION_TIME     5
#define QUICK_TEST_BP_REG_INIT_STABILIZATION_TIME       2

//#define QUICK_TEST_WDOG_TIME_FEED	(BP_WDOG_TIME_FEED + SPO2_WDOG_TIME_FEED + QUICK_TEST_BUZZER_ON_TIME + WDOG_EXTRA_TIME + 90+5+40)

/* Quick Test Flash Storage Configuration Macros */

/***************************************************/
#define BP_PPG_RED_SAMPLES 1200U
#define BP_ECG_L1_SAMPLES  1200U
#define ECG_L1_SAMPLES  600U
#define ECG_L2_SAMPLES  600U
#define SPO2_RED_SAMPLES 600U
#define SPO2_IR_SAMPLES  600U

/***************************************************/
uint32_t SPO2_PPG_IR_BUFF[TOTAL_SAMPLES];
uint32_t SPO2_PPG_RED_BUFF[TOTAL_SAMPLES];
//uint32_t SPO2_PPG_ECG_BUFF[TOTAL_SAMPLES];
float ECG_Lead1_buff[TOTAL_SAMPLES_VCS];
float ECG_Lead2_buff[TOTAL_SAMPLES_VCS];
//float ECG_TEMP[TOTAL_SAMPLES_VCS];
float ECG_Lead3_buff[TOTAL_SAMPLES_VCS];
float BP_ECG_Lead1_buff[TOTAL_SAMPLES_VCS];
uint32_t BP_PPG_RED_BUFF[TOTAL_SAMPLES];
uint32_t BP_PPG_IR_BUFF[TOTAL_SAMPLES];
//float FilterOutputBuffer1[TOTAL_SAMPLES];
//float FilterOutputBuffer2[TOTAL_SAMPLES];
//float FilterOutputBuffer3[TOTAL_SAMPLES];
//float FilterOutputBuffer4[TOTAL_SAMPLES];
extern uint8_t qv_flag,mv_flag;
char date_info[50];
uint8_t reg;
extern uint32_t ppg_count;
typedef struct __attribute__((__packed__))
{
	uint32_t  		record_len;
	uint8_t  		patient_id[10];
	uint8_t  		day;
	uint8_t  		month;
	uint8_t  		year;
	uint8_t  		hour;
	uint8_t  		minute;
	uint8_t  		second;
	float 			sbp_multiplier;
	float 			dbp_multiplier;
}BP_MULTIPLIER_TO_FLASH;

typedef enum {
			TEST_SUCCESS = 0,
			SpO2_REG_INIT_FAILED ,
			BP_ECG_REG_INIT_FAILED,
			BP_PPG_REG_INIT_FAILED,
			SpO2_CAPTURE_FAILED,
			BP_CAPTURE_FAILED,
			SpO2_PEAKS_NOT_DETECTED,
			SpO2_VALLEY_NOT_DETECTED,
			SpO2_AC_DC_NOT_IN_RANGE,
			SpO2_NOT_IN_RANGE,
			SpO2_FLASH_WRITE_FAILED,
			BP_ECG_PEAKS_NOT_DETECTED,
			BP_PPG_PEAK_NOT_DETECTED,
			BP_SBP_DBP_NOT_IN_RANGE,
			BP_FLASH_WRITE_FAILED,
			ECG_FLASH_WRITE_FAILED

}QUICK_TEST_STATUS;



bool Lead12_LeadOff_Detect(void);

bool skip_quick_test = FALSE;




 bool Run_Quick_Vital(void)
{
	//uint16_t result[4] = {0};


	printf("\nQuick Test2 Started");

	Is_time_displayed = TRUE;
	//API_DISP_Toggle_Date_Time();
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);

	 API_IO_Exp_Power_Control(EN_VLED,HIGH);
	 API_IO_Exp_Power_Control(EN_ANALOG,HIGH);
	// API_IO_Exp_Power_Control(EN_IR,HIGH);



	if((Selected_PID_type == VALID_PID) || (Selected_PID_type == GUEST_PID))
	{
			if(API_Flash_Org_Check_For_Memory_Free())
			{

				//API_Buzzer_Sound(SHORT_BEEP);

				API_Disp_Quick_Test_Icon();

				API_Disp_Quick_test_screen(DISP_QT_PLACE_FINGER);

				printf("\nEnteriing into ECG init");

				Enable_Power_Supply();
			//	Select_Vlead(LEAD6);// Enable only 6 lead
				API_IO_Exp_Power_Control(EN_IR,LOW);
				API_ECG_Chip_Reset();

				/*	if(API_ECG_Init())
				{

					if(API_ECG_Lead_OFF_Detect(LEAD2))
					{
						API_Disp_Display_Lead_Connection_Status((uint8_t)LEAD2);

						 API_Disp_Quick_test_screen(DISP_QT_TEST_IN_PROGRESS);
						 API_Disp_Quick_Test_Icon();

						bool status = QUICK_Test1();

						if(status == false)
						{

						}

						Disable_Power_Supply();

						return false;
					}
					API_ECG_Chip_Reset();
				}*/
				/*  uint8_t ret;
				float temp;
				 while(1)
				 {
				       Delay_ms(100);
					   writeRegister8(MAX30101_ADDR,0x21,0x01);

					   ret = (readPartID(0x20) & 0x0F);
					   temp = ret * 0.0625;
				       temp = temp + readPartID(0x1F);
				       printf("\n %f",temp);

				}*/

				//uint8_t ret;

				//printf("\n\t\t\t%d",gpio_get_level(25));


			//	 if(API_MAX30101_Setup())
			//	 {
					if((API_ECG_Init()))
						{
						printf("\n\t%d",gpio_get_level(25));
						  uint8_t ret;

						  Detect_low_battery_display_notification();
						  if(IsUSB_Charger_Connected())
						  {
							  return  0;
						  }
						 /* while(1)
						  {
						  printf("\nDevice ID = 0x%2X.\n",readPartID(0x04));
						}*/

						 // while(1)
							//  printf("\n\t%d",gpio_get_level(25));
                              // uint8_t ret;

							//  writeRegister8(MAX30101_ADDR,0x21,0x01);

					      /*while(1)
						  {
							 ret = readRegister8(0x15,reg);
							 if(ret)
							 printf("\n %x",ret);
							 if(API_Push_Btn_Get_Buttton_Press())
					         {
								Disable_Power_Supply();
							    return 0;
							 }

						  }*/


#if 0
						    if(API_Push_Btn_Get_Buttton_Press())
						    {
						    	Disable_Power_Supply();
						    	return 0;
						    }
							API_Disp_Quick_test_screen(DISP_QT_PPG_TEST_IN_PROGRESS);
							if(!(Capture_PPG_ECG_Data(CAPTURE_PPG,TRUE)))
							{
								Disable_Power_Supply();
								return 0;
							}
							API_IO_Exp_Power_Control(EN_VLED,LOW);
							API_IO_Exp_Power_Control(EN_IR,LOW);
#endif
#if 1
							API_Disp_Quick_test_screen(DISP_QT_ECG_TEST_IN_PROGRESS);

							if(qv_flag == 12)
							{
								if(Capture_PPG_ECG_Data(CAPTURE_ECG_L1_AND_L2,TRUE)==FALSE)
								{
									Disable_Power_Supply();
									return FALSE;
								}
							}
							else if(qv_flag == 11)
							{
								if(Capture_PPG_ECG_Data(CAPTURE_ECG_L1,TRUE)==FALSE)
								{
									Disable_Power_Supply();
									return FALSE;
								}
							}
#endif
#if 0
							API_Disp_Quick_test_screen(DISP_QT_BP_TEST_IN_PROGRESS);
							printf("\nCapturing BP................");
							API_IO_Exp_Power_Control(EN_VLED,HIGH);
							//API_IO_Exp_Power_Control(EN_IR,HIGH);
							Print_time("\nBP START");
							if(!(Capture_BP_Data(TRUE)))
							{
								   Disable_Power_Supply();
								   return FALSE;
							}
							Print_time("\nBP END");
							qv_flag == 0;
#endif

							Vital_result.SBP1 = 117; // Need to change later
							Vital_result.DBP1 = 77; // Need to change later

                            if(qv_flag)
                            {
							    API_Disp_Quick_Test_Result();
                            }
							if(Selected_PID_type != GUEST_PID) Store_QuickTest1_Data_To_Flash();
					   }
					else
					{
						 printf("\nECg init failed ....\n");
					}
				/* }
				 else
				 {
					 printf("\nMAx init failed ....\n");
				 }*/
			}

			else
			{
				printf("\nDevice Memory Full... Please sync the data");
				 API_DISP_Memory_Full_Status();
#if 1 //Enabled for Debug purpose to delete all 100 data
				 for(int delete_rec = 1; delete_rec <=100; delete_rec++)
				 {
					 printf("\n Deleting Record = %d",delete_rec);
					 erase_one_record(BP1);
					 erase_one_record(ECG_1_Lead);
					 erase_one_record(ECG_6_Lead);
					 erase_one_record(ECG_12_LEAD);
					 erase_one_record(SPO2);
					 API_Flash_Check_Update_Valid_Record_Status();
				 }
#endif
			}
	}

	else
	{
        printf("\nPlease select the PID");
        API_Disp_Quick_test_screen(DISP_QT_PLEASE_REGISTER_PID);
	}

	API_Buzzer_Sound(SHORT_BEEP);

	Disable_Power_Supply();

  	printf("\nTest completed.\t%d",Is_Device_Paired);

  	if(Is_Device_Paired == BT_DISCONNECTED) // Paired condition
  	{
  		Selected_PID_type = PID_NOT_SELECTED;
  	}

	return true;
}


/*void Dummy_Capture(uint16_t total_samples)
{

    uint16_t raw_data_index;
    esp_err_t status;
    uint32_t max_ecg_no_use; // Not using ecg data from max86150 as hardware will not support
    float value1;
    float value2;
    uint32_t value3;
    uint32_t value4;

	for(raw_data_index=0; raw_data_index<total_samples; raw_data_index++)
	{
		status = API_ECG_Capture_Samples_2Lead(&value1,&value2);

		if(status != ESP_OK) Catch_RunTime_Error(ECG_DATA_CAPTURE_FAIL);

		status = API_MAX86150_Raw_Data_capture(&value3,&value4,&max_ecg_no_use,1,0,0);

		if(status != ESP_OK) Catch_RunTime_Error(PPG_DATA_CAPTURE_FAIL);
	}

}*/

 bool Capture_PPG_ECG_Data(DATA_CAPTURE_TYPE_t captureType, bool enableDummyCapture)
{
    uint16_t raw_data_index;
    esp_err_t status=ESP_FAIL;
    uint16_t capture_number;

    uint32_t max_ecg_no_use[1]; // Not using ecg data from max86150 as hardware will not support

    bool dataQuality = false;
	//API_IO_Exp2_P1_write_pin(DC_LEAD_OFF,LOW);
	//API_IO_Exp1_P1_write_pin(DC_LEAD_OFF_V,LOW);

	API_IO_Exp1_P1_write_pin(DISPLAY_CSN,HIGH);

	API_IO_Exp1_P0_write_pin(EFM_DISP_EN2,LOW);
	if(captureType == CAPTURE_ECG_L1)
	{
		printf("\nECG L1 Test in Progress............\n");
		API_ECG_Start_Conversion();

		API_Disp_Exit_Text();
		if(enableDummyCapture)
		{
			if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
			{
				return  0;
			}
			for(raw_data_index=0; raw_data_index<300; raw_data_index++)// ~3sec dummy capture
			{
				status = API_ECG_Capture_Samples_2Lead(ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
			}
		}

		MemSet(ECG_Lead1_buff,0,sizeof(ECG_Lead1_buff));
		MemSet(ECG_Lead2_buff,0,sizeof(ECG_Lead2_buff));
		Print_time("/ECG start");
		for(raw_data_index=0; raw_data_index<600; raw_data_index++)
			{
				if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
				{
					return  0;
				}
				status = API_ECG_Capture_Samples_2Lead(ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
			}
		Print_time("/ECG END");
		API_Clear_Display(DISP_BOTTOM_SEC,BLUE);
		printf("\nECG L1 data:");
		for(int i=0;i<600;i++)
		{
			printf("\n%f",ECG_Lead1_buff[i]);
		}

		MemSet(ECG_Lead2_buff,0,sizeof(ECG_Lead2_buff)); // Lead1 capture doesn't need Lead2 data.

		API_ECG_Stop_Conversion();

	}

	else if(captureType == CAPTURE_ECG_L2)
	{
		API_ECG_Start_Conversion();


		if(enableDummyCapture)
		{
			for(raw_data_index=0; raw_data_index<100; raw_data_index++)// ~3sec dummy capture
			{
				status = API_ECG_Capture_Samples_2Lead(ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
			}
		}

	MemSet(ECG_Lead1_buff,0,sizeof(ECG_Lead1_buff));
	MemSet(ECG_Lead2_buff,0,sizeof(ECG_Lead2_buff));

	for(raw_data_index=0; raw_data_index<600; raw_data_index++)
		{
			status = API_ECG_Capture_Samples_2Lead(ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
		}
	printf("\nECG L1");

	for(int i=0;i<600;i++)
			{
			  printf("%f\n",ECG_Lead1_buff[i]);
			}
	printf("\nECG L2");
	for(int i=0;i<600;i++)
			{
			  printf("%f\n",ECG_Lead2_buff[i]);
			}

	API_ECG_Stop_Conversion();

	}

	else if(captureType == CAPTURE_PPG)
	{
		printf("\nSPO2 PPG-RED DATA: Capture START");
		Max86150_Clear_Fifo();

#if 0
		if(enableDummyCapture)
		{
			for(raw_data_index=0; raw_data_index<100; raw_data_index++)// 100 dummy capture
			{
				status = API_MAX86150_Raw_Data_capture(SPO2_PPG_RED_BUFF, SPO2_PPG_IR_BUFF,max_ecg_no_use,1,1,0);
			}
		}

		MemSet(SPO2_PPG_RED_BUFF,0,sizeof(SPO2_PPG_RED_BUFF));
		MemSet(SPO2_PPG_IR_BUFF,0,sizeof(SPO2_PPG_IR_BUFF));


		for(raw_data_index=0; raw_data_index<600; raw_data_index++)
			{
				status = API_MAX86150_Raw_Data_capture(SPO2_PPG_RED_BUFF, SPO2_PPG_IR_BUFF,max_ecg_no_use,1,0,0);
			}
#endif
		//uint8_t reg = 0xA5;
#if 1
		int cnt =-1,dly=0,Tcount=0;

		 API_TIMER_Register_Timer(LOD_WAIT_TIME);
		    	  //API_TIMER_Kill_Timer(DATA_SYNC_TIMEOUT);
		 while(1)
		 {
			 dly++;
			 if(dly == 25000)
			 {
			     cnt++;
			     API_MAX30101_Raw_Data_capture_new(SPO2_PPG_RED_BUFF, SPO2_PPG_IR_BUFF,0,0);
			     dly =0;
			     printf("\n %ld",SPO2_PPG_RED_BUFF[cnt]);
			 }
			 if(API_TIMER_Get_Timeout_Flag(LOD_WAIT_TIME))
			 {
				 printf("\ntimer stopped");
				 return 0;
			 }
			 if(SPO2_PPG_RED_BUFF[cnt] > 2000)
			 {
				 SPO2_PPG_RED_BUFF[cnt] = 0;
				Tcount++;
			 }
			 if(Tcount == 40)
			 {
				 break;
			 }
		 }

		API_Disp_Exit_Text();
		if(enableDummyCapture)
		{
			ppg_count = 0;
			do{
				if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
				{
					return  0;
				}
				status = API_MAX30101_Raw_Data_capture_new(SPO2_PPG_RED_BUFF, SPO2_PPG_IR_BUFF,0,1);
			}while(ppg_count < 200);
		}

		MemSet(SPO2_PPG_RED_BUFF,0,sizeof(SPO2_PPG_RED_BUFF));
		MemSet(SPO2_PPG_IR_BUFF,0,sizeof(SPO2_PPG_IR_BUFF));

		ppg_count = 0;
		Print_time("\nSPO2 start");
		do{
			if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
			{
				return  0;
			}
			status = API_MAX30101_Raw_Data_capture_new(SPO2_PPG_RED_BUFF, SPO2_PPG_IR_BUFF,0,0);
		}while(ppg_count < ((ECG_IN_SECONDS*SET_ODR)));
		API_Clear_Display(DISP_BOTTOM_SEC,BLUE);

		Print_time("\nSPO2 END");
		ppg_count = 0;
#endif

		printf("\nSPO2 PPG-RED DATA:");
		for(int i=0;i<((ECG_IN_SECONDS*SET_ODR)) ;i++)
			{
			  printf("\n%ld",SPO2_PPG_RED_BUFF[i]);
			}
		printf("\nSPO2 PPG-IR DATA:");
		for(int i=0;i<((ECG_IN_SECONDS*SET_ODR));i++)
		{
		  printf("\n%ld",SPO2_PPG_IR_BUFF[i]);
		}
#if 0
		printf("\nSPO2 PPG_ECG_BUFF DATA:");
		for(int i=0;i<((ECG_IN_SECONDS*SET_ODR));i++)
		{
		  printf("\n%ld",SPO2_PPG_ECG_BUFF[i]);
		}
#endif
		printf("\nSPO2 PPG-RED DATA: Capture END");
	}

	else if(captureType == CAPTURE_ECG_L1_AND_L2)
		{
		bool leadoffstatus_lead1 = 0, leadoffstatus_lead2 = 0;
		//leadoffstatus_lead1 = API_ECG_Lead_OFF_Detect(LEAD1);
	    //leadoffstatus_lead2 = API_ECG_Lead_OFF_Detect(LEAD2);
		//printf("leadoffstatus_lead1=%d,leadoffstatus_lead2=%d\n",leadoffstatus_lead1,leadoffstatus_lead2);
		if(leadoffstatus_lead1&leadoffstatus_lead1)
		{
			printf("leads are not connected\n");
			API_Disp_Quick_test_screen(DISP_QT_PLACE_FINGER_PROPERLY);
			return false;
		}
		else
		{
			printf("\nECG6 test in progress ..........");
			ECG_Drdy_count = 0;
			API_ECG_Stop_Conversion();
			API_ECG_Start_Conversion();
			API_Disp_Exit_Text();
			if(enableDummyCapture)
			{
				for(raw_data_index=0; raw_data_index<ECG_DUMMY_CAPTURES; raw_data_index++)// ~3sec dummy capture
				{
					if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
					{
						return  0;
					}
				/*	if(API_Push_Btn_Get_Buttton_Press())
					{
						return 0;
					}*/

					status = API_ECG_Capture_Samples_2Lead(ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
				}
			}

			MemSet(ECG_Lead1_buff,0,sizeof(ECG_Lead1_buff));
			MemSet(ECG_Lead2_buff,0,sizeof(ECG_Lead2_buff));
			Print_time("/ECG start");
			for(raw_data_index=0; raw_data_index<(ECG_IN_SECONDS*SET_ODR); raw_data_index++)
			{
				if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
				{
					return  0;
				}
				status = API_ECG_Capture_Samples_2Lead(ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
			}
			Print_time("/ECG END");
			API_Clear_Display(DISP_BOTTOM_SEC,BLUE);

			//printf("\n total data ready interrupts = %d\n", ECG_Drdy_count);
			ECG_Drdy_count = 0;
			API_ECG_Stop_Conversion();
#if 1
			printf("\nECG L1 data:\n");
			for(int i=0;i<(ECG_IN_SECONDS*SET_ODR);i++)
			{
			  printf("\n%f",ECG_Lead1_buff[i]);
			}

			printf("\nECG L2 data:\n");
			for(int i=0;i<(ECG_IN_SECONDS*SET_ODR);i++)
			{
			  printf("\n%f",ECG_Lead2_buff[i]);
			}
#endif
			
		}
		}

	else
	{
		;
	}

	API_IO_Exp1_P0_write_pin(EFM_DISP_EN2,HIGH);
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);

	return (status==ESP_OK);
}

 bool Capture_BP_Data(bool enableDummyCapture)
 {
		uint32_t max_ecg_no_use[1U] ={0U}; // Not using ecg data from max86150 as hardware will not support
		uint16_t raw_data_index =0U;
		esp_err_t status = ESP_FAIL;
#if 0
		API_ECG_Start_Conversion();
		Max86150_Clear_Fifo();

		if(enableDummyCapture)
		{
			for(raw_data_index=0; raw_data_index<600; raw_data_index++)// ~3sec dummy capture
			{
				status = API_ECG_Capture_Samples_2Lead(BP_ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
				status = API_MAX86150_Raw_Data_capture(BP_PPG_RED_BUFF+raw_data_index, BP_PPG_IR_BUFF+raw_data_index,max_ecg_no_use,1,0,0);
			}
		}

		MemSet(BP_ECG_Lead1_buff,0,sizeof(BP_ECG_Lead1_buff));
		MemSet(ECG_Lead2_buff,0,sizeof(ECG_Lead2_buff));
		MemSet(BP_PPG_RED_BUFF,0,sizeof(BP_PPG_RED_BUFF));
		MemSet(BP_PPG_IR_BUFF,0,sizeof(BP_PPG_IR_BUFF));

		for(raw_data_index=0; raw_data_index<(ECG_IN_SECONDS*SET_ODR); raw_data_index++)
			{
				status = API_ECG_Capture_Samples_2Lead(BP_ECG_Lead1_buff + raw_data_index, ECG_Lead2_buff+raw_data_index);
				status = API_MAX86150_Raw_Data_capture(BP_PPG_RED_BUFF+raw_data_index, BP_PPG_IR_BUFF+raw_data_index,max_ecg_no_use,1,0,0);
			}

		API_ECG_Stop_Conversion();
#endif

#if 1
		ECG_Drdy_count = 0;
		uint8_t ppg_index_cnt =0;
		API_ECG_Stop_Conversion();
		float ECG_temp =0 ;
		uint32_t  ECG=0;
		API_ECG_Start_Conversion();
		int cnt =-1,dly=0,Tcount=0;

		API_TIMER_Register_Timer(LOD_WAIT_TIME);
		//API_TIMER_Kill_Timer(DATA_SYNC_TIMEOUT);
		while(1)
		{
			dly++;
			if(dly == 25000)
			{
				cnt++;
				API_MAX30101_Raw_Data_capture_new(SPO2_PPG_RED_BUFF, SPO2_PPG_IR_BUFF,0,0);
				dly =0;
				printf("\n %ld",SPO2_PPG_RED_BUFF[cnt]);
			}
			if(API_TIMER_Get_Timeout_Flag(LOD_WAIT_TIME))
			{
				printf("\ntimer stopped");
				return 0;
			}
			if(SPO2_PPG_RED_BUFF[cnt] > 2000)
			{
				SPO2_PPG_RED_BUFF[cnt] = 0;
				Tcount++;
			}
			if(Tcount == 40)
			{
				break;
			}
		}
		API_Disp_Exit_Text();
		if(enableDummyCapture)
		{
			ppg_count = 0;
			for(raw_data_index=0; raw_data_index<ECG_DUMMY_CAPTURES; raw_data_index++)// ~3sec dummy capture
			{
				if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
				{
					return  0;
				}
				status = API_ECG_Capture_Samples_2Lead(BP_ECG_Lead1_buff + raw_data_index, &ECG_temp);
				status = API_MAX30101_Raw_Data_capture_new(BP_PPG_RED_BUFF, BP_PPG_IR_BUFF,0,0);
			}
		}

		printf("\nReal BP Capture START:\n");
		MemSet(BP_ECG_Lead1_buff,0,sizeof(BP_ECG_Lead1_buff));
		//MemSet(ECG_Lead2_buff,0,sizeof(ECG_Lead2_buff));
		MemSet(BP_PPG_RED_BUFF,0,sizeof(BP_PPG_RED_BUFF));
		MemSet(BP_PPG_IR_BUFF,0,sizeof(BP_PPG_IR_BUFF));
		ppg_count = 0;
		ppg_index_cnt = 0;

		Print_time("\nBP start");
		for(raw_data_index=0; raw_data_index<(ECG_IN_SECONDS*SET_ODR); raw_data_index++)
		{
			if(IsUSB_Charger_Connected() || API_Push_Btn_Get_Buttton_Press())
			{
				return  0;
			}
			status = API_ECG_Capture_Samples_2Lead(BP_ECG_Lead1_buff + raw_data_index, &ECG_temp);
//			ppg_index_cnt++;
//			if(ppg_index_cnt >=10)
			{
				status = API_MAX30101_Raw_Data_capture_new(BP_PPG_RED_BUFF,BP_PPG_IR_BUFF,0,0);
//				ppg_index_cnt = 0;
			}
		}
		API_Clear_Display(DISP_BOTTOM_SEC,BLUE);
		ppg_index_cnt = 0;
		Print_time("\nBP END");

		printf("\n total data ready interrupts = %d\n", ECG_Drdy_count);
		ppg_count = 0;
		ECG_Drdy_count = 0;
		API_ECG_Stop_Conversion();
		printf("\nECG L1 data:\n");
		for(int i=0;i<(ECG_IN_SECONDS*SET_ODR);i++)
		{
		  printf("\n%f",BP_ECG_Lead1_buff[i]);
		}
#if 0
		printf("\nECG L2 data:\n");
		for(int i=0;i<(ECG_IN_SECONDS*SET_ODR);i++)
		{
		  printf("\n%f",ECG_Lead2_buff[i]);
		}
#endif

		printf("\nSPO2 PPG-RED DATA:");
		for(int i=0;i<(ECG_IN_SECONDS*SET_ODR);i++)
		{
		  printf("\n%ld",BP_PPG_RED_BUFF[i]);
		}
		printf("\nSPO2 PPG-IR DATA:");
		for(int i=0;i<(ECG_IN_SECONDS*SET_ODR);i++)
		{
		  printf("\n%ld",BP_PPG_IR_BUFF[i]);
		}
#if 0
		printf("\nSPO2 PPG_ECG_BUFF DATA:");
		for(int i=0;i<((ECG_IN_SECONDS*SET_ODR));i++)
		{
		  printf("\n%ld",SPO2_PPG_ECG_BUFF[i]);
		}
#endif
		printf("\nSPO2 PPG-RED DATA: Capture END");
#endif
		return true;
 }


void Store_QuickTest1_Data_To_Flash(void)
{
	RECORD_OPS_STATUS status;

	uint32_t offfset = 0;
	if(qv_flag == 11 )
	{
		API_Update_Record_Header(ECG_1_Lead,&record_header);


		MemSet(BT_flash_buffer,0,sizeof(BT_flash_buffer));

		MemCpy(BT_flash_buffer,&record_header,REC_HEADER_LEN);
		offfset = REC_HEADER_LEN;

		MemCpy(BT_flash_buffer+offfset,ECG_Lead1_buff,(600*4));
		offfset += 600*4;


		status = API_Flash_Write_Record(ECG_1_Lead,(void*)BT_flash_buffer);

		if(status != WRITE_RECORDS_SUCCESS) Catch_RunTime_Error(ECG_DATA_STORE_TO_FLASH_FAIL);
	}
	else if(qv_flag == 12)
	{
		API_Update_Record_Header(ECG_6_Lead,&record_header);


		MemSet(BT_flash_buffer,0,sizeof(BT_flash_buffer));

		MemCpy(BT_flash_buffer,&record_header,REC_HEADER_LEN);
		offfset = REC_HEADER_LEN;

		MemCpy(BT_flash_buffer+offfset,ECG_Lead1_buff,(600*4));
		offfset += 600*4;

		MemCpy(BT_flash_buffer+offfset,ECG_Lead2_buff,(600*4));
		offfset += 600*4;


		status = API_Flash_Write_Record(ECG_6_Lead,(void*)BT_flash_buffer);

		if(status != WRITE_RECORDS_SUCCESS) Catch_RunTime_Error(ECG_DATA_STORE_TO_FLASH_FAIL);
	}
	else
	{
		API_Update_Record_Header(BP1,&record_header);
		MemCpy(BT_flash_buffer+offfset,&record_header,REC_HEADER_LEN);

		offfset = REC_HEADER_LEN;
		MemCpy(BT_flash_buffer+offfset,BP_PPG_IR_BUFF,(SPO2_RED_SAMPLES*4));
		offfset += 600*4;
		MemCpy(BT_flash_buffer+offfset,BP_ECG_Lead1_buff,(SPO2_RED_SAMPLES*4));
		offfset += 600*4;
		status = API_Flash_Write_Record(BP1,(void*)BT_flash_buffer);

		if(status != WRITE_RECORDS_SUCCESS) Catch_RunTime_Error(BP_DATA_STORE_TO_FLASH_FAIL);


		offfset = 0;
		API_Update_Record_Header(ECG_6_Lead,&record_header);


		MemSet(BT_flash_buffer,0,sizeof(BT_flash_buffer));

		MemCpy(BT_flash_buffer,&record_header,REC_HEADER_LEN);
		offfset = REC_HEADER_LEN;

		MemCpy(BT_flash_buffer+offfset,ECG_Lead1_buff,(600*4));
		offfset += 600*4;

		MemCpy(BT_flash_buffer+offfset,ECG_Lead2_buff,(600*4));
		offfset += 600*4;


		status = API_Flash_Write_Record(ECG_6_Lead,(void*)BT_flash_buffer);

		if(status != WRITE_RECORDS_SUCCESS) Catch_RunTime_Error(ECG_DATA_STORE_TO_FLASH_FAIL);

		offfset = 0;
		MemSet(BT_flash_buffer,0,sizeof(BT_flash_buffer));

		API_Update_Record_Header(SPO2,&record_header);

		MemCpy(BT_flash_buffer,&record_header,REC_HEADER_LEN);
		offfset = REC_HEADER_LEN;

		MemCpy(BT_flash_buffer+offfset,SPO2_PPG_RED_BUFF,(SPO2_RED_SAMPLES*4));

		offfset += SPO2_RED_SAMPLES*4;
		MemCpy(BT_flash_buffer+offfset,SPO2_PPG_IR_BUFF,(SPO2_IR_SAMPLES*4));

		status = API_Flash_Write_Record(SPO2,(void*)BT_flash_buffer);

		if(status != WRITE_RECORDS_SUCCESS) Catch_RunTime_Error(SPO2_DATA_STORE_TO_FLASH_FAIL);
	}
	IsValidRecordsInFlash = true;

	printf("\nTotal SPO2 Records = %ld or %ld", get_records_count(SPO2),get_records_count(SPO2));
	printf("\nTotal BP1 Records = %ld or %ld", get_records_count(BP1),get_records_count(BP1));
	printf("\nTotal ECG L1 Records = %ld or %ld", get_records_count(ECG_1_Lead),get_records_count(ECG_1_Lead));
	printf("\nTotal ECG6 Records = %ld or %ld", get_records_count(ECG_6_Lead),get_records_count(ECG_6_Lead));

}




bool Run_Multi_Vital(void)
{
	API_DISP_Display_Screen(DISP_ECG_12_LEAD_SCREEN);

	Enable_Power_Supply();

#if 0

	if(1)
	//if(Lead12_LeadOff_Detect() == FALSE)
	{
		if(Run_Quick_Vital()==TRUE)
		{
			//if(Selected_PID_type != GUEST_PID)
				Store_QuickTest2_Data_To_Flash();

			Lead12_Test(); // This is the core function to capture 12Lead ECG


		}
	}

	else
	{
		API_Clear_Display (DISP_MIDDLE_SEC ,WHITE);
		API_Disp_Quick_test_screen(DISP_12LEAD_CABLE_NOT_CONNECTED_PROPERLY);
	}
#endif

#if 1 //only 12 Lead capture not sure why previously done quik Vital
	//if(Lead12_LeadOff_Detect() == FALSE)
	if(1)
	{
		if(!(Run_Quick_Vital()))
		{
			Disable_Power_Supply();
			return 0;
		}
		//printf("lead off not detected\n");
		if(!(Lead12_Test()))
		{
			Disable_Power_Supply();
			return 0; // This is the core function to capture 12Lead ECG
		}
	}
	else
	{
		printf("lead off detected\n");
		API_Clear_Display (DISP_MIDDLE_SEC ,WHITE);
		API_Disp_Quick_test_screen(DISP_12LEAD_CABLE_NOT_CONNECTED_PROPERLY);
		return false;
	}
#endif
	Disable_Power_Supply();
	return true;
}

bool Lead12_LeadOff_Detect(void)
{
	uint8_t count=0;

	bool leadOffStatus = true;

	API_ECG_Chip_Reset();

	if(API_ECG_Init())
	{
		for(uint8_t vlead=1;vlead<=6;vlead++)
		{
			Select_Vlead(vlead);
			if(API_ECG_Lead_OFF_Detect(LEAD12))
			{
				count++;
				printf("\nError Lead Off : vlead%d",vlead);
			}
		}
	}

	if(count == 0) leadOffStatus = false;

	return leadOffStatus;
}


void Enable_Power_Supply(void)
{
	API_IO_Exp_Power_Control(EN_VLED,HIGH);
	API_IO_Exp_Power_Control(EN_ANALOG,HIGH);
	API_IO_Exp_Power_Control(EN_IR,HIGH);
	Delay_ms(500);
}


void Disable_Power_Supply(void)
{
	API_IO_Exp_Power_Control(EN_VLED,LOW);
	API_IO_Exp_Power_Control(EN_ANALOG,LOW);
	API_IO_Exp_Power_Control(EN_IR,LOW);
	Delay_ms(500);

}


