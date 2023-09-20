
#ifndef API_API_ECG_H_
#define API_API_ECG_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"

typedef enum
{
	ECG_NO_ERROR = 0,
	ECG_INIT_ERROR,
	ECG_CHIP_ERROR,
	BAT_LOW,
	LEAD_OFF,
	ECG_DEINIT_ERROR,
	EXIT_REQUESTED,
}ECG_STATUS;

typedef enum
{
	LEAD1=1,
	LEAD2,
	LEAD12=12,
}ECG_LEADS_t;

extern volatile bool ECG_Drdy_Flag;

void API_ADS_Test(void);


void API_ECG_Power_up(void);																			// Power up ecg chip
void API_ECG_Power_down(void);
bool API_ECG_Init(void);
void API_ECG_Chip_Reset(void);
ECG_STATUS API_ECG_Deinit(void);																		// De initialize the spi
ECG_STATUS API_ECG_Reginit_1Lead(void);																	// register settings for single lead
ECG_STATUS API_ECG_Reginit_2Lead(void);																	// register settings for 3 lead
ECG_STATUS API_ECG_Reginit_12Lead();
ECG_STATUS API_ECG_Reginit_Testsetup();																	// setup for test signal
ECG_STATUS API_ECG_Check_Error_Status(void);															// checking for error status
ECG_STATUS IRAM_ATTR API_ECG_Capture_Samples_2Lead(float *buff_lead_1, float *buff_lead_2);								// capture samples for 3 lead
bool API_ECG_Capture_Samples_1Lead(float *buff_lead_1);													// capture sample for single lead
bool API_ECG_Capture_Samples_VLead(float *vlead,uint16_t nbf_samples);
bool API_ECG_Diagnostic_Test(void);

void API_ECG_ADS_Capture_Data(uint32_t nbd_samples); // 600 samples L1, 600 samples L2

uint8_t API_Test_ADS(void);

void API_RUN_ECG_LEAD2_TEST(void);
void API_RUN_ECG_LEAD6_TEST(void);

void API_ECG_Enable_LeadOff_Detection(void);
void API_ECG_Disable_LeadOff_Detection(void);

bool API_ECG_Lead_OFF_Detect(ECG_LEADS_t lead);
void API_ECG_Registers_Check_For_Corruption(void);

void API_ECG_Start_Conversion(void);
void API_ECG_Stop_Conversion(void);
ECG_STATUS API_ECG_Chip_Init(void);
extern int ECG_Drdy_count;

#endif /* API_API_ECG_H_ */
