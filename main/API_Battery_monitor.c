#include "API_Battery_monitor.h"
#include "API_Display.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "API_timer.h"
#include "Error_Handling.h"
#include "API_IO_Exp.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   10          //Multisampling

static const adc_channel_t channel = ADC_CHANNEL_7;     //GPIO35 if ADC1
static const adc_atten_t atten = ADC_ATTEN_DB_0;//;ADC_ATTEN_DB_0;
//static const adc_unit_t unit = ADC_UNIT_1;



bool  API_Battery_monitor_Init(void)
{
	esp_err_t error;

	error = adc1_config_width(ADC_WIDTH_BIT_12);
	error |= adc1_config_channel_atten(channel, atten);
	//  adc2_config_width(ADC_WIDTH_BIT_12);
	error |= adc2_config_channel_atten((adc2_channel_t)channel, atten);

	if(error != ESP_OK) Catch_RunTime_Error(BATTERY_MONITOR_INIT_FAIL);

	return true;
}


float API_ADC_Read_Battery_Voltage(void)
{
		int   adc_raw_data = 0;
		float battery_vol  = 0.0;
		float temp = 0;

		for(int i=0;i<100;i++)
			{
				adc_raw_data = adc1_get_raw((adc1_channel_t)channel);
                 //printf("\n%d",adc_raw_data);
				temp = (adc_raw_data*1.1)/4095;
				battery_vol += temp;
				for(int i=0;i<5000;i++){}// Software delay
			}

		//printf("\nBattery Voltage = %f",(battery_vol/3));
	return ((battery_vol/100)*4.1);
}

void Fuel_Guage_update_battery_status(float batVoltage)
{

		if(batVoltage < FUEL_GUAGE_10_PER_VOLTAGE)
		{
			API_DISP_Update_Battery_Status(DISP_ONE_STICK,false,true,RED);
		}

		else if((batVoltage >= FUEL_GUAGE_10_PER_VOLTAGE) && (batVoltage <= FUEL_GUAGE_33_PER_VOLTAGE))
		{
			API_DISP_Update_Battery_Status(DISP_ONE_STICK,false,false,WHITE);
		}

		else if(IsUSB_Charger_Connected())
		{
			API_DISP_Update_Battery_Status(DISP_CHARGE_STICK,false,false,WHITE);
		}

		else if((batVoltage > FUEL_GUAGE_33_PER_VOLTAGE) && (batVoltage <= FUEL_GUAGE_66_PER_VOLTAGE))
		{
			API_DISP_Update_Battery_Status(DISP_TWO_STICK,false,false,WHITE);
		}

		else if(batVoltage > FUEL_GUAGE_66_PER_VOLTAGE)
		{
			API_DISP_Update_Battery_Status(DISP_THREE_STICK,false,false,WHITE);
		}

	}

void Fuel_Guage_turn_off_notification_led(void)
{
	//

}
