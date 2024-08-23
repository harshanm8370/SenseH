/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "API_adc.h"

#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "API_IO_Exp.h"
#include "API_timer.h"
#include "API_utility.h"
#include<math.h>
#include "API_Display.h"
#include "Quick_Test.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   10          //Multisampling

/*******TEMPERATURE CONSTANTS START********/


const float a = -412.6;
const float b = 140.41;
const float c = 0.00764;
const float d = -0.0000000000000000625;
const float e = -0.00000000000000000000000115;

extern float ECG_Lead1_buff[TOTAL_SAMPLES_VCS];
extern float ECG_Lead2_buff[TOTAL_SAMPLES_VCS];


// constants for the thermopile calculation
const float k = 0.004313;
const float delta = 2.468;

const float reftemp = 25;
const float shiftv = 0.5;
const float verr = 0.5;


//int count1 = 0,count2 = 0;


/********TEMPERATURE CONSTANTS END *******/

#if CONFIG_IDF_TARGET_ESP32
static esp_adc_cal_characteristics_t *adc_chars;
#elif CONFIG_IDF_TARGET_ESP32S2BETA
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_0;//;ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

#if CONFIG_IDF_TARGET_ESP32
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
#endif

void  API_IR_ADC_Init(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
#endif

    //Configure ADC
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC_CHANNEL_1, atten);
        adc1_config_channel_atten(ADC_CHANNEL_2, atten);

      //  adc_power_on();
      //  adc2_config_width(ADC_WIDTH_BIT_12);
        //adc2_config_channel_atten((adc2_channel_t)channel, atten);

#if CONFIG_IDF_TARGET_ESP32
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
#endif

}

float API_IR_Data_Read(float data_object[], uint16_t data_ambient[],bool enable_adc2_read,uint16_t length_to_read)
{
	float temp=0;
	//Multisampling
	for (int i = 0; i < length_to_read; i++)
	{
		data_object[i] = adc1_get_raw(ADC_CHANNEL_1);

		temp += data_object[i];
		//printf("\n %f",temp);
		//
		//            	if(enable_adc2_read){
		//                //adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, data_ambient+i);
		//            		data_ambient[i] = adc1_get_raw(ADC_CHANNEL_2);

	}
	temp /= 25;
	return temp;
}


void API_ADC_Conv_Avg(float ref_vol, uint16_t data_buff[],uint8_t avg_len,char *tag)
{
	uint32_t avg = 0;
	float voltage = 0.0;

	 for(int i=0;i<avg_len;i++)
	 {
		 avg +=  data_buff[i];
	 }

	 avg = avg/avg_len;
       // float voltage = adc_reading*1.1/4095;

	    voltage = avg*ref_vol/4095;
        printf("%s, Raw: %ld\tVoltage: %f\n", tag, avg, voltage);
}

void API_RUN_TEMPERATURE_TEST(void)
{
	API_IO_Exp_Power_Control(EN_IR,HIGH);
	float obj_data=0;
	float amb_data = 0;
	
	printf("\n\nIR Data Capture 10 SPS");
	float temp_data[25] ;
	char str1[10];
	int i = 256;
	//float arr[i];
	//int adc[i];
	printf("temperature test in progress\n");
//	 printf("\n\nthermopile   obj temp    thermistor     amp temp\n");
	// float thermo_pile[50], thermister[50],obj_temp[50],amb_temp[50];
	while(i)
	{
#if 1



		//	 ECG_Lead1_buff[TOTAL_SAMPLES_VCS] = {183.88, 180.77, 182.03, 188.85, 187.48, 188.65, 188.26, 188.07, 188.65, 180.59, 188.65, 189.84, 189.84, 189.64, 189.24, 189.84, 189.05, 190.23, 189.24, 183.32, 190.03, 188.46, 189.05, 190.63, 183.88, 182.95, 189.84, 189.84, 191.84, 190.63, 191.24, 185.57, 192.86, 192.25, 186.33, 192.05, 186.9, 191.44, 184.81, 192.25, 192.86, 192.66, 193.48, 193.48, 192.05, 192.86, 193.89, 192.25, 185.76, 192.86, 192.86, 192.86, 192.86, 191.44, 193.48, 194.31, 190.03, 191.84, 193.48, 194.73, 193.69, 188.46, 195.57, 191.04, 187.48, 195.78, 195.15, 188.65, 194.94, 187.87, 195.99, 195.78, 195.57, 188.46, 194.94, 187.48, 189.44, 195.15, 189.05, 196.2, 194.94, 196.2, 196.41, 196.41, 195.78, 196.63, 189.44, 196.84, 196.2, 193.07, 196.2, 196.84, 196.84, 196.84, 189.44, 197.7, 191.24, 197.27, 196.2, 195.57, 197.7, 196.84, 197.06, 197.48, 196.2, 198.57, 198.13, 193.69, 191.84, 199, 200.32, 198.57, 192.86, 192.66, 191.84, 192.86, 190.83, 199.22, 194.94, 193.07, 199.22, 199, 200.77, 199, 199.22, 199.44, 196.2, 199.22, 199.44, 200.32, 197.92, 198.35, 199.66, 192.86, 200.1, 199.66, 198.13, 194.73, 200.32, 200.77, 200.99, 200.32, 193.27, 200.99, 194.1, 200.54, 199};
		//	 ECG_Lead2_buff[TOTAL_SAMPLES_VCS] = {19.09, 21.45, 21.47, 21.53, 21.54, 21.72, 21.72, 21.81, 22.07, 22.19, 22.67, 22.74, 22.74, 23.08, 23.5, 23.51, 23.76, 23.88, 24.11, 24.21, 24.25, 24.6, 24.86, 25.03, 25.1, 25.15, 25.47, 25.66, 26.21, 26.63, 26.8, 27.12, 27.28, 27.35, 27.39, 27.4, 27.44, 27.5, 27.6, 27.79, 27.98, 28.04, 28.09, 28.09, 28.13, 28.29, 28.42, 28.54, 28.75, 28.76, 28.94, 28.97, 28.97, 28.97, 29.26, 29.31, 29.39, 29.51, 31.27, 31.48, 31.73, 31.74, 31.86, 31.95, 31.95, 32.07, 32.07, 32.09, 32.21, 32.25, 32.43, 32.44, 32.44, 32.6, 32.65, 32.67, 32.73, 32.93, 33.02, 33.14, 33.35, 33.39, 33.45, 33.54, 33.56, 33.6, 33.63, 33.73, 33.81, 33.83, 33.9, 33.91, 34.03, 34.12, 34.26, 34.46, 34.52, 34.59, 34.67, 34.73, 34.74, 34.96, 35.13, 35.32, 35.41, 35.51, 35.7, 35.72, 35.78, 36.08, 36.28, 36.31, 36.42, 36.43, 36.68, 36.91, 36.94, 37.03, 37.09, 37.11, 37.16, 37.23, 37.28, 37.51, 37.6, 37.72, 37.83, 37.86, 37.99, 38.03, 38.05, 38.07, 38.27, 38.52, 38.56, 38.78, 38.79, 38.79, 38.88, 38.92, 39.27, 39.27, 39.27, 39.36, 39.38, 39.62, 39.9};



		float thermopileValue = 0.00; // adc1_get_raw(ADC_CHANNEL_1);
		float thermistorValue = 0.00; // adc1_get_raw(ADC_CHANNEL_2);



		thermopileValue = adc1_get_raw(ADC_CHANNEL_1);
		thermistorValue = adc1_get_raw(ADC_CHANNEL_2);


		//	 printf("ADC = %.2f\n",thermopileValue);
		ECG_Lead1_buff[i] = thermopileValue;
		float v1 = (thermistorValue / 1024) * 3.3; // source voltage is 3.3v so reading is fraction of that


		float r = (v1*1000)/(v1-3.3); // to get the resistance

		float ambtemp = a + b * sqrt(1+c*r) + d*pow(r,3.3) + e*pow(r,7); // ambient temp
		//	 printf("Ambient tem = %.2f\n",ambtemp);

		float comp = k * (pow(ambtemp,4-delta)-pow(reftemp,4-delta));  // equivalent thermopile V for amb temp

		// calculate the thermopile temp
		float v2 = (thermopileValue / 1024) * 3.3 + comp - verr - shiftv; // thermopile voltage

		float objtemp = pow((v2+k*pow(ambtemp,4-delta))/k, 1/(4-delta)); // object temp
		objtemp -= 273.15;

		if (objtemp > 16 && objtemp <= 50){
			printf("Actual temp : %.2f\n",objtemp);
			ECG_Lead2_buff[i] = objtemp;
			//	 count1++;
			//	 printf("count = %d\n",count1);
		}
		//	 Actual temp : 36.47
		//	 count = 12
		else if(objtemp > 51){
			printf("temperature is high\n");
			ECG_Lead2_buff[i] = 0;
			//	 printf("Temperature is : %.2f\n",objtemp);
			// count2++;
			//	 printf("count2 = %d\n",count2);
		}
		else
		{
			// TODO
			ECG_Lead2_buff[i] = 0;
		}




#endif
#if 0

		//	obj /= 801;
		// 	amb /= 801;
		char str1[8];
		char str2[8];
		//	char str3[8];
		sprintf(str1, "%.2f", objtemp);
		sprintf(str2, "%.2f",ambtemp);
		//	sprintf(str3, "%.2f",actualTemp);
		printf("\nobj str is : %s\n",str1);
		printf("\namb str is : %s\n",str2);
		struct DISPLAY_TEXT text1,text2,text3,text4,text5,text6,text7;


		text1.text_starting_addr = " OBJ ";
		text2.text_starting_addr = str1;
		text3.text_starting_addr = " AMB ";
		text4.text_starting_addr =str2;
		text5.text_status = 0;
		text6.text_status = 0;
		text7.text_status = 0;
		text1.text_status = display;
		text2.text_status = display;
		text3.text_status = display;
		text4.text_status = display;
		//	API_Clear_Display (DISP_MIDDLE_SEC ,WHITE);
		API_Disp_Display_Text(text1,text2,text3,text4, text5,text6,text7);
#endif
	//	Delay_ms(1000);
		if(--i == 0)
				break;
	}
	printf("%s\t%s\n","ADC VALUE","OBJ TEMP");
	for(int i = 256;i>=0;i--)
	{
		if(ECG_Lead2_buff[i])
	printf("%.2f\n",ECG_Lead2_buff[i]);
	}

}
