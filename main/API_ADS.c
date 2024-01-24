#include "ecg_reg_config.h"
#include "API_utility.h"
#include "driver/spi_master.h"
#include "stdint.h"
#include "API_ADS.h"
#include "stdlib.h"
#include "string.h"
#include "API_IO_Exp.h"
#include "API_Display.h"
#include "API_timer.h"
#include "driver/gpio.h"
#include "bpf.h"
#include "Hardware.h"
#include "Error_Handling.h"
#include "push_button.h"

#define ECG_DRDY_PIN_SEL  ( 1ULL<<ESP32_MCU_DRDY_PIN )
#define PPG_INTR_PIN_SEL  ( 1ULL<<MAX30101_DRDY_INTR_PIN )


/*-------------------------------------MACROS-------------------------------------------------------------------*/
#define LEAD_ERROR			   		0x08							// to check lead error in error status reg // refer manual //
#define BAT_LOW_ERROR		   		0x04							// to check battery low error
#define NO_OF_BYTES			   		3

#define MASK_HIGHER_BYTE			(uint16_t)0xE000u		// To mask higher byte of temperature raw data
#define MASK_LOWER_BYTE				(uint16_t)0x00FFu		// To mask lower byte of temperature raw data
#define SIGN_BIT_CHECK				(uint16_t)0x0400u		// To check if 11th bit is a sign bit
/*--------------------------------------STRUCT-------------------------------------------------------------------*/

extern  spi_device_handle_t disp_spi;
volatile bool ECG_Drdy_Flag;

typedef struct REG_WRITE
{
	uint8_t address;
	uint8_t data;
}ECG_REG_WRITE;


static uint8_t api_ads_read_data(void);
static void api_ads_write(uint8_t data);

static void api_ads_reg_read(uint8_t addr,uint8_t *data);
static void api_ads_reg_write(uint8_t addr,uint8_t *data);
static bool api_ecg_data_capture_pre_config(void);

static void api_drdy_input_config(void);
static void IRAM_ATTR api_drdy_isr(void* arg);
/*-----------------------------------LOCAL FUNCTIONS--------------------------------------------------------------*/

static ECG_STATUS api_ads_check_Response(void);										// Initialize ECG chip
static ECG_STATUS IRAM_ATTR api_ecg_reg_read(uint8_t addr, uint8_t *buffer);
static ECG_STATUS IRAM_ATTR api_ecg_reg_write(uint8_t addr, uint8_t value);
static bool api_ecg_reg_check(uint8_t addr, uint8_t value);

static ECG_STATUS api_ecg_read_error_status();
static void api_ecg_gpio_init(void);
static void api_ecg_gpio_deinit(void);
static bool api_sensetemp_gpio(void);



/************ Lead OFF Detection *******************************/
/***************************************************************/
static uint8_t GetLeadOffStatus(void);
static void SetDcLeadOffCurrent_in_Steps_8nA(uint16_t currentIn_nA);

/***************************************************************/

/*
 * ECG_STATUS API_ECG_Chip_Init(void)
 *
 * \brief		INIT the ECG chip
 *
 * \retval		ECG_STATUS,ECG_NO_ERROR on successful init
 */


void api_PPG_drdy_input_config()
{
	esp_err_t error;

    gpio_config_t io_conf;

    MemSet(&io_conf,0,sizeof(io_conf));

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;

    io_conf.pin_bit_mask = PPG_INTR_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    error = gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    error |= gpio_set_intr_type(MAX30101_DRDY_INTR_PIN, GPIO_INTR_POSEDGE);

    error |= gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    error |= gpio_isr_handler_add(MAX30101_DRDY_INTR_PIN, api_drdy_isr, (void*) MAX30101_DRDY_INTR_PIN);

}


ECG_STATUS API_ECG_Chip_Init(void)
{
	ECG_STATUS ecg_init = ECG_INIT_ERROR;
	uint8_t rev_id =0;
	uint8_t reg_read = 0;
										// initialize gpio pins
//	  API_IO_Exp1_P1_write_pin(ECG_CSN,LOW);
		gpio_set_level(ECG_CSn_VCS, 0);
	  api_ads_reg_read(0x40 | 0x80, &rev_id);					// 0x40 = address of RevID. R/W bit = 1 for RegRead,=> "C0". rev_id = 0x01, thevalue to be read
//	  API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	  gpio_set_level(ECG_CSn_VCS, 1);

      printf("Rev_id=%x\n",rev_id);

	if((reg_read == ECG_NO_ERROR) && (rev_id == 1))
	{
		ecg_init = ECG_NO_ERROR;
	}

	return ecg_init;

}

static void api_ads_reg_read(uint8_t addr,uint8_t *data)
{
	api_ads_write(addr);
	*data = api_ads_read_data();
}

static void api_ads_reg_write(uint8_t addr,uint8_t *data)
{
	uint8_t write_data = *data;
	    api_ads_write(addr);
	    api_ads_write(write_data);
}

static uint8_t api_ads_read_data(void)
{
    uint8_t dummy = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    memset(&t, 0, sizeof(t));
    t.length=8;
    t.rxlength=8;                 //Len is in bytes, transaction length is in bits.

    t.flags = 4U;
    t.user = (void*)1;
    t.tx_buffer = &dummy;
    esp_err_t ret = spi_device_transmit(disp_spi, &t);
    assert( ret == ESP_OK );


   //return t.rx_data[0];
   return *(uint8_t*)t.rx_data;



}



static void api_ads_write(uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8;
    t.user = (void*)1;
    t.tx_buffer = &data;

    esp_err_t ret = spi_device_transmit(disp_spi, &t);
    assert( ret == ESP_OK );

}

void API_ADS_Test(void)
{
	 static uint8_t rd_data = 0;

	  API_IO_Exp1_P1_write_pin(DISPLAY_CSN,HIGH); // Disable Display

	  API_IO_Exp_Power_Control(EN_VLED,HIGH);
	  API_IO_Exp_Power_Control(EN_ANALOG,HIGH);

//	  API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	  gpio_set_level(ECG_CSn_VCS, 1);
	  Delay_ms(500);
	  API_IO_Exp1_P1_write_pin(ECG_RESETN,LOW);
	  Delay_ms(3000);
	  API_IO_Exp1_P1_write_pin(ECG_RESETN,HIGH);
	  Delay_ms(1500);



	  /*** For Wroom */
	  /*******************************************/

/*
gpio_set_level(ADS_CS, 1);
Delay_ms(500);

gpio_set_level(ADS_RST, 0);
Delay_ms(3000);

gpio_set_level(ADS_RST, 1);
Delay_ms(500);
*/


	  /******************************************/


//	  while(1){
//
//		  //API_ECG_Chip_Init();
//		  API_ADS_Test();
//		  Delay_ms(5);
//
//	  }


	  printf("ADS Test\n");
	while(1)
	{
		// api_ads_write(0x40 | 0x80);
		 api_ads_write(0x2F | 0x80);
		 api_ads_write(0x11);
		 api_ads_write(0x2F | 0x00);

		 rd_data = api_ads_read_data();

		 printf("\n rd_data = %X",rd_data);


    }

}

uint8_t API_Test_ADS()
{
	uint8_t read_data = 0;
	API_IO_Exp_Power_Control(EFM_DISP_RESN,LOW);
	API_IO_Exp_Power_Control(ECG_RESETN,HIGH);

	API_IO_Exp_Power_Control(EN_VLED,HIGH);
	API_IO_Exp_Power_Control(EN_ANALOG,HIGH);
	API_IO_Exp_Power_Control(EN_IR,HIGH);

	API_IO_Exp1_P1_write_pin(DISPLAY_CSN,HIGH);
//	API_IO_Exp1_P1_write_pin(ECG_CSN,LOW);
	gpio_set_level(ECG_CSn_VCS, 0);
	Delay_ms(1);

	API_IO_Exp1_P1_write_pin(ECG_RESETN,LOW);
	Delay_ms(500);
	API_IO_Exp1_P1_write_pin(ECG_RESETN,HIGH);
	Delay_ms(500);

	api_ads_write(0x21);

	api_ads_write(0x01);
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);

	Delay_ms(500);

//	API_IO_Exp1_P1_write_pin(ECG_CSN,LOW);
	gpio_set_level(ECG_CSn_VCS, 0);

	api_ads_write(0x21 | 0x80);
	 read_data = api_ads_read_data();
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);

	printf("ADS Read = %X\n",read_data);

	return read_data;

}


/*****************************************************************************************/

static ECG_STATUS api_ads_check_Response(void)
{
	ECG_STATUS status = ECG_INIT_ERROR;
	uint8_t read_val=0;

	API_IO_Exp1_P1_write_pin(DISPLAY_CSN,HIGH);
//	API_IO_Exp1_P1_write_pin(ECG_CSN,LOW);
	gpio_set_level(ECG_CSn_VCS, 0);
	Delay_ms(1);

	API_IO_Exp1_P1_write_pin(ECG_RESETN,LOW);
	Delay_ms(500);
	API_IO_Exp1_P1_write_pin(ECG_RESETN,HIGH);
	Delay_ms(500);

	api_ads_write(0x21);

	api_ads_write(0x01);
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);

	Delay_ms(50);

//	API_IO_Exp1_P1_write_pin(ECG_CSN,LOW);
	gpio_set_level(ECG_CSn_VCS, 0);
	Delay_ms(50);
	api_ads_write(0x21 | 0x80);
	read_val = api_ads_read_data();
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);


	if(read_val == 0x01)
	{
		status = ECG_NO_ERROR;
	}

	else Catch_RunTime_Error(ADS1293_INIT_FAIL);


	return status;
}



static ECG_STATUS IRAM_ATTR api_ecg_reg_write(uint8_t addr, uint8_t value)
{
	bool write_status = true;

//	API_IO_Exp1_P1_write_pin(ECG_CSN,LOW);
	gpio_set_level(ECG_CSn_VCS, 0);

	api_ads_write(addr);
	api_ads_write(value);

//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);

	write_status = ECG_NO_ERROR;

	return write_status;

}

static bool api_ecg_reg_check(uint8_t addr, uint8_t value)
{
	bool status = false;

   uint8_t regData=0;

    api_ecg_reg_read(addr, &regData);

    if(regData == value)
    {
    	status = true;
    }

    else
    {
    	printf("\nRegAddr=%d  , WriteValue=%d  , ReadValue=%d",addr,value,regData);
    }

    return status;
}
/*
* void api_ecg_read_reg(uint8_t addr, uint8_t *buffer)
* \brief        reading the register
*
* \retval		TRUE on success
*/

ECG_STATUS IRAM_ATTR api_ecg_reg_read(uint8_t addr, uint8_t *buffer)
{
	ECG_STATUS read_status = 0;

	addr |= READ_BIT;

//	API_IO_Exp1_P1_write_pin(ECG_CSN,LOW);
	gpio_set_level(ECG_CSn_VCS, 0);

	api_ads_write(addr);

	*buffer  =  api_ads_read_data();					// read the register value

	gpio_set_level(ECG_CSn_VCS, 1);
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);

	read_status = ECG_NO_ERROR;															// if its 0 read status is true

	return read_status;
}

ECG_STATUS API_ECG_Reginit_2Lead()
{
	ECG_STATUS reg_init = ECG_INIT_ERROR;
	bool reg_config_status = FALSE;

	if(api_ads_check_Response() == ECG_NO_ERROR)
	{
		reg_config_status = api_ecg_reg_write(CONFIG_REG, STOP_CONV);								//Stops data conversion

		/*reg_config_status  |= api_ecg_reg_write(FLEX_CH1_CN_REG, CH1_INP_INN);						//Connect channel 1’s INP to IN2 and INN to IN1.
		reg_config_status |= api_ecg_reg_write(FLEX_CH2_CN_REG, 0x19);								//DisConnect channel 2’s INP and INN for Single lead configuration
		reg_config_status |= api_ecg_reg_write(FLEX_CH3_CN_REG, CH3_INP_INN_DISCNCT);				//DisConnect channel 3’s INP and INN for Single lead configuration
		reg_config_status |= api_ecg_reg_write(FLEX_PACE_CN_REG, PACE_CH_INP_INN_DISCNCT);			//DisConnect pace channel’s INP and INN for Single lead configuration
		reg_config_status |= api_ecg_reg_write(FLEX_VBAT_CN_REG,VBAT_MONITOR_DISABLE);				//Battery voltage monitor disable
		reg_config_status |= api_ecg_reg_write(LOD_CN_REG, DC_LEAD_OFF_CTRL);						//DC_LEAD_OFF_CTRL - active
		reg_config_status |= api_ecg_reg_write(LOD_EN_REG, DC_LEAD_OFF_EN_CH1_CH2_CH3);				//Enable DC lead-off for CH1,CH2,CH3, current to CH1 is anti-phase
		reg_config_status |= api_ecg_reg_write(LOD_CURRENT_REG, LOD_CURRENT_SEL);					//LOD current selected  8nA, which is the minimum current
		reg_config_status |= api_ecg_reg_write(LOD_AC_CN_REG, LOD_AC_CTRL);							//Selected default settings
		reg_config_status |= api_ecg_reg_write(CMDET_EN_REG, CMDET_IN1_IN2_IN3);					//Enable the common-mode detector on input pins IN1 and IN2.
		reg_config_status |= api_ecg_reg_write(CMDET_CN_REG, CMDET_CTRL_LOWBW);						//Selected low BW = 0 and low capacitive drive mode = 00
		reg_config_status |= api_ecg_reg_write(RLD_CN_REG, RLD_IN4);								//RLD output connected to IN4.
		reg_config_status |= api_ecg_reg_write(WILSON_EN1_REG, WILSON_EN1_NO_CONN);					//No Wilson Ref connection to 1st buffer amplifier
		reg_config_status |= api_ecg_reg_write(WILSON_EN2_REG, WILSON_EN2_NO_CONN);					//No Wilson Ref connection to 2nd buffer amplifier
		reg_config_status |= api_ecg_reg_write(WILSON_EN3_REG, WILSON_EN3_NO_CONN);					//No Wilson Ref connection to 3rd buffer amplifier
		reg_config_status |= api_ecg_reg_write(WILSON_CN_REG, WILSON_CN_DISCONN);					//Goldberger ref disabled and Wilson reference output internally disconnected from IN6
		reg_config_status |= api_ecg_reg_write(REF_CN_REG, VREF_CTRL_ON);							//Internal Reference voltage and CM and RLD reference voltage ON
		reg_config_status |= api_ecg_reg_write(OSC_CN_REG, OSC_EXT);								//Use external crystal and feed the internal oscillator's output to the digital.
		reg_config_status |= api_ecg_reg_write(AFE_RES_REG, AFE_FREQ_2kHZ_CH1_CH2_CH3);				//Ckl freq = 204800Hz for CH1,CH2,CH3, High resolution mode = disabled
		reg_config_status |= api_ecg_reg_write(AFE_SHDN_CN_REG, AFE_SHDN_CH1_CH2_CH3_ACTIVE);		//Shuts down unused channel 2's and 3’s signal path.
		reg_config_status |= api_ecg_reg_write(AFE_FAULT_CN_REG, AFE_FAULT_CTRL_ACT_CH1_CH2_CH3);	//Fault detection Active in channel1, channel2 and channel3
		reg_config_status |= api_ecg_reg_write(AFE_PACE_CN_REG, AFE_PACE_CH_SHDN_OP_RLD);			//Analog pace channel shutdown
		reg_config_status |= api_ecg_reg_write(DIGO_STRENGTH_REG, DIGO_STRENGTH_HIGH_DRIVE);		//High digital output drive strength
		reg_config_status |= api_ecg_reg_write(R2_RATE_REG, R2_RATE_8);								//Configures the R2 decimation rate as 8.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, R3_RATE_CH1_128);					//Configures the R3 decimation rate as 128 for channel 1.// ODR = 100sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, R3_RATE_CH1_128);					//Configures the R3 decimation rate as 6 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, R3_RATE_CH1_128);					//Configures the R3 decimation rate as 6 for channel 3.
		reg_config_status |= api_ecg_reg_write(R1_RATE_REG, R1_RATE_CH1_CH2_CH3);					//Standard PACE Data Rate R1 = 2
		reg_config_status |= api_ecg_reg_write(DIS_EFILTER_REG, ECG_FILTER_EN_CH1_CH2_CH3);			//Enable ECG filter for channels 1,2 and 3
		reg_config_status |= api_ecg_reg_write(DRDYB_SRC_REG, DRDYB_SRC_CH1);						//Configures the DRDYB source to channel 1 ECG (or fastest channel).
		reg_config_status |= api_ecg_reg_write(SYNCOUTB_SRC_REG, SYNCOUTB_SRC_DESEL);				//Pin configured as output. No src selected to drive SYNCB
		reg_config_status |= api_ecg_reg_write(MASK_DRDYB_REG, MASK_DRDYB);							//optional mask control for DRDYB
		reg_config_status |= api_ecg_reg_write(MASK_ERR_REG, ALARM_ACTIVE);							//Alarm condition active
		reg_config_status |= api_ecg_reg_write(ALARM_FILTER_REG, ALARM_FILTER);						//default value set
		reg_config_status |= api_ecg_reg_write(CH_CNFG_REG, CH_CNFG_CH1_CH2_CH3);					//Enables channel 1 ECG for loop read-back mode.*/


		reg_config_status |= api_ecg_reg_write(FLEX_CH1_CN_REG, CH1_INP_INN);
		reg_config_status |= api_ecg_reg_write(FLEX_CH2_CN_REG, 0x19);
		reg_config_status |= api_ecg_reg_write(CMDET_EN_REG, CMDET_IN1_IN2_IN3);
		reg_config_status |= api_ecg_reg_write(RLD_CN_REG, RLD_IN5);
		reg_config_status |= api_ecg_reg_write(OSC_CN_REG, OSC_EXT);
		reg_config_status |= api_ecg_reg_write(AFE_SHDN_CN_REG, AFE_SHDN_CH1_CH2_CH3_ACTIVE);
		reg_config_status |= api_ecg_reg_write(R1_RATE_REG, 0x00);
		reg_config_status |= api_ecg_reg_write(R2_RATE_REG, 0x01);//Configures the R2 decimation rate as 4
#if ODR_50
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x80);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 50sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x80);//Configures the R3 decimation rate as 64 for channel 2.
#elif ODR_100
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x40);//40Configures the R3 decimation rate as 64 for channel 1.// ODR = 100sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x40);//40Configures the R3 decimation rate as 64 for channel 2.
#elif ODR_200
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x20);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 200sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x20);//Configures the R3 decimation rate as 64 for channel 2.
#elif ODR_400
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x10);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 400sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x10);//Configures the R3 decimation rate as 64 for channel 2.
#elif ODR_533
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x08);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 533sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x08);//Configures the R3 decimation rate as 64 for channel 2.
#elif ODR_800
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x04);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 800sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x04);//Configures the R3 decimation rate as 64 for channel 2.
#elif ODR_1067
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x02);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 1067sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x02);//Configures the R3 decimation rate as 64 for channel 2.
#elif ODR_1600
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x01);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 1600sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x01);//Configures the R3 decimation rate as 64 for channel 2.
#else

#endif
		reg_config_status |= api_ecg_reg_write(DRDYB_SRC_REG, DRDYB_SRC_CH1);//Configures the DRDYB source to channel 1 ECG (or fastest channel).
		reg_config_status |= api_ecg_reg_write(CH_CNFG_REG, 0X10);//Enables channel 1 ECG for loop read-back mode.
		reg_config_status |= api_ecg_reg_write(AFE_RES_REG, 0x3F);
		reg_config_status |= api_ecg_reg_write(FLEX_CH3_CN_REG, 0X40);

	}

	if (reg_config_status == ECG_NO_ERROR)
	{
		reg_init = ECG_NO_ERROR;
//		API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
		gpio_set_level(ECG_CSn_VCS, 1);
	}

	else Catch_RunTime_Error(ADS1293_INIT_FAIL);



	return reg_init;

}



ECG_STATUS API_ECG_Reginit_12Lead_new()
{
	ECG_STATUS reg_init = ECG_INIT_ERROR;
	bool reg_config_status = FALSE;

	if(api_ads_check_Response() == ECG_NO_ERROR)
	{
		reg_config_status = api_ecg_reg_write(CONFIG_REG, STOP_CONV);								//Stops data conversion
		//Since wilson ref. terminals are enabled & R123 resistor is removed
		reg_config_status |= api_ecg_reg_write(FLEX_CH1_CN_REG, CH1_INP_INN);
		reg_config_status |= api_ecg_reg_write(FLEX_CH2_CN_REG, 0x19);
		reg_config_status |= api_ecg_reg_write(FLEX_CH3_CN_REG, 0x26);
		reg_config_status |= api_ecg_reg_write(CMDET_EN_REG, 0x0F);
		reg_config_status |= api_ecg_reg_write(RLD_CN_REG, RLD_IN5);
		reg_config_status |= api_ecg_reg_write(WILSON_EN1_REG, 0x01);
		reg_config_status |= api_ecg_reg_write(WILSON_EN2_REG, 0x02);
		reg_config_status |= api_ecg_reg_write(WILSON_EN3_REG, 0x03);
		reg_config_status |= api_ecg_reg_write(WILSON_CN_REG, 0x01);
		reg_config_status |= api_ecg_reg_write(OSC_CN_REG, OSC_EXT);
		reg_config_status |= api_ecg_reg_write(AFE_SHDN_CN_REG, AFE_SHDN_CH1_CH2_CH3_ACTIVE);
		reg_config_status |= api_ecg_reg_write(R1_RATE_REG, 0x00);
		reg_config_status |= api_ecg_reg_write(R2_RATE_REG, 0x01);//Configures the R2 decimation rate as 4
#if ODR_50
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x80);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 50sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x80);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x80);
#elif ODR_100
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x40);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 100sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x40);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x40);
#elif ODR_200
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x20);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 200sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x20);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x20);
#elif ODR_400
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x10);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 400sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x10);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x10);
#elif ODR_533
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x08);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 533sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x08);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x08);
#elif ODR_800
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x04);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 800sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x04);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x04);
#elif ODR_1067
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x02);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 1067sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x02);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x02);
#elif ODR_1600
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH1_REG, 0x01);//Configures the R3 decimation rate as 64 for channel 1.// ODR = 1600sps
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH2_REG, 0x01);//Configures the R3 decimation rate as 64 for channel 2.
		reg_config_status |= api_ecg_reg_write(R3_RATE_CH3_REG, 0x01);
#else

#endif
		reg_config_status |= api_ecg_reg_write(DRDYB_SRC_REG, DRDYB_SRC_CH1);
		//reg_write_ads(CN_CNFG, CH_CNFG_CH1_LPRD);//Enables channel 1 ECG for loop read-back mode.
		reg_config_status |= api_ecg_reg_write(AFE_RES_REG, 0x3F);		//Resolution: 100khz
		reg_config_status |= api_ecg_reg_write(CH_CNFG_REG, 0X70);
	}

	if (reg_config_status == ECG_NO_ERROR)
	{
		reg_init = ECG_NO_ERROR;
		gpio_set_level(ECG_CSn_VCS, 0);
	}

	return reg_init;

}

/*ECG_STATUS API_ECG_Reginit_12Lead()
 *
 *  \brief register initialization for 12 lead
 *	\param[in]	void
 *	\param[out] ECG_STATUS
 */
ECG_STATUS API_ECG_Reginit_12Lead()
{
		ECG_STATUS reg_init = ECG_INIT_ERROR;
		bool reg_config_status = FALSE;

		if(api_ads_check_Response() == ECG_NO_ERROR)
			{
			reg_config_status = api_ecg_reg_write(CONFIG_REG, STOP_CONV);								//Stops data conversion

			//Since wilson ref. terminals are Disabled & R123 resistor is present
			/*reg_config_status &= api_ecg_reg_write(FLEX_CH1_CN_REG, CH1_INP_INN);						//Connect channel 1’s INP to IN2 and INN to IN1.
			reg_config_status &= api_ecg_reg_write(FLEX_CH2_CN_REG, 0x19);				//DisConnect channel 2’s INP and INN for Single lead configuration
			reg_config_status &= api_ecg_reg_write(FLEX_CH3_CN_REG, 0x26);				//positive connected to V-inp and negative connected to IN6 (which is connected to WCT)
			reg_config_status &= api_ecg_reg_write(FLEX_PACE_CN_REG, PACE_CH_INP_INN_DISCNCT);			//DisConnect pace channel’s INP and INN for Single lead configuration
			reg_config_status &= api_ecg_reg_write(FLEX_VBAT_CN_REG,VBAT_MONITOR_DISABLE);				//Battery voltage monitor disable
			reg_config_status &= api_ecg_reg_write(LOD_CN_REG, 0x00);						//Lead-off detection circuitry is shut down
			reg_config_status &= api_ecg_reg_write(LOD_EN_REG, 0x0F);							//enable DC lead-off
			reg_config_status &= api_ecg_reg_write(LOD_CURRENT_REG, 0xFF);					//LOD current selected  8nA, which is the minimum current
			reg_config_status &= api_ecg_reg_write(LOD_AC_CN_REG, LOD_AC_CTRL);							//Selected default settings
			reg_config_status &= api_ecg_reg_write(CMDET_EN_REG, 0x07);						//Enable the common-mode detector on input pins IN1 and IN2.
			reg_config_status &= api_ecg_reg_write(CMDET_CN_REG, 0x00);						//Selected low BW = 0 and low capacitive drive mode = 00
			reg_config_status &= api_ecg_reg_write(RLD_CN_REG, 0x05);		//0x04						//DisConnect the output of the RLD amplifier for Single Lead ECG configuration.
			reg_config_status &= api_ecg_reg_write(WILSON_EN1_REG, ECG_12_LEAD_WILSON_EN1_NO_CONN);					//No Wilson Ref connection to 1st buffer amplifier
			reg_config_status &= api_ecg_reg_write(WILSON_EN2_REG, ECG_12_LEAD_WILSON_EN2_NO_CONN);					//No Wilson Ref connection to 2nd buffer amplifier
			reg_config_status &= api_ecg_reg_write(WILSON_EN3_REG, ECG_12_LEAD_WILSON_EN3_NO_CONN);					//No Wilson Ref connection to 3rd buffer amplifier
			reg_config_status &= api_ecg_reg_write(WILSON_CN_REG, ECG_12_LEAD_WILSON_CN_DISCONN);					//Goldberger ref disabled and Wilson reference output internally disconnected from IN6
			reg_config_status &= api_ecg_reg_write(REF_CN_REG, VREF_CTRL_ON);							//Internal Reference voltage and CM and RLD reference voltage ON
			reg_config_status &= api_ecg_reg_write(OSC_CN_REG, OSC_EXT);								//Use external crystal and feed the internal oscillator's output to the digital.
			reg_config_status &= api_ecg_reg_write(AFE_RES_REG, 0x38);							//Ckl freq = 102400Hz, High resolution mode = disabled
			reg_config_status &= api_ecg_reg_write(AFE_SHDN_CN_REG, 0x00);					//Shuts down unused channel 2's and 3’s signal path.
			reg_config_status &= api_ecg_reg_write(AFE_FAULT_CN_REG, 0x00);	//Fault detection Active in channel1, channel2 and channel3
			reg_config_status &= api_ecg_reg_write(AFE_DITHER_EN_REG, AFE_DITHER_EN);					// Enable Dithering in Sigma-Delta- this is not present in datasheet
			reg_config_status &= api_ecg_reg_write(AFE_PACE_CN_REG, 0x05);				//Analog pace channel shutdown
			reg_config_status &= api_ecg_reg_write(DIGO_STRENGTH_REG, DIGO_STRENGTH_HIGH_DRIVE);		//High digital output drive strength
			reg_config_status &= api_ecg_reg_write(R2_RATE_REG, R2_RATE_8);								//Configures the R2 decimation rate as 8.
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH1_REG, R3_RATE_CH1_128);					//Configures the R3 decimation rate as 128 for channel 1.// ODR = 100sps
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH2_REG, R3_RATE_CH1_128);						//Configures the R3 decimation rate as 6 for channel 2.
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH3_REG, R3_RATE_CH1_128);						//Configures the R3 decimation rate as 6 for channel 3.
			reg_config_status &= api_ecg_reg_write(R1_RATE_REG, R1_RATE_CH1_CH2_CH3);				//Standard PACE Data Rate R1 = 2
			reg_config_status &= api_ecg_reg_write(DIS_EFILTER_REG, ECG_FILTER_EN_CH1_CH2_CH3);			//Enable ECG filter for channels 1,2 and 3
			reg_config_status &= api_ecg_reg_write(DRDYB_SRC_REG, DRDYB_SRC_CH1);						//Configures the DRDYB source to channel 1 ECG (or fastest channel).
			reg_config_status &= api_ecg_reg_write(SYNCOUTB_SRC_REG, SYNCOUTB_SRC_DESEL);				//Pin configured as output. No src selected to drive SYNCB
			reg_config_status &= api_ecg_reg_write(MASK_DRDYB_REG, MASK_DRDYB);							//optional mask control for DRDYB
			reg_config_status &= api_ecg_reg_write(MASK_ERR_REG, ALARM_ACTIVE);							//Alarm condition active
			reg_config_status &= api_ecg_reg_write(ALARM_FILTER_REG, ALARM_FILTER);						//default value set
			reg_config_status &= api_ecg_reg_write(CH_CNFG_REG, 0x70);	//0x30						//Enables channel 1 ECG for loop read-back mode.*/

			//Since wilson ref. terminals are enabled & R123 resistor is removed
			reg_config_status &= api_ecg_reg_write(FLEX_CH1_CN_REG, CH1_INP_INN);
			reg_config_status &= api_ecg_reg_write(FLEX_CH2_CN_REG, 0x19);
			reg_config_status &= api_ecg_reg_write(FLEX_CH3_CN_REG, 0x26);
			reg_config_status &= api_ecg_reg_write(CMDET_EN_REG, 0x0F);
			reg_config_status &= api_ecg_reg_write(RLD_CN_REG, RLD_IN5);
			reg_config_status &= api_ecg_reg_write(WILSON_EN1_REG, 0x01);
			reg_config_status &= api_ecg_reg_write(WILSON_EN2_REG, 0x02);
			reg_config_status &= api_ecg_reg_write(WILSON_EN3_REG, 0x03);
			reg_config_status &= api_ecg_reg_write(WILSON_CN_REG, 0x01);
			reg_config_status &= api_ecg_reg_write(OSC_CN_REG, OSC_EXT);
			reg_config_status &= api_ecg_reg_write(AFE_SHDN_CN_REG, AFE_SHDN_CH1_CH2_CH3_ACTIVE);
			reg_config_status &= api_ecg_reg_write(R2_RATE_REG, 0x02);
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH1_REG, 0x08);
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH2_REG, 0x08);
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH3_REG, 0x08);
			reg_config_status &= api_ecg_reg_write(DRDYB_SRC_REG, DRDYB_SRC_CH1);
			//reg_write_ads(CN_CNFG, CH_CNFG_CH1_LPRD);//Enables channel 1 ECG for loop read-back mode.
			reg_config_status &= api_ecg_reg_write(AFE_RES_REG, 0x01);		//Resolution: 100khz
			reg_config_status |= api_ecg_reg_write(CONFIG_REG, START_CONV);			//Starts data conversion

			//Since wilson ref. terminals are disabled & R123 resistor is given
			/*reg_config_status &= api_ecg_reg_write(FLEX_CH1_CN_REG, CH1_INP_INN);
			reg_config_status &= api_ecg_reg_write(FLEX_CH2_CN_REG, 0x19);
			reg_config_status &= api_ecg_reg_write(FLEX_CH3_CN_REG, 0x26);
			reg_config_status &= api_ecg_reg_write(CMDET_EN_REG, 0x0F);
			reg_config_status &= api_ecg_reg_write(RLD_CN_REG, RLD_IN5);
			reg_config_status &= api_ecg_reg_write(WILSON_EN1_REG, 0x00);
			reg_config_status &= api_ecg_reg_write(WILSON_EN2_REG, 0x00);
			reg_config_status &= api_ecg_reg_write(WILSON_EN3_REG, 0x00);
			reg_config_status &= api_ecg_reg_write(WILSON_CN_REG, 0x00);
			reg_config_status &= api_ecg_reg_write(OSC_CN_REG, OSC_EXT);
			reg_config_status &= api_ecg_reg_write(AFE_SHDN_CN_REG, AFE_SHDN_CH1_CH2_CH3_ACTIVE);
			reg_config_status &= api_ecg_reg_write(R2_RATE_REG, 0x02);
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH1_REG, 0x40);
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH2_REG, 0x40);
			reg_config_status &= api_ecg_reg_write(R3_RATE_CH3_REG, 0x40);
			reg_config_status &= api_ecg_reg_write(DRDYB_SRC_REG, DRDYB_SRC_CH1);
			//reg_write_ads(CN_CNFG, CH_CNFG_CH1_LPRD);//Enables channel 1 ECG for loop read-back mode.
			reg_config_status &= api_ecg_reg_write(AFE_RES_REG, 0x01);		//Resolution: 100khz
			reg_config_status |= api_ecg_reg_write(CONFIG_REG, START_CONV);			//Starts data conversion*/



			}

		if (reg_config_status == ECG_NO_ERROR)
		{
			reg_init = ECG_NO_ERROR;
//			API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);							// make CS pin low
			gpio_set_level(ECG_CSn_VCS, 0);
		}

		return reg_init;
}

/*
  * ECG V LEAD CAPTURE
  */

bool API_ECG_Capture_Samples_VLead(float *vlead,uint16_t nbf_samples)
{
		bool read_status = TRUE;
		uint8_t sample[3] = {0};

		while(!ECG_Drdy_Flag);
		ECG_Drdy_Flag = false;

		read_status = api_ecg_reg_read(DATA_CH2_ECG_H_REG,&sample[0]);				// reading the CH3 data- higher byte
		read_status &= api_ecg_reg_read(DATA_CH2_ECG_M_REG,&sample[1]);				// reading the CH3 data- middle byte
		read_status &= api_ecg_reg_read(DATA_CH2_ECG_L_REG,&sample[2]);				// reading the CH3 data- lower byte

		*vlead= ((((uint32_t)sample[0])<<16) | (((uint32_t)sample[1])<<8) | (sample[2]) );

	return read_status;
}

ECG_STATUS IRAM_ATTR API_ECG_Capture_Samples_3Lead(float *buff_lead_1, float *buff_lead_2, float *buff_lead_3)
{
	ECG_STATUS read_status = 0;
	uint8_t sample[10] = {0};
	sample[9U] = 0x00;

		while(!ECG_Drdy_Flag);	//|| (API_TIMER_Get_Timeout_Flag(Set_1sec_timer) == FALSE)
//	if(ESP32_MCU_DRDY_PIN)
	{
		read_status = api_ecg_reg_read(DATA_CH1_ECG_H_REG,&sample[0U]);				// reading the CH1 data- higher byte

		read_status |= api_ecg_reg_read(DATA_CH1_ECG_M_REG,&sample[1U]);				// reading the CH1 data- middle byte

		read_status |= api_ecg_reg_read(DATA_CH1_ECG_L_REG,&sample[2U]);				// reading the CH1 data- lower byte

		read_status |= api_ecg_reg_read(DATA_CH2_ECG_H_REG,&sample[3U]);				// reading the CH2 data- higher byte

		read_status |= api_ecg_reg_read(DATA_CH2_ECG_M_REG,&sample[4U]);				// reading the CH2 data- middle byte

		read_status |= api_ecg_reg_read(DATA_CH2_ECG_L_REG,&sample[5U]);				// reading the CH2 data- lower byte

		read_status |= api_ecg_reg_read(DATA_CH3_ECG_H_REG,&sample[6U]);				// reading the CH3 data- higher byte

		read_status |= api_ecg_reg_read(DATA_CH3_ECG_M_REG,&sample[7U]);				// reading the CH3 data- middle byte

		read_status |= api_ecg_reg_read(DATA_CH3_ECG_L_REG,&sample[8U]);				// reading the CH3 data- lower byte

		*buff_lead_1= (float)((((uint32_t)sample[9U])<<24U) | (((uint32_t)sample[0U])<<16U) | (((uint32_t)sample[1U])<<8U) | (sample[2U]) );

		*buff_lead_2= (float)((((uint32_t)sample[9U])<<24U) | (((uint32_t)sample[3U])<<16U) | (((uint32_t)sample[4U])<<8U) | (sample[5U]) );

		*buff_lead_3= (float)((((uint32_t)sample[9U])<<24U) | (((uint32_t)sample[6U])<<16U) | (((uint32_t)sample[7U])<<8U) | (sample[8U]) );

		if(read_status == ECG_NO_ERROR)
		{
			read_status = ESP_OK;
		}
	}
	ECG_Drdy_Flag = false;

	return read_status;
}

/*
* bool API_ECG_Capture_Samples_1Lead(float buff_lead_1)
* \brief        captures the raw data - 1 lead
*
* \retval		TRUE on success
*/
static uint32_t drdy_count = 0;

ECG_STATUS IRAM_ATTR API_ECG_Capture_Samples_2Lead(float *buff_lead_1, float *buff_lead_2)
{
	ECG_STATUS read_status = 0;
//	uint8_t sample[0] = 0x11;
//	uint8_t sample[1] = 0x11;
//	uint8_t sample[2] = 0x11;
	uint8_t sample[7] = {0};

		while(!ECG_Drdy_Flag);	//|| (API_TIMER_Get_Timeout_Flag(Set_1sec_timer) == FALSE)
//	if(ESP32_MCU_DRDY_PIN)
	{
		read_status = api_ecg_reg_read(DATA_CH1_ECG_H_REG,&sample[0U]);				// reading the CH1 data- higher byte

		read_status |= api_ecg_reg_read(DATA_CH1_ECG_M_REG,&sample[1U]);				// reading the CH1 data- middle byte

		read_status |= api_ecg_reg_read(DATA_CH1_ECG_L_REG,&sample[2U]);				// reading the CH1 data- lower byte

		read_status |= api_ecg_reg_read(DATA_CH2_ECG_H_REG,&sample[3U]);				// reading the CH2 data- higher byte

		read_status |= api_ecg_reg_read(DATA_CH2_ECG_M_REG,&sample[4U]);				// reading the CH2 data- middle byte

		read_status |= api_ecg_reg_read(DATA_CH2_ECG_L_REG,&sample[5U]);				// reading the CH2 data- lower byte

		*buff_lead_1= (float)((((uint32_t)sample[6U])<<24U) | (((uint32_t)sample[0U])<<16U) | (((uint32_t)sample[1U])<<8U) | (sample[2U]) );

		*buff_lead_2= (float)((((uint32_t)sample[6U])<<24U) | (((uint32_t)sample[3U])<<16U) | (((uint32_t)sample[4U])<<8U) | (sample[5U]) );

		if(read_status == ECG_NO_ERROR)
		{
			read_status = ESP_OK;
		}
	}
	ECG_Drdy_Flag = false;

	return read_status;
}

/*
* bool API_ECG_Capture_Samples_1Lead(uint32_t *buff_lead_1)
* \brief        captures the raw data - 1 lead
*
* \retval		TRUE on success
*/

bool API_ECG_Capture_Samples_1Lead(float *buff_lead_1)
{
	bool read_status = TRUE;
	uint8_t sample[3] = {0};

	//API_TIMER_Register_Timer(Set_1sec_timer);
	//Set_time_out_flag(Set_1sec_timer) ;
	while(ECG_Drdy_Flag);	//|| (API_TIMER_Get_Timeout_Flag(Set_1sec_timer) == FALSE)
	ECG_Drdy_Flag = false;
	//if(API_TIMER_Get_Timeout_Flag(Set_1sec_timer) == FALSE)
	//{
	read_status = api_ecg_reg_read(DATA_CH1_ECG_H_REG,&sample[0]);				// reading the CH1 data- higher byte

	read_status |= api_ecg_reg_read(DATA_CH1_ECG_M_REG,&sample[1]);				// reading the CH1 data- middle byte

	read_status |= api_ecg_reg_read(DATA_CH1_ECG_L_REG,&sample[2]);				// reading the CH1 data- lower byte

	*buff_lead_1 = (float)((((uint32_t)sample[0])<<16) | (((uint32_t)sample[1])<<8) | (sample[2]));


	return read_status;
}

static void api_drdy_input_config()
{
	esp_err_t error;

    gpio_config_t io_conf;

    MemSet(&io_conf,0,sizeof(io_conf));

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;

    io_conf.pin_bit_mask = ECG_DRDY_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    error = gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    error |= gpio_set_intr_type(ESP32_MCU_DRDY_PIN, GPIO_INTR_POSEDGE);

    error |= gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    error |= gpio_isr_handler_add(ESP32_MCU_DRDY_PIN, api_drdy_isr, (void*) ESP32_MCU_DRDY_PIN);

}

int ECG_Drdy_count = 0;
static void IRAM_ATTR api_drdy_isr(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	static uint32_t Temp_drdy = 0;

	esp_err_t status=ESP_FAIL;

	if(gpio_num == ESP32_MCU_DRDY_PIN)
	{
		Temp_drdy++;
		if(Temp_drdy == 1)
		{
			ECG_Drdy_Flag = true;
			ECG_Drdy_count++;
			gpio_set_level(JTAG_MTDI_DEBUG, 0);
		}
		else if(Temp_drdy > 1)
		{
			ECG_Drdy_Flag = false;
			Temp_drdy = 0;
			gpio_set_level(JTAG_MTDI_DEBUG, 1);
		}
	}

}


bool API_ECG_Init(void)
{

	bool config_status = false;

	API_IO_Exp1_P1_write_pin(DISPLAY_CSN,HIGH); // Disable Display

	api_drdy_input_config();

	for(int rpt=0;rpt<3;rpt++)
	{
		if(api_ads_check_Response() == ECG_NO_ERROR)
		{
			printf("\n[%s:%d:%s]", __FILE__, __LINE__, __func__);
			if(API_ECG_Reginit_2Lead() == ECG_NO_ERROR)
			{
				printf("\n[%s:%d:%s]", __FILE__, __LINE__, __func__);
				config_status = true;
				break;
			}
		}
	}

	return config_status;

}

void API_ECG_Chip_Reset(void)
{
//	API_IO_Exp1_P1_write_pin(ECG_CSN,HIGH);
	gpio_set_level(ECG_CSn_VCS, 1);
	API_IO_Exp1_P1_write_pin(ECG_RESETN,LOW);
	Delay_ms(500);
	API_IO_Exp1_P1_write_pin(ECG_RESETN,HIGH);

	Delay_ms(500);

}
static uint8_t GetLeadOffStatus(void)
{
	ECG_STATUS read_status = 0xFF;
	uint8_t flags = 0;

	read_status = api_ecg_reg_read(ERROR_LOD_REG,&flags);

	if(read_status != ECG_NO_ERROR ){
		printf("\nLead off Detection.. Register read fail");
	}

	printf("\nflags: 0x%2X",flags);

	//API_DISP_Error_Code(flags);

	return flags;
}

static void SetDcLeadOffCurrent_in_Steps_8nA(uint16_t currentIn_nA)
{
	ECG_STATUS write_read_status = 0;

	// The currentIn_nA range should be in 0 to 2048

	if(currentIn_nA<2048)
	{
		write_read_status |= api_ecg_reg_write(LOD_CURRENT_REG, (currentIn_nA/8));
	}

	else
	{
		printf("\nThe currentIn_nA value is Out off Range. it should be in (0 to 2048)");
	}

	if(write_read_status != ECG_NO_ERROR)
	{
		printf("\nEcg write fail! In function: SetDcLeadOffCurrent_in_Steps_8nA");
	}

}

bool API_ECG_Lead_OFF_Detect(ECG_LEADS_t lead)
{
    bool leadOffStatus = true;
    ECG_STATUS error_status = 0xFF;
	TIMER_t timeout = TIMER_10SEC;

	ECG_STATUS write_read_status = 0;

    uint8_t counter = 0;
    uint8_t leadMask=0xFF;
    uint8_t lod = 0;

	API_ECG_Enable_LeadOff_Detection();


	write_read_status |= api_ecg_reg_write(OSC_CN_REG,0x04);

	//Set DC Lead OFF Mode
	/*
	 *  Bit 2 = 0;
	 *  Bit 3 = 0;  Default it is 1.
	 */
	write_read_status |= api_ecg_reg_write(LOD_CN_REG, 0x00);

	// Setting Drive current in nA
	SetDcLeadOffCurrent_in_Steps_8nA(2047);

	// Enable Lead OFF detection
	write_read_status |= api_ecg_reg_write(LOD_EN_REG, 0x3F);

	error_status=api_ecg_reg_read(ERROR_STATUS_REG,&lod);

	if(error_status != ECG_NO_ERROR ){
		printf("\nLead off Detection.. Register read fail");
	}

	if(lead == LEAD1)
	{
	  leadMask = 0x03;// Lead1 Error Mask
	}
	else if(lead == LEAD2)
	{
	  leadMask = 0x07;// Lead2 Error Mask
	}

	else if(lead == LEAD12)
	{
		leadMask = 0x08;// Lead12 Error Mask
		timeout = TIMER_3SEC;
	}

	API_TIMER_Register_Timer(timeout);

	while(1)
	{
	    if( API_TIMER_Get_Timeout_Flag(timeout)) break;

	    Delay_ms(1000);

	    if(GetLeadOffStatus() & leadMask) // Returns true if electrode is not connected.
	    {
	    	counter = 0;
	    }

	    else
	    {
	    	counter++;
	    }

	    if(counter >= 1 ) // for retry
	    {
	    	leadOffStatus = false;
	    	break;
	    }
	}
	printf("error_status=0x%2X\n",lod);

	if(lod&0x08)
	{
		printf("lead of detected\n");
		API_ECG_Disable_LeadOff_Detection();
		return true;
	}


	printf("\ncounter = %d",counter);
	API_ECG_Disable_LeadOff_Detection();


	return leadOffStatus;
}

void API_ECG_Enable_LeadOff_Detection(void)
{
	 API_IO_Exp2_P1_write_pin(DC_LEAD_OFF,HIGH);
	 API_IO_Exp1_P1_write_pin(DC_LEAD_OFF_V,HIGH);

}

void API_ECG_Disable_LeadOff_Detection(void)
{
	 API_IO_Exp2_P1_write_pin(DC_LEAD_OFF,LOW);
	 API_IO_Exp1_P1_write_pin(DC_LEAD_OFF_V,LOW);

}

void API_ECG_Start_Conversion(void)
{
  api_ecg_reg_write(CONFIG_REG, START_CONV);	//Starts data conversion
}

void API_ECG_Stop_Conversion(void)
{
	api_ecg_reg_write(CONFIG_REG, STOP_CONV);	//Starts data conversion
}
