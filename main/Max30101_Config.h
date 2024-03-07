#include "MainFlow.h"


#if MAX86150

#ifndef _MAX86150_CONFIG_H_
#define _MAX86150_CONFIG_H_

#include <stdint.h>

static const uint8_t MAX86150_INTSTAT1 =		0x00;
static const uint8_t MAX86150_INTSTAT2 =		0x01;
static const uint8_t MAX86150_INTENABLE1 =		0x02;
static const uint8_t MAX86150_INTENABLE2 =		0x03;

static const uint8_t MAX86150_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX86150_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX86150_FIFOREADPTR = 	0x06;
static const uint8_t MAX86150_FIFODATA 		=		0x07;

static const uint8_t MAX86150_FIFOCONFIG = 		0x08;
static const uint8_t MAX86150_FIFOCONTROL1= 	0x09;
static const uint8_t MAX86150_FIFOCONTROL2 = 	0x0A;

static const uint8_t MAX86150_SYSCONTROL = 	0x0D;
static const uint8_t MAX86150_PPGCONFIG1 = 		0x0E;
static const uint8_t MAX86150_PPGCONFIG2 = 		0x0F;
static const uint8_t MAX86150_LED_PROX_AMP = 	0x10;

static const uint8_t MAX86150_LED1_PULSEAMP = 	0x11;
static const uint8_t MAX86150_LED2_PULSEAMP = 	0x12;
static const uint8_t MAX86150_LED_RANGE 		= 	0x14;
//static const uint8_t MAX86150_LED_PILOT_PA 	= 	0x15;

static const uint8_t MAX86150_ECG_CONFIG1 	= 	0x3C;
static const uint8_t MAX86150_ECG_CONFIG3 	= 	0x3E;
static const uint8_t MAX86150_PROXINTTHRESH = 	0x10;

static const uint8_t MAX86150_PARTID = 			0xFF;

// MAX86150 Commands
static const uint8_t MAX86150_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX86150_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX86150_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX86150_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX86150_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX86150_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX86150_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX86150_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX86150_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX86150_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX86150_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX86150_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX86150_SAMPLEAVG_MASK =	(byte)~0b11100000;
static const uint8_t MAX86150_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX86150_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX86150_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX86150_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX86150_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX86150_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX86150_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX86150_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX86150_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX86150_A_FULL_MASK = 	0xF0;

static const uint8_t MAX86150_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX86150_SHUTDOWN = 		0x80;
static const uint8_t MAX86150_WAKEUP = 			0x00;

static const uint8_t MAX86150_RESET_MASK = 		0xFE;
static const uint8_t MAX86150_RESET = 			0x01;

static const uint8_t MAX86150_MODE_MASK = 		0xF8;
//static const uint8_t MAX86150_MODE_REDONLY = 	0x02;
//static const uint8_t MAX86150_MODE_REDIRONLY = 	0x03;
//static const uint8_t MAX86150_MODE_MULTILED = 	0x07;

//static const uint8_t MAX86150_ADCRANGE_MASK = 	0x9F;
//static const uint8_t MAX86150_ADCRANGE_2048 = 	0x00;
//static const uint8_t MAX86150_ADCRANGE_4096 = 	0x20;
//static const uint8_t MAX86150_ADCRANGE_8192 = 	0x40;
//static const uint8_t MAX86150_ADCRANGE_16384 = 	0x60;

//static const uint8_t MAX86150_SAMPLERATE_MASK = 0xE3;
//static const uint8_t MAX86150_SAMPLERATE_50 = 	0x00;
//static const uint8_t MAX86150_SAMPLERATE_100 = 	0x04;
//static const uint8_t MAX86150_SAMPLERATE_200 = 	0x08;
//static const uint8_t MAX86150_SAMPLERATE_400 = 	0x0C;
//static const uint8_t MAX86150_SAMPLERATE_800 = 	0x10;
//static const uint8_t MAX86150_SAMPLERATE_1000 = 0x14;
//static const uint8_t MAX86150_SAMPLERATE_1600 = 0x18;
//static const uint8_t MAX86150_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX86150_PULSEWIDTH_MASK = 0xFC;
//static const uint8_t MAX86150_PULSEWIDTH_69 = 	0x00;
//static const uint8_t MAX86150_PULSEWIDTH_118 = 	0x01;
//static const uint8_t MAX86150_PULSEWIDTH_215 = 	0x02;
//static const uint8_t MAX86150_PULSEWIDTH_411 = 	0x03;

static const uint8_t MAX86150_SLOT1_MASK = 		0xF0;
static const uint8_t MAX86150_SLOT2_MASK = 		0x0F;
static const uint8_t MAX86150_SLOT3_MASK = 		0xF0;
static const uint8_t MAX86150_SLOT4_MASK = 		0x0F;

//static const uint8_t SLOT_NONE 				= 			0x00;
static const uint8_t SLOT_RED_LED 		= 			0x01;
static const uint8_t SLOT_IR_LED 			= 			0x02;
//static const uint8_t SLOT_RED_PILOT 	=				0x09;
//static const uint8_t SLOT_IR_PILOT 		= 			0x0A;
static const uint8_t SLOT_ECG					= 			0x0D;

#endif /* _MAX86150_CONFIG_H_ */


#endif


#if MAX30101

#ifndef _MAX30101_CONFIG_H_
#define _MAX30101_CONFIG_H_

#include <stdint.h>

/*MAX30101 macros */

static const uint8_t MAX30101_MODECONFIG =		0x09;
#define REG_INTR_STATUS_1 			(uint8_t)0x00u
#define REG_INTR_STATUS_2 			(uint8_t)0x01u
#define REG_INTR_ENABLE_1 			(uint8_t)0x02u
#define REG_INTR_ENABLE_2 			(uint8_t)0x03u
#define REG_FIFO_WR_PTR 			(uint8_t)0x04u
#define REG_OVF_COUNTER 			(uint8_t)0x05u
#define REG_FIFO_RD_PTR 			(uint8_t)0x06u
#define REG_FIFO_DATA 				(uint8_t)0x07u
#define REG_FIFO_CONFIG 			(uint8_t)0x08u
#define REG_MODE_CONFIG 			(uint8_t)0x09u
#define REG_SPO2_CONFIG 			(uint8_t)0x0Au
#define REG_LED1_PA 				(uint8_t)0x0Cu
#define REG_LED2_PA 				(uint8_t)0x0Du
#define REG_PILOT_PA 				(uint8_t)0x10u

/* Values to Configuration Registers*/
#define RESET						(uint8_t)0x40u
#define INTR_ENABLE_1				(uint8_t)0xC0u					// Interrupt enabled  when new data is ready
#define INTR_ENABLE_2				(uint8_t)0x00u					// Disable interrupt for die temperature
#define FIFO_WR_PTR					(uint8_t)0x00u					// FIFO Write Pointer points to the location where the MAX30102 writes the next sample
#define OVF_COUNTER					(uint8_t)0x00u					// OVF_COUNTER counts the number of samples lost. It saturates at 0xF.
#define FIFO_RD_PTR					(uint8_t)0x00u					// FIFO Read Pointer points to the location from where the processor gets the next sample from the FIFO

#define LED1_PA						(uint8_t)0x32u					// Choose value for approx. 7.4mA for LED1
#define LED2_PA						(uint8_t)0x32u					// Choose value for approx. 7.4mA for LED2
#define PILOT_PA					(uint8_t)0x7Fu					// Choose value for approx. 25.4mA for Pilot LED



/* BP Config */
#define MODE_CONFIG_BP				(uint8_t)0x02u					// Heart Rate mode, Red LED only
#define FIFO_CONFIG_BP				(uint8_t)0x0Fu					// 1 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17
#define SPO2_CONFIG_BP				(uint8_t)0x15u//29					// SPO2_ADC range = 4096nA, SPO2 sample rate - 200sps, LED pulseWidth - (118uS)



/* SPO2 Config */

#define MODE_CONFIG_SpO2			(uint8_t)0x03u					// Enable Red LED and IR LED
#define FIFO_CONFIG_SpO2			(uint8_t)0x5Fu					// 2 SAMPLES AVERAGED PER FIFO SAMPLE,FIFO is not updated until FIFO_DATA is read, fifo almost full = 17
#define SPO2_CONFIG_SpO2			(uint8_t)0x2Du					// SPO2_ADC range = 4096nA, SPO2 sample rate - 200sps, LED pulseWidth - (411uS)


static const uint8_t MAX86150_INTSTAT1 =		0x00;
static const uint8_t MAX86150_INTSTAT2 =		0x01;
static const uint8_t MAX86150_INTENABLE1 =		0x02;
static const uint8_t MAX86150_INTENABLE2 =		0x03;

static const uint8_t MAX86150_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX86150_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX86150_FIFOREADPTR = 	0x06;
static const uint8_t MAX86150_FIFODATA 		=		0x07;

static const uint8_t MAX86150_FIFOCONFIG = 		0x08;
static const uint8_t MAX86150_FIFOCONTROL1= 	0x09;
static const uint8_t MAX86150_FIFOCONTROL2 = 	0x0A;

static const uint8_t MAX86150_SYSCONTROL = 	0x0D;

static const uint8_t MAX86150_PPGCONFIG1 = 		0x0E;
static const uint8_t MAX86150_PPGCONFIG2 = 		0x0F;
static const uint8_t MAX86150_LED_PROX_AMP = 	0x10;

static const uint8_t MAX86150_LED1_PULSEAMP = 	0x11;
static const uint8_t MAX86150_LED2_PULSEAMP = 	0x12;
static const uint8_t MAX86150_LED_RANGE 		= 	0x14;
static const uint8_t MAX86150_LED_PILOT_PA 	= 	0x15;

static const uint8_t MAX86150_ECG_CONFIG1 	= 	0x3C;
static const uint8_t MAX86150_ECG_CONFIG3 	= 	0x3E;
static const uint8_t MAX86150_PROXINTTHRESH = 	0x10;

static const uint8_t MAX86150_PARTID = 			0xFF;

// MAX86150 Commands
static const uint8_t MAX86150_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX86150_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX86150_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX86150_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX86150_INT_DATA_RDY_ENABLE =	0x50;
static const uint8_t MAX86150_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX86150_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX86150_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX86150_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX86150_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX86150_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX86150_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX86150_SAMPLEAVG_MASK =	(byte)~0b11100000;
static const uint8_t MAX86150_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX86150_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX86150_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX86150_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX86150_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX86150_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX86150_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX86150_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX86150_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX86150_A_FULL_MASK = 	0xF0;

static const uint8_t MAX86150_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX86150_SHUTDOWN = 		0x80;
static const uint8_t MAX86150_WAKEUP = 			0x00;

static const uint8_t MAX86150_RESET_MASK = 		0xFE;
static const uint8_t MAX86150_RESET = 			0x01;

static const uint8_t MAX86150_MODE_MASK = 		0xF8;
//static const uint8_t MAX86150_MODE_REDONLY = 	0x02;
//static const uint8_t MAX86150_MODE_REDIRONLY = 	0x03;
//static const uint8_t MAX86150_MODE_MULTILED = 	0x07;

//static const uint8_t MAX86150_ADCRANGE_MASK = 	0x9F;
//static const uint8_t MAX86150_ADCRANGE_2048 = 	0x00;
//static const uint8_t MAX86150_ADCRANGE_4096 = 	0x20;
//static const uint8_t MAX86150_ADCRANGE_8192 = 	0x40;
//static const uint8_t MAX86150_ADCRANGE_16384 = 	0x60;

//static const uint8_t MAX86150_SAMPLERATE_MASK = 0xE3;
//static const uint8_t MAX86150_SAMPLERATE_50 = 	0x00;
//static const uint8_t MAX86150_SAMPLERATE_100 = 	0x04;
//static const uint8_t MAX86150_SAMPLERATE_200 = 	0x08;
//static const uint8_t MAX86150_SAMPLERATE_400 = 	0x0C;
//static const uint8_t MAX86150_SAMPLERATE_800 = 	0x10;
//static const uint8_t MAX86150_SAMPLERATE_1000 = 0x14;
//static const uint8_t MAX86150_SAMPLERATE_1600 = 0x18;
//static const uint8_t MAX86150_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX86150_PULSEWIDTH_MASK = 0xFC;
//static const uint8_t MAX86150_PULSEWIDTH_69 = 	0x00;
//static const uint8_t MAX86150_PULSEWIDTH_118 = 	0x01;
//static const uint8_t MAX86150_PULSEWIDTH_215 = 	0x02;
//static const uint8_t MAX86150_PULSEWIDTH_411 = 	0x03;

static const uint8_t MAX86150_SLOT1_MASK = 		0xF0;
static const uint8_t MAX86150_SLOT2_MASK = 		0x0F;
static const uint8_t MAX86150_SLOT3_MASK = 		0xF0;
static const uint8_t MAX86150_SLOT4_MASK = 		0x0F;

//static const uint8_t SLOT_NONE 				= 			0x00;
static const uint8_t SLOT_RED_LED 		= 			0x01;
static const uint8_t SLOT_IR_LED 			= 			0x02;
static const uint8_t SLOT_RED_PILOT 	=				0x09;
//static const uint8_t SLOT_IR_PILOT 		= 			0x0A;
static const uint8_t SLOT_ECG					= 			0x0D;

#endif /* _MAX86150_CONFIG_H_ */

#endif
