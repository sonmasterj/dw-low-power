#include "sdk_config.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include <stdbool.h>
//#include <stdint.h>
//#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"
//#include "nrf_uart.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
//#include "app_error.h"
#include "app_util_platform.h"
//#include "app_error.h"
//#include <string.h>
#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
//#include "nrf_soc.h"
//#include "nrf_drv_clock.h"
//#include "SEGGER_RTT.h"
//#include "nrf_pwr_mgmt.h"
#include "nrf_power.h"
//#include "nrf_section.h"
//#include "nrf_mtx.h"
//#include "uart.h"
// dw define
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_16M,      /* Pulse repetition frequency. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    3,               /* TX preamble code. Used in TX only. */
    3,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
//static dwt_txconfig_t txconfig = {
//    0xC0,            /* PG delay. */
//    0x0E082848,      /* TX power. */
//};
//--------------dw1000---end---------------
static uint8_t tx_mess[]={0xC5,0,'D','E','C','A','W','A','V','E',0,0};
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 7000

/* Dummy buffer for DW1000 wake-up SPI read. See NOTE 2 below. */
#define DUMMY_BUFFER_LEN 600
static uint8 dummy_buffer[DUMMY_BUFFER_LEN];
#define PIN_IN 2
/**
 * Application entry point.
 */
uint32 reg=0;
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	  dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
    dwt_writetxdata(sizeof(tx_mess), tx_mess, 0);
    dwt_writetxfctrl(sizeof(tx_mess), 0, 0);
		dwt_starttx(DWT_START_TX_IMMEDIATE);
	  tx_mess[BLINK_FRAME_SN_IDX]++;
	  if(tx_mess[BLINK_FRAME_SN_IDX]==10)
		{
			NRF_POWER->SYSTEMOFF=1;
		}
	  //dwt_entersleep();
}
static void gpio_init(void)
{
    nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}
int main(void)
{
	  //nrf_power_dcdcen_set(true);
	   /* clock_config*/
	   //nrf_drv_clock_init();
	   //nrf_drv_clock_lfclk_request(NULL);
    /* Reset and initialise DW1000. See NOTE 3 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
   // dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
	gpio_init();
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        while (1)
        { };
    }
    //SEGGER_RTT_WriteString(0,"sucessed");
    dwt_setsmarttxpower(1);
   // dwt_configuretxrf(&tx_config);
  //  dwt_softreset();
    port_set_dw1000_fastrate();
    /* Configure DW1000. See NOTE 4 below. */
    dwt_configure(&config);
    /* Configure sleep and wake-up parameters. */
    dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);
    dwt_entersleepaftertx(1);
    dwt_setinterrupt(DWT_INT_TFRS, 0);
    dwt_write8bitoffsetreg(AON_ID,AON_CFG1_OFFSET,0x00);
    dwt_writetxdata(sizeof(tx_mess), tx_mess, 0);
    dwt_writetxfctrl(sizeof(tx_mess), 0, 0);
		dwt_starttx(DWT_START_TX_IMMEDIATE);
		tx_mess[BLINK_FRAME_SN_IDX]++;
		//nrf_power_task_trigger(
   // deca_sleep(5000);
    //dwt_entersleep();
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE = 0;
		NRF_UARTE0->ENABLE=0;
		NRF_POWER->TASKS_LOWPWR=1;
		deca_sleep(8000);
		//nrf_power_dcdcen_set(1);
		NRF_SPI0->ENABLE=0;
	   //NRF_SPI1->ENABLE=0;
	//	nrf_pwr_mgmt_feed();
	//	nrf_pwr_mgmt_run();
	/*system power off*/
		//__WFE();
///		nrf_power_rampower_mask_on()
		//nrf_gpio_cfg_sense_input(2,NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);
		//NRF_POWER->SYSTEMOFF=1;
		//deca_sleep(3000);
		//nrf_power_system_off();
		//sd_app_evt_wait();
		
    while (1)
    {
			__SEV();
			__WFE();
			__WFE();
			//power_manage();
        /* Write frame data to DW1000 and prepare transmission. See NOTE 5 below. */
       // dwt_writetxdata(sizeof(tx_mess), tx_mess, 0); /* Zero offset in TX buffer. */
       // dwt_writetxfctrl(sizeof(tx_mess), 0, 0); /* Zero offset in TX buffer, no ranging. */
        /* Start transmission. */
      //  dwt_starttx(DWT_START_TX_IMMEDIATE);
        /* Execute a delay between transmissions. */
       // dwt_entersleep();
		//NRF_POWER->SYSTEMOFF=1;
   //     deca_sleep(TX_DELAY_MS);
     //   nrf_pwr_mgmt_run();
        /* Wake DW1000 up. See NOTE 2 below. */
      //  dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
        /* Increment the blink frame sequence number (modulo 256). */
      //  tx_mess[BLINK_FRAME_SN_IDX]++;
       // reg=dwt_read32bitoffsetreg(AON_ID,AON_CFG0_OFFSET);
   }
}
/* result: i=5mA, peak=45mA, charge 0.719C* in Segger IDE*/
/* result: i=4.2mA, peak=18, charge 0.513C* in KeilC IDE*/
/*date:21/08/2019*/
/*result: i=108uA, charge 13mC in KeilC IDE*/
/*result of other tag module: i=37uA, charge 4.5mC in 2 minute*/
/* result of low power /system on : i=4.2mA*/
/*date:22/08/2019*/
/*result when use _WFE() : i=200uA, =firmware os*/
