#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "tremo_system.h"
#include "tremo_regs.h"
#include "tremo_rtc.h"
#include "tremo_uart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "tremo_pwr.h"
#include "tremo_delay.h"
#include "rtc-board.h"
#include "tremo_adc.h"


//#define FW_VERSION "debug/1.0.0"
#define FW_VERSION "release/1.0.0"

#define SDK_VERSION "Ai-Thinker 1.6.2"

/////////////flash///////////////////
#define TEST_DATA_SIZE 2

static uint8_t test_data[TEST_DATA_SIZE]={0xba,0xad};;
static uint32_t test_addr = 0x0800D000;
/////////////////////////////////////

#define ADC_DATA_NUM 100
extern int app_start(void);

int32_t adc_data1=0;
int32_t adc_data2=0;
int32_t adc_cal=0;
int8_t buzzer=0;

uint16_t adc_data_1[ADC_DATA_NUM] = {0};
uint16_t adc_data_2[ADC_DATA_NUM] = {0};
float gain, dco;

void printInfo(void){
	uint32_t sn[2];
	uint8_t *buf = (uint8_t *)sn;

	system_get_chip_id(sn);

	printf("\r\n-------------------------------------\r\n");
	printf("ASR6601chipID, %02X%02X%02X%02X%02X%02X%02X%02X\r\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);

	printf("Firmware Version:%s\r\n", FW_VERSION);
	printf("Compile_time:%s %s\r\n",__DATE__, __TIME__);
	printf("\r\n");
	printf("-------------------------------------\r\n");

}

void adc_continue_mode_test(void)
{
	//printf("ADC/");
	uint32_t i;
    gpio_t *gpiox;
    uint32_t pin;
	float calibratedVal;
	gpio_write(GPIOA,GPIO_PIN_15,0);
	gpio_write(GPIOB,GPIO_PIN_0,0);
	
    gpiox = GPIOA;
    pin = GPIO_PIN_8;
    gpio_init(gpiox, pin, GPIO_MODE_ANALOG);
    pin = GPIO_PIN_11;
    gpio_init(gpiox, pin, GPIO_MODE_ANALOG);

    adc_init();

    adc_config_clock_division(20); //sample frequence 150K
	adc_config_ref_voltage(ADC_INTERNAL_REF_VOLTAGE);
	
	adc_config_sample_sequence(0, 1);
    adc_config_sample_sequence(1, 2);
    adc_config_sample_sequence(2, 2);

    adc_config_conv_mode(ADC_CONV_MODE_SINGLE);
	adc_get_calibration_value(false, &gain, &dco);
    adc_enable(true);

    adc_start(true);

    for (i = 0; i < ADC_DATA_NUM; i++)
    {
        while(!adc_get_interrupt_status(ADC_ISR_EOC));
        adc_data_1[i] = adc_get_data();
        while(!adc_get_interrupt_status(ADC_ISR_EOC));
        adc_data_2[i] = adc_get_data();
        while(!adc_get_interrupt_status(ADC_ISR_EOC));
        (void)adc_get_data();
        adc_start(true);
    }

    adc_start(false);
    adc_enable(false);
		adc_data1=0;
		adc_data2=0;
	  for (i = 0; i < ADC_DATA_NUM; i++)
    {
        adc_data1=adc_data1+adc_data_1[i];
		adc_data2=adc_data2+adc_data_2[i];
    }
		adc_data1=adc_data1/100;
		adc_data2=adc_data2/100;
		
		
		calibratedVal = ((1.2/4096)*adc_data1 - dco)/gain;
		//printf("RAW: ADC1 %ld ADC2 %ld, CalibratedVal: ADC1 %f \r\n", adc_data1, adc_data2, calibratedVal);
		//printf("gain: %f,  dco: %f\r\n",gain, dco);
		adc_data2=adc_data2-adc_cal;
		adc_data1 = (5.914*calibratedVal + 0.1812)*1000;
		if(adc_data2<0)
		{
		adc_data2=0;
		}
		printf("Return VAl ADC1: %ld, ADC2: %ld \r\n",adc_data1, adc_data2);
		if(buzzer==1)
		{
			if(adc_data2>300)
			{
				gpio_write(GPIOA,GPIO_PIN_2,1);
			}
			else{gpio_write(GPIOA,GPIO_PIN_2,0);}
		}		
		gpio_write(GPIOA,GPIO_PIN_15,1);
		gpio_write(GPIOB,GPIO_PIN_0,1);
		

}



void uart_log_init(void)
{
    // uart0
    
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

    /* uart config struct init */
    uart_config_t uart_config;
    uart_config_init(&uart_config);

    uart_config.baudrate = UART_BAUDRATE_9600;
    uart_init(CONFIG_DEBUG_UART, &uart_config);
    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
}

void board_init()
{
	////////////////ADC////////////////////////////////

	////////////////////////////////////////////////////	
    rcc_enable_oscillator(RCC_OSC_XO32K, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_AFEC, true);
	rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);
	
	gpio_set_iomux(GPIOB, GPIO_PIN_0, 0);
	gpio_set_iomux(GPIOA, GPIO_PIN_2, 0);
	gpio_init(GPIOA, GPIO_PIN_15, GPIO_MODE_OUTPUT_PP_LOW);  /// Power lm358 voltage enable pin
	gpio_init(GPIOB, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP_HIGH); /// battery voltage enable pin
	gpio_init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP_HIGH); /// buzzer control pin
		
    delay_ms(100);
    pwr_xo32k_lpm_cmd(true);   
    uart_log_init();
    RtcInit();	 
	delay_ms(500);
	gpio_init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP_LOW); 
		//////////Calibration///////////////
		if(*(uint8_t*)(test_addr )!=0x00)
		{
			adc_continue_mode_test();	
			test_data[0]=0x00;
			test_data[1]=adc_data2/8;
			flash_erase_page(test_addr);
			flash_program_bytes(test_addr, test_data, TEST_DATA_SIZE);
		}
		adc_cal=*(uint8_t*)(test_addr+1)*8;
		if(adc_cal>1700)
			{
						adc_cal=1700;
			}
		buzzer=1;


		/////////////////////////////////////
		
}

int main(void)
{
    // Target board initialization
    board_init();
	printInfo();
    app_start();
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;

    while (1) { }
}
#endif
