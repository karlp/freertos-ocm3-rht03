/*
 * Karl Palsson, 2012 <karlp@tweak.net.au
 */

#include <errno.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

#include "syscfg.h"

struct state_t volatile state;

 static TaskHandle_t xHandleRht = NULL;
#define RHT_FLAG_COMPLETE (1<<0)
#define RHT_FLAG_TIMEOUT (1<<1)

void clock_setup(void)
{
	rcc_clock_setup_in_hsi_out_24mhz();
	/* Lots of things on all ports... */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_TIM7);
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_ADC1);
}

void usart_console_setup(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);
	usart_enable(USART_CONSOLE);
}

void gpio_setup(void)
{
	gpio_set_mode(PORT_RHT_POWER, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PIN_RHT_POWER);

	// FIXME - we aren't using this yet...
	gpio_set_mode(PORT_STATUS_LED, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PIN_STATUS_LED);
}

void systick_setup(void)
{
	/* 24MHz / 8 => 3000000 counts per second. */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* 3000000/3000 = 1000 overflows per second - every 1ms one interrupt */
	systick_set_reload(2999);
	systick_interrupt_enable();
	systick_counter_enable();
}

/**
 * Use USART_CONSOLE as a console.
 * @param file
 * @param ptr
 * @param len
 * @return 
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;

	return -1;
}

void dht_power(bool enable)
{
	if (enable) {
		gpio_set(PORT_RHT_POWER, PIN_RHT_POWER);
	} else {
		gpio_clear(PORT_RHT_POWER, PIN_RHT_POWER);
	}
}

void stuff_bit(int bitnumber, int timing, volatile uint8_t * bytes)
{
	int byte_offset = bitnumber / 8;
	int bit = 7 - (bitnumber % 8); // Stuff MSB first.
	if (timing < RHT_LOW_HIGH_THRESHOLD) {
		bytes[byte_offset] &= ~(1 << bit);
	} else {

		bytes[byte_offset] |= (1 << bit);
	}
}

void RHT_isr(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	exti_reset_request(RHT_EXTI);
	int cnt = TIM7_CNT;
	TIM7_CNT = 0;
	// Skip catching ourself pulsing the start line until the 150uS start.
	if (!state.seen_startbit) {
		if (cnt < RHT_LOW_HIGH_THRESHOLD) {
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
			return;
		} else {
			state.seen_startbit = true;
		}
	}
	if (state.bitcount > 0) { // but skip that start bit...
		stuff_bit(state.bitcount - 1, cnt, state.rht_bytes);
	}
	state.bitcount++;
	if (state.bitcount >= 40) {
		xTaskNotifyFromISR( xHandleRht, RHT_FLAG_COMPLETE, eSetBits, &xHigherPriorityTaskWoken );
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

/**
 * We set this timer to count uSecs.
 * The interrupt is only to indicate that it timed out and to shut itself off.
 */
void setup_tim7(void)
{
	timer_clear_flag(TIM7, TIM_SR_UIF);
	TIM7_CNT = 1;
	timer_set_prescaler(TIM7, 23); // 24Mhz/1Mhz - 1
	timer_set_period(TIM7, RHT_INTER_BIT_TIMEOUT_USEC);
	timer_enable_irq(TIM7, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM7_IRQ);
	timer_enable_counter(TIM7);
}

static void start_rht_read(void)
{
	// First, move the pins up and down to get it going...
	gpio_set_mode(PORT_RHT_IO, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PIN_RHT_IO);
	gpio_clear(PORT_RHT_IO, PIN_RHT_IO);
	// docs say 1-10ms is enough....
	vTaskDelay(portTICK_PERIOD_MS * 5);
	gpio_set(PORT_RHT_IO, PIN_RHT_IO);
	// want to wait for 40us here, but we're ok with letting some code delay us..
	state.bitcount = 0;
	state.seen_startbit = false;
	nvic_enable_irq(RHT_NVIC);
	// pull up will finish the job here for us.
	gpio_set_mode(PORT_RHT_IO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, PIN_RHT_IO);
	exti_select_source(RHT_EXTI, PORT_RHT_IO);
	exti_set_trigger(RHT_EXTI, EXTI_TRIGGER_FALLING);
	exti_enable_request(RHT_EXTI);
	setup_tim7();
}

void tim7_isr(void)
{
	timer_clear_flag(TIM7, TIM_SR_UIF);
	nvic_disable_irq(NVIC_TIM7_IRQ);
	timer_disable_irq(TIM7, TIM_DIER_UIE);
	timer_disable_counter(TIM7);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR( xHandleRht, RHT_FLAG_TIMEOUT, eSetBits, &xHigherPriorityTaskWoken );
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* Just to make "return" easier to read with a while (1) loop */
static void taskRht_Single(void) {
	printf("rht start\n");
	start_rht_read();
	// wait for events here,
	// either an event for complete, or an event for timeout!
	uint32_t val;
	BaseType_t xResult = xTaskNotifyWait(pdFALSE, /* don't clear bits on entry? */
		ULONG_MAX, /* clear on exit */
		&val,
		pdMS_TO_TICKS(3)
		);
	if (xResult == pdPASS) {
		if (val & RHT_FLAG_COMPLETE) {
			unsigned chksum = state.rht_bytes[0] + state.rht_bytes[1] + state.rht_bytes[2] + state.rht_bytes[3];
			chksum &= 0xff;
			// Removing this print makes the checksum fail most of the time! :|
			printf("%x %x %x %x sum: %x == %x\n",
				state.rht_bytes[0], state.rht_bytes[1], state.rht_bytes[2], state.rht_bytes[3],
				chksum, state.rht_bytes[4]);
			if (chksum != state.rht_bytes[4]) {
				printf("CHKSUM failed, ignoring: \n");
				return;
			}

			int rh = (state.rht_bytes[0] << 8 | state.rht_bytes[1]);
			state.last_relative_humidity = rh / 10.0;
			int temp = (state.rht_bytes[2] << 8 | state.rht_bytes[3]);
			state.last_temperature = temp / 10.0;
			printf("Temp: %d.%d C, RH = %d.%d %%\n", temp / 10, temp % 10, rh / 10, rh % 10);

		} else if (val & RHT_FLAG_TIMEOUT) {
			printf("RHT timeout, toggling power\n");
			dht_power(false);
			vTaskDelay(pdMS_TO_TICKS(1000));
			dht_power(true);
			printf("rht repowered on...\n");
			return;
		} else {
			printf("unexpected notification?\n");
		}
	} else {
		printf("timed out waiting for event?!\n");
	}
	
}

static void prvTaskRht(void *pvParameters) {
	(void)pvParameters;
	dht_power(true);
	while (1) {
		vTaskDelay(portTICK_PERIOD_MS * 3000);
		taskRht_Single();
	}
}

static void prvTaskTicker(void *pvParameters) {
	(void)pvParameters;
	while (1) {
		vTaskDelay(portTICK_PERIOD_MS * 1000);
		printf("Tick: %d\n", state.seconds);
		state.seconds++;
		gpio_toggle(PORT_STATUS_LED, PIN_STATUS_LED);
	}
}

int main(void)
{
	clock_setup();
	gpio_setup();
	systick_setup();
	
	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);
        // FIXME - this works, but what priority is what really?!
#define IRQ2NVIC_PRIOR(x)       ((x)<<4)
        nvic_set_priority(NVIC_SYSTICK_IRQ, IRQ2NVIC_PRIOR(1));
        nvic_set_priority(NVIC_EXTI9_5_IRQ, IRQ2NVIC_PRIOR(6));
        nvic_set_priority(NVIC_TIM7_IRQ, IRQ2NVIC_PRIOR(7));
	
	adc_off(ADC1);
	usart_console_setup();
	printf("hello!\n");
	// power up the RHT chip...
	
	xTaskCreate(prvTaskRht, "rhtreader", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xHandleRht);
	configASSERT(xHandleRht);
	
	xTaskCreate(prvTaskTicker, "ticker", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

	adc_power_on(ADC1);
	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);

	vTaskStartScheduler();

	while (1) {
	}

	return 0;
}


void vAssertCalled(const char * const pcFileName, unsigned long ulLine)
{
        volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;

        /* Parameters are not used. */
        (void) ulLine;
        (void) pcFileName;

        taskENTER_CRITICAL();
        {
                while (ulSetToNonZeroInDebuggerToContinue == 0) {
                        /* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
                        non zero value to step out of this function to the point that raised
                        this assert(). */
                        __asm volatile( "NOP");
                        __asm volatile( "NOP");
                }
        }
        taskEXIT_CRITICAL();
}
