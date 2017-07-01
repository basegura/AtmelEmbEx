/*
 * Embedded_III_Final_Assignment.c
 *
 * Created: 11/23/2015 6:43:31 PM
 *  Author: Brandon Segura
 * Using the blink.c from the Resources Directory as the initial framework built from 
 * there.
 *
 *
 * Circuit setup:
 * Please refer to schematic image for setup.
 *
 * I have added a Diagnostic LED to digital output 10. This LEDs purpose was to remain on 
 * while the push button was not pressed and to turn off while the push button was active.
 *
 * Note that when first initializing the Uno with this code the Diagnostic LED will flash rapidly
 * then a delay where the main LED will not blink then it will begin.
 *
 * Known Issues: 
 * 1) Upon pressing the button to suspend updating the change in blink 
 * rate, the brightness of the LED will cut in half.
 * 2) When changing the potentiometer's value, there is a chance that
 * jitter in the LED may be present. This is a response to the timer 
 * value surpassing the OCR value and having to manually be reset to below 
 * as to avoid waiting for an overflow to occur. More detail is explained below.
 */ 

#define F_CPU 1000000UL  // 1 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "croutine.h"

//Task Priorities
#define PUSH_BUTTON_PRIORITY	    (tskIDLE_PRIORITY + 3)
#define BLINK_TASK_PRIORITY			(tskIDLE_PRIORITY + 2)
#define NON_BLINK_PRIORITY			1


#define BLINK_LED				(_BV(PORTB1)) /* IO9 */
#define PUSH_BUTTON				(_BV(PORTB0)) /* IO8 */
#define PUSH_BUTTON_DIAG_LED    (_BV(PORTB2)) /* IO10*/



static SemaphoreHandle_t mxpcr;

//Task Handler for suspend updating the Blink Rate Task
TaskHandle_t xUpdateBlinkHandle;

volatile int current_ADC_Value = 0;
//flags
volatile int timer_flag = 0;
volatile int adc_flag = 0;
volatile int blink_rate_flag = 0;
volatile int push_button_flag = 0;
volatile int push_button_check_count = 0;
volatile int push_button_status_flag = 0;
volatile int push_button_state_flag = 0;

//Semaphore Code as from blink.c
static int mxpcr_init(void)
{
	mxpcr = xSemaphoreCreateMutex();
	return (mxpcr != NULL) ? 0 : -1;
}

static int mxpcr_lock(void)
{
	BaseType_t ret;
	
	ret = xSemaphoreTake(mxpcr, portMAX_DELAY);
	return (ret == pdPASS) ? 0 : -1;
}

static int mxpcr_unlock(void)
{
	BaseType_t ret;

	ret = xSemaphoreGive(mxpcr);
	return (ret == pdPASS) ? 0 : -1;
}

static const inline TickType_t ms_to_ticks(int ms)
{
	return ms / portTICK_PERIOD_MS;
}
//End Semaphore Setup Code



//Begin Pin setup code
static void setup_main_led(void)
{
	mxpcr_lock();
	{
		DDRB |= BLINK_LED;
		PORTB |= BLINK_LED;
		//Setup of Diagnostic LED
		DDRB |= PUSH_BUTTON_DIAG_LED;
		PORTB |= PUSH_BUTTON_DIAG_LED;
	}
	mxpcr_unlock();
}

static void setup_push_button(void)
{
	
	/*
	*  Wanted the push button to work on a Pin change interrupt.
	*  Setup code taken from here: https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
	*/
	mxpcr_lock();
	{
		// Clear IO8
		// IO8 (PCINT0 pin) is now an input
		DDRB &= ~(1 << DDB0);
		
		//Turn On the Pull-up for IO8
		PORTB |= (1 << PORTB0);

		PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
		PCMSK0 |= (1 << PCINT0);  // set PCINT0 to trigger an interrupt on state change

	}
	mxpcr_unlock();
}

static void timerInit(void) {
	
	//Timer code as presented in AVR Dude and C lecture. 
	
	mxpcr_lock();
	{
		//Using Timer 1 with interrupt
		//Interrupts enabled in main function
		OCR1A = 0x0008;

		TCCR1B |= (1 << WGM12);
		// Mode 4, CTC on OCR1A

		TIMSK1 |= (1 << OCIE1A);
		//Set interrupt on compare match

		TCCR1B |= (1 << CS12) | (1 << CS10);
		// set prescaler to 1024 and start the timer
	}
	mxpcr_unlock();
}

//Interrupt service routine for the Timer Compare Match.
ISR (TIMER1_COMPA_vect){
	timer_flag = 1;
}

static void adcInit(void) {
	//Set to channel 0; Use ARef as voltage reference.
	ADMUX |= (1<<REFS0);
	
	//ADC Interrupt Enable
	ADCSRA = 0x8F;
	//ADCSRA |= (1 << ADPS1); //Pre-scalar of 8
}
//End setup code

//Main LED blink function.
static void toggle_main_led(void)
{
	mxpcr_lock();
	{
		PORTB ^= BLINK_LED;
	}
	mxpcr_unlock();
}

static void blink_task(void *args)
{
	//Framework for this function comes from blink.c
	//Want to also add a flag.
	//Flag will be set by ISR when a Compare Match is found in the timer
	//If the flag is set, pass to toggle function, then clear the flag.
	//If not then delay this task.
	mxpcr_init();
	
	for (;;)
	{
		if (timer_flag == 1){
			toggle_main_led();
			timer_flag = 0;
		}
		else{
			//If we don't have to do anything we can yield to another task
			vTaskDelay(1);
		}
	}
}



static void adc_task(void *args) {
	
	//Check to see if ADC ISR has set flag
	//If so set the new ADC value.
	//Clear ADC flag
	//Then set blink_rate_flag to notify 
	//the update blink rate task that they can 
	//now update the frequency of the LED flash.
	for(;;)
	{
		if (adc_flag ==1)
		{
			//if (current_ADC_Value != ADC)
			{
				current_ADC_Value = ADC;
				adc_flag = 0;
				blink_rate_flag = 1;
			}
		}
	}
}

static void updateBlinkRateTask(void *args) {
	
	//Check Blink Rate Flag
	//If set update the OCR1A to new ADC value
	//Restart ADC sweeping
	//Clear Blink Rate Flag
	for(;;)
	{
		if (blink_rate_flag == 1)
		{
			OCR1A = current_ADC_Value*3;
			/*
			* Timer check logic. Since I am asynchronously changing
			* the output compare register I cannot be sure that the current
			* timer value will be greater than the max value.
			* This would lead to having to wait until the timer overflowed before
			* the next timer interrupt would be executed leading to a delay in the
			* next LED state change.
			* Solution and explanation retrieved from:
			* http://www.avrfreaks.net/forum/timer-haltdelay-when-changing-ocr1a-value
			* 
			* Added logic to ensure that when we check we will only change the value
			* of the time iff we are above the current OCR1A value.
			* This leads to a slight jitter in the LED when it occurs (i.e. sweeping
			* the potentiometer too quickly).
			*/
			if(TCNT1 > OCR1A){
			TCNT1 = (current_ADC_Value*3)-10;
			}
			blink_rate_flag = 0;
			ADCSRA |= 1<<ADSC;
		}
	}
}

static void suspendBlinkRateUpdate (void){
	//Suspend updateBlinkRateTask
	//Set push button flag as to not suspend more than once.
	vTaskSuspend(xUpdateBlinkHandle);
	push_button_flag = 1;
}

static void resumeBlinkRateUpdate (void) {
	//Resume updateBlinkRateTask
	//Remove push button flag.
	vTaskResume(xUpdateBlinkHandle);
	push_button_flag = 0;
}

static void check_push_button (void){
	//Check to see if a pin state change has been detected
	if (push_button_state_flag == 1)
	{
		push_button_status_flag = (PINB & _BV(PORTB0));
		//Check the pins status
		//if pin is low extinguish Diag LED and set flag high
		//suspend updateBlinkRateTask function
		if (push_button_status_flag == 1)
		{
			PORTB = (0<<PORTB2);
			suspendBlinkRateUpdate();
		}
		//if pin is low extinguish Diag LED and set flag low
		//resume updateBlinkRateTask
		if (push_button_status_flag == 0)
		{
			PORTB = (1<<PORTB2);
			resumeBlinkRateUpdate();
		}
	}
}

static void checkPushButtonTask(void *args) {
	//Task to scan if a change has been detected 
	//in the push button state.
	//If none has been detected delay the task as 
	//to allow other tasks to run.
	for(;;){
		check_push_button();
		vTaskDelay(1);
	}
}
//Initialization Code
//Flashes LED rapidly
static void flashDiagLED (void) {
	PORTB = (0<<PORTB2);
	_delay_ms(100);
	PORTB = (1<<PORTB2);
	_delay_ms(100);
	PORTB = (0<<PORTB2);
	_delay_ms(500);
	PORTB = (1<<PORTB2);
	_delay_ms(500);
	PORTB = (0<<PORTB2);
	_delay_ms(500);
	PORTB = (1<<PORTB2);
	_delay_ms(500);
}

int main(void)
{
	setup_main_led();
	timerInit();
	adcInit();
	setup_push_button();
	flashDiagLED();
	//Interrupt Enable function call
	sei();
	//Begin sweeping ADC Conversions
	ADCSRA |= 1<<ADSC;
	
	xTaskCreate(blink_task, "blink", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
	xTaskCreate(adc_task, "read", configMINIMAL_STACK_SIZE, NULL, NON_BLINK_PRIORITY, NULL);
	xTaskCreate(updateBlinkRateTask, "update" ,configMINIMAL_STACK_SIZE,NULL,NON_BLINK_PRIORITY, &xUpdateBlinkHandle);
	xTaskCreate(checkPushButtonTask, "pushButton", configMINIMAL_STACK_SIZE, NULL, PUSH_BUTTON_PRIORITY, NULL);
	vTaskStartScheduler(); /* never returns */

	return 0;
}

/* callback from FreeRTOS; can be disabled in config */
void vApplicationIdleHook (void)
{
}

ISR(ADC_vect){
	//Set a flag
	adc_flag=1;
	//ADCSRA |= 1<<ADSC;
}

ISR (PCINT0_vect)
{
	push_button_state_flag = 1;
	//Debounce Code???
}