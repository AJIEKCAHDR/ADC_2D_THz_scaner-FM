/*
 * main.c (ADC_2D_THz_scaner+FM project)
 *
 *  Created on: 15 July. 2019.
 *      Author: aleksanderberdyugin@gmail.com
 */

#include <stdio.h>
#include <math.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include <sys/fcntl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/select.h>
#include <esp32/rom/ets_sys.h>
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "driver/periph_ctrl.h"
#include "esp_types.h"

#define BUFFER_SIZE	    64	//Set max buffer reading data for UART
#define BUZZ_PIN		 		 4  //pin for buzzer
#define WINDOW_APROX		30  //Set the window for approximating the signal values
#define ARRAY_SIZE		1000

int signals[2][ARRAY_SIZE];	//Array for signal

static const char* UART = "uart", * PIN = "gpio", * ADC = "adc";

EventGroupHandle_t adc_event_group;
static const int ADC_START_BIT = BIT0;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1
static esp_adc_cal_characteristics_t *adc_chars;


void begin_tone(int pin, uint32_t duty, int time) {					// Tone buzzer
	// Set LED Controller with previously prepared configuration	// number pin for buzzer, duty channel LED, time tone
    ledc_timer_config_t ledc_timer = { .duty_resolution = LEDC_TIMER_13_BIT, .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_1, .clk_cfg = LEDC_AUTO_CLK, };
	ledc_timer_config(&ledc_timer);
	ledc_channel_config_t ledc_channel[1] = { { .channel = LEDC_CHANNEL_2, .duty = 0, .gpio_num = pin,
	          .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_1 } };
	ledc_channel_config(&ledc_channel[0]);
	ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, duty);
	ledc_update_duty(ledc_channel[0].speed_mode,ledc_channel[0].channel);
	vTaskDelay(time / portTICK_PERIOD_MS);
	ledc_stop(ledc_channel[0].speed_mode,ledc_channel[0].channel,0);
}
void allow(uint8_t pin) {                              // buzz event allow
	begin_tone(pin, 2500, 250);
}
void denied(uint8_t pin) {                             // buzz event denied
	for (int i = 0; i < 5; i++) {
		begin_tone(pin, 2500, 70);
		vTaskDelay(30 / portTICK_PERIOD_MS);
	}
}

int string_to_number(const char* str) {
	int result = 0;
	result = atoi(str);
	return result;
}// cut out of array char value transform to integer
bool shift_and_verify_string_number(char* str, int shift) {
	for(int i = 0; i < strlen(str); i++)
	{
		str[i] = str[i+shift]; 					//string left shift by 'shift' symbol
		if(i == strlen(str)) { break; }
		if(((str[i] < '0')||(str[i] > '9')) && (str[i] != '\n')) { return false; }
	}
	return true;
} //verify number value step, if value not correct then return error
void shift_string(char* str, int shift) {
	for(int i = 0; i < strlen(str); i++) {
		str[i] = str[i+shift];
	}		//string left shift by 'SHIFT' symbol
}

int gpio_control_pin(char *str, gpio_mode_t mode, int value) {
	shift_and_verify_string_number(str, 4);		//string verify and left shift by 4 symbol (pin:)
	if(!strncmp(str,"error",5)) { return -1; }	//if value not correct then return error= -1
	int number_pin = string_to_number(str);		//what pin number?
	ESP_LOGI(PIN, "PIN: %i", number_pin);
	switch(mode) {
	case GPIO_MODE_DISABLE:  	//mode reading gpio
		if(((number_pin>=0)&&(number_pin!=20)&&(number_pin!=24)&&(number_pin<=27))||((number_pin>=32)&&(number_pin<=39))) {}
		else { ESP_LOGE(PIN, "failed reading pin: %s", str); return -5; }
		value = gpio_get_level(number_pin);
		if(value < 0) { ESP_LOGE(PIN, "failed reading pin: %s", str); }
		ESP_LOGI(PIN, "PIN value read: %i", value);
		return value;
	case GPIO_MODE_OUTPUT: 		//mode output gpio
		if(((number_pin>=0)&&(number_pin!=1)&&(number_pin!=3)&&(number_pin<=5))||(number_pin==14)||
			((number_pin>=16)&&(number_pin!=20)&&(number_pin!=24)&&(number_pin<=27))||((number_pin>=32)&&(number_pin<=33))) {}
		else { ESP_LOGE(PIN, "failed set output high pin: %s", str); return -5; }
		gpio_set_direction(number_pin, GPIO_MODE_OUTPUT);	//set gpio output mode
		gpio_set_level(number_pin, value);						//set gpio HIGH level output gpio
		break;
	case GPIO_MODE_INPUT:  		//mode input gpio
		if(((number_pin>=0)&&(number_pin!=20)&&(number_pin!=24)&&(number_pin<=27))||((number_pin>=32)&&(number_pin<=39))) {}
		else { ESP_LOGE(PIN, "failed set input pin: %s", str); return -5; }
		gpio_set_direction(number_pin, GPIO_MODE_INPUT);		//set gpio input mode
		break;
	default:
		ESP_LOGE(PIN, "no such mode gpio: %s", str);
		return -4;
	}
	return 1;
}
int command_gpio(char *str) {
	shift_string(str, 5);		//string left shift by 5 symbol
	if(!strncmp(str,"read/",5))	{	//which mode gpio? reading mode?
		shift_string(str, 5);		//string left shift by 5 symbol
		int read_value = gpio_control_pin(str, GPIO_MODE_DISABLE, 0);	//reading digital value pin
		if(read_value < 0) { ESP_LOGE(PIN, "failed reading pin: %s", str); return read_value; }
	} else if(!strncmp(str,"out/",4))	{	//which mode gpio? output mode?
		shift_string(str, 4);		//string left shift by 4 symbol
		if(!strncmp(str,"high/",5))	{	//which output mode? high?
			shift_string(str, 5);		//string left shift by 5 symbol
			return gpio_control_pin(str, GPIO_MODE_OUTPUT, pdTRUE);			//set HIGH value pin
		} else if(!strncmp(str,"low/",4))	{	//which output mode? low?
			shift_string(str, 4);		//string left shift by 4 symbol
			return gpio_control_pin(str, GPIO_MODE_OUTPUT, pdFALSE);		//set LOW value pin
		} else { ESP_LOGE(PIN, "no such output mode gpio: %s", str); return -3; } //error= -3 (no such output mode gpio)
	} else if(!strncmp(str,"in/",3))	{	//which mode gpio? input mode?
		shift_string(str, 3);		//string left shift by 3 symbol
		if(!strncmp(str,"pin:",4))	{	//which command gpio? pin?
			return gpio_control_pin(str, GPIO_MODE_INPUT, 0);			//set INPUT direction pin
		} else { ESP_LOGE(PIN, "no such command gpio: %s", str); return -3; } //error= -3 (no such command gpio)
	} else { ESP_LOGE(PIN, "no such mode gpio: %s", str); return -3; } //error= -3 (no such mode gpio)
	return 1;				//Command completed
}
int reading_adc1_one(void) {
	uint32_t adc_reading = 0;
	adc_reading = adc1_get_raw((adc1_channel_t)channel);
	ESP_LOGI(ADC,"Raw: %d", adc_reading);
	return adc_reading;
}
int reading_adc1_multi(void) {
	uint32_t adc_reading = 0;
    for (int i = 0; i < 64; i++) { adc_reading += adc1_get_raw((adc1_channel_t)channel); }
    adc_reading /= 64;
	ESP_LOGI(ADC,"Raw: %d", adc_reading);
	return adc_reading;
}
int reading_adc1_period(char *str) {
	if(false == shift_and_verify_string_number(str, 7)) {return -1;} //string verify and left shift by 7 symbol
	int *ptr_adc_val; int size_array = string_to_number(str);
	ESP_LOGI(ADC," size: %d", size_array);
	ptr_adc_val = (int*)malloc(size_array * sizeof(int));
	if (ptr_adc_val == NULL) { ESP_LOGE(ADC, "failed create dynamic massive"); return -11; }
	for (int i = 0; i < size_array; i++) {
		ptr_adc_val[i] = adc1_get_raw((adc1_channel_t)channel);
		if(ptr_adc_val[i] == -1) { ESP_LOGE(ADC, "failed reading ADC"); return -12; }
	}
	for (int n = 0; n < size_array; n++) { ESP_LOGI(ADC,"	%d	%d", n, ptr_adc_val[n]); }
	free(ptr_adc_val);
	return 1;
}
int reading_adc1_max_period(char *str) {
	if(false == shift_and_verify_string_number(str, 4)) {return -1;} //string verify and left shift by 4 symbol
	int *ptr_adc_val; int size_array = string_to_number(str);
	ptr_adc_val = (int*)malloc(size_array * sizeof(int));
	if (ptr_adc_val == NULL) { ESP_LOGE(ADC, "failed create dynamic massive"); return -11; }
	for (int i = 0; i < size_array; i++) {
		ptr_adc_val[i] = adc1_get_raw((adc1_channel_t)channel);
		if(ptr_adc_val[i] == -1) { ESP_LOGE(ADC, "failed reading ADC"); return -12; }
	}
	int max_sign = 0;
	for (int n = 0; n < size_array; n++) { if(ptr_adc_val[n] > max_sign) { max_sign =  ptr_adc_val[n]; } }
	ESP_LOGI(ADC,"Max: %d", max_sign);
	free(ptr_adc_val);
	return 1;
}

int getData(void) {
  for (int i = 0; i < ARRAY_SIZE; i++) {
    signals[1][i] = adc1_get_raw((adc1_channel_t)channel);
    if(signals[1][i] == -1) { ESP_LOGE(ADC, "failed reading ADC"); return -12; }
  }
  return 1;
}
void getApprox(int *a, int *b, int n, int step) {
  int sumx = 0, sumy = 0, sumx2 = 0, sumxy = 0;
  for (int i = n; i < n+step; i++) {
    sumx += signals[0][i];
    sumy += signals[1][i];
    sumx2 += signals[0][i] * signals[0][i];
    sumxy += signals[0][i] * signals[1][i];
  }
  *a = (step*sumxy - (sumx*sumy)) / (step*sumx2 - sumx*sumx);
  *b = (sumy - *a*sumx) / step;
  return;
}
static void reading_adc1_smooth_max_start(void *arg) {
	xEventGroupSetBits(adc_event_group, ADC_START_BIT);				//set flag START adc
	ESP_LOGI(ADC,"Success ADC start!");
	int a, b, max_sign = 0;
	while(1) {
		if (-12 == getData()) { xEventGroupClearBits(adc_event_group, ADC_START_BIT); vTaskDelete(NULL); }		//clear flag START adc
		for (int i = 0, step = WINDOW_APROX; i < ARRAY_SIZE; i+=step) {
			if(ARRAY_SIZE-i < WINDOW_APROX) step = ARRAY_SIZE-i;
			getApprox(&a, &b, i, step);
			for (int n = i; n < i+step; n++) { signals[1][n] = a*n + b; }
		}
		// for (int i = 0; i<ARRAY_SIZE; i++) ESP_LOGI(ADC,"%d : %d", signals[0][i], signals[1][i]);	// Testing Aproximation
		for (int n = 0; n < ARRAY_SIZE; n++) { if(signals[1][n] > max_sign) { max_sign =  signals[1][n]; } }
		ESP_LOGI("Max","%d", max_sign);
		if((ADC_START_BIT & xEventGroupWaitBits(adc_event_group, ADC_START_BIT, false, false, (TickType_t) 0)) == 0) { break; }
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	xEventGroupClearBits(adc_event_group, ADC_START_BIT);		//clear flag START adc
	vTaskDelete(NULL);
}
int command_adc(char *str) {
	shift_string(str, 4);		//string left shift by 4 symbol
	if(!strncmp(str,"sm_max/",7))	{	//reading mode? maximum level line smooth signal in periodic reading adc
		shift_string(str, 7);		//string left shift by 7 symbol
		if(!strncmp(str,"stop",4))	{	//stop adc proccess
			if((ADC_START_BIT & xEventGroupWaitBits(adc_event_group, ADC_START_BIT, false, false, (TickType_t) 0)) != 0) {
				xEventGroupClearBits(adc_event_group, ADC_START_BIT);		//clear flag START adc								//Stop of the ADC series process
				ESP_LOGI(ADC,"Success ADC stop!");
			} else { ESP_LOGE(ADC, "Fail STOP. adc no action!"); return -7; }	//if no set flag START adc
		} else if(!strncmp(str,"start",5))	{	//start adc proccess
			if((ADC_START_BIT & xEventGroupWaitBits(adc_event_group, ADC_START_BIT, false, false, (TickType_t) 0)) == 0) {
				xTaskCreatePinnedToCore(reading_adc1_smooth_max_start, "adc1_smooth_max_start", 8192, NULL, 5, NULL, 1);		//Start of the ADC series process
			} else { ESP_LOGE(ADC, "Fail START. adc action!"); return -7; }	//if no clear flag START adc
		} else {ESP_LOGE(ADC, "no such mode sm_max: %s", str); return -4;} //error= -4 (no such mode sm_max:)
	} else if(!strncmp(str,"max:",4))	{	//reading mode? maximum level signal in periodic reading adc
		return reading_adc1_max_period(str);
	} else if(!strncmp(str,"period:",7))	{	//reading mode? periodic reading adc
		return reading_adc1_period(str);
	} else if(!strncmp(str,"one",3))	{	//reading mode? one sampling reading adc
		int read_value = reading_adc1_one();
		if(read_value < 0) { ESP_LOGE(ADC, "failed reading adc one: %s", str); return read_value; }
	} else if(!strncmp(str,"multi",5))	{	//reading mode? multisampling reading adc
		int read_value = reading_adc1_multi();
		if(read_value < 0) { ESP_LOGE(ADC, "failed reading adc multi: %s", str); return read_value; }
	} else {ESP_LOGE(ADC, "no such mode adc: %s", str); return -3;} //error= -3 (no such mode adc)
	return 1;				//Command completed
}

static int handler_command(char *string) {
	if(!strncmp(string,"c/",2))	{		//compares the first symbol == 'c' it means a command
		shift_string(string, 2);		//string left shift by 2 symbol
		if(!strncmp(string,"adc/",4)) {		//which driver? adc?
			return command_adc(string);
		} else if(!strncmp(string,"gpio/",5)) {		//which driver? gpio?
			return command_gpio(string);
		} else if(!strncmp(string,"reboot",6)) {	//rebooting esp32?
			ESP_LOGI("handler_cmd", "Rebooting ESP32...");
			esp_restart();
		} else {
			ESP_LOGE("handler_cmd", "no such driver: %s", string); return -2;
		}	//error= -2 (no such driver)
	}
	else { return 0; }		//No command
	return 1;				//Command completed
}

static void uart_select_task(void *arg) {
	ESP_LOGI(UART, "Initialization UART...");
	vTaskDelay(20 / portTICK_PERIOD_MS);
    uart_config_t uart_config = {   //Configuring Initial Parameters of UART
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0);
    ESP_LOGI(UART, "Initialization DONE!");
    while (1) {
    	ESP_LOGI(UART, "Open UART...");
        int fd; char buf[BUFFER_SIZE + 1];
        if ((fd = open("/dev/uart/0", O_RDWR)) == -1) {     //Opening UART
        	ESP_LOGE(UART, "Cannot open UART");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        esp_vfs_dev_uart_use_driver(0);						//Configuration UART0 driver for read/write data VFS
        int s; fd_set rfds; unsigned int error_h;
        struct timeval tv = { .tv_sec = 5, .tv_usec = 0, };		//Set timeout data 5 second, 0 microsecond	
        ESP_LOGI(UART, "Open DONE!");
        while (1) {
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            s = select(fd + 1, &rfds, NULL, NULL, &tv);				//Waiting new data in VFS from UART0
            if (s < 0) { ESP_LOGE(UART, "Select failed: errno %d", errno); break; }                          //Failed select data
			else if (s == 0) { /*ESP_LOGI(UART, "Timeout has been reached and nothing has been received");*/ }   //Timeout, no data
            else {
                if (FD_ISSET(fd, &rfds)) {							//Checking file descriptor readiness
                    if (read(fd, &buf, BUFFER_SIZE) > 0) {	//Reading received data success
                    	//ESP_LOGW(UART, "%s", buf);
                    	if((error_h = handler_command(buf)) == 1) { /*ESP_LOGI(UART, "OK");*/}	//Call handler commands UART, Result call handler commands UART
                    	else { if (error_h == 0) { ESP_LOGI(UART, "No command UART"); } else { denied(BUZZ_PIN); ESP_LOGE(UART, "Command failed: error %d", error_h); } }
                    }
					else { ESP_LOGE(UART, "UART read error"); break; }      //Failed reading received data
				} else { ESP_LOGE(UART, "No FD has been set in select()"); break; }
            }
        }
        close(fd);          //Close UART
    }
    vTaskDelete(NULL);
}
static void adc1_initialize(void *arg) {
	vTaskDelay(100 / portTICK_PERIOD_MS);
	ESP_LOGI(ADC, "Initialization ADC1...");
    //Configure ADC
	esp_err_t ret = adc1_config_width(ADC_WIDTH_BIT_12);
	if (ret != ESP_OK) {ESP_LOGE(ADC,"Failed configuring width ADC1"); vTaskDelete(NULL);}
	ret = adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
    if (ret != ESP_OK) {ESP_LOGE(ADC,"Failed configuring channel attenuation ADC1"); vTaskDelete(NULL);}
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) { ESP_LOGI(ADC,"Characterized using Two Point Value"); }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) { ESP_LOGI(ADC,"Characterized using eFuse Vref"); }
    else { ESP_LOGI(ADC,"Characterized using Default Vref"); }
    ESP_LOGI(ADC, "Initialization DONE!");
    xEventGroupClearBits(adc_event_group, ADC_START_BIT);		//clear flag ADC start
    vTaskDelete(NULL);
}

void app_main(void)
{
	printf("ADC_2D_THz_scaner+FM start...\n");
	adc_event_group = xEventGroupCreate();	//create event group for flags allow mcpwm to go
	xTaskCreate(uart_select_task, "uart_select_task", 4096, NULL, 6, NULL);		//start task uart communication
	xTaskCreate(adc1_initialize, "adc1_initialize", 4096, NULL, 7, NULL);	//task for initialize adc1
	for(int n = 0; n < ARRAY_SIZE; n++) { signals[0][n] = n; signals[1][n] = 0; }	//clear data in array signal
	for (int i = 0; i < 2; i++) {	//tone for success start ADC_2D_THz_scaner+FM
		begin_tone(BUZZ_PIN, 2500, 80);
		vTaskDelay(90 / portTICK_PERIOD_MS);
		if(i==1) begin_tone(BUZZ_PIN, 2500, 80);
		vTaskDelay(90 / portTICK_PERIOD_MS);
	}
}

