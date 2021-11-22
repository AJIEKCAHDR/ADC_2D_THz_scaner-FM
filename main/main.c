/*
 * main.c (ADC_2D_THz_scaner+FM project)
 *
 *  Created on: 15 July. 2021.
 *      Author: aleksanderberdyugin@gmail.com
 */

#include <stdio.h>
#include <math.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include <sys/fcntl.h>
#include <sys/errno.h>
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
#define BUZZ_PIN		 4  //pin for buzzer
#define WINDOW_APROX	30  //Set the window for approximating the signal values
#define ARRAY_SIZE	  1000

int signals[2][ARRAY_SIZE];	//Array for signal

static const char* UART = "uart", * ADC = "adc";

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
void denied(uint8_t pin) {                             // buzz event denied
	for (int i = 0; i < 5; i++) {
		begin_tone(pin, 2500, 70);
		vTaskDelay(30 / portTICK_PERIOD_MS);
	}
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
int reading_adc1_smooth_max_period(void) {
	int a, b, max_sign = 0;
	for (int i = 0; i < ARRAY_SIZE; i++) { signals[1][i] = adc1_get_raw((adc1_channel_t)channel); }
	for (int i = 0, step = WINDOW_APROX; i < ARRAY_SIZE; i+=step) {
		if(ARRAY_SIZE-i < WINDOW_APROX) step = ARRAY_SIZE-i;
		getApprox(&a, &b, i, step);
		for (int n = i; n < i+step; n++) { signals[1][n] = a*n + b; }
	}
	// for (int i = 0; i<ARRAY_SIZE; i++) ESP_LOGI(ADC,"%d : %d", signals[0][i], signals[1][i]);	// Testing Aproximation
	for (int n = 0; n < ARRAY_SIZE; n++) { if(signals[1][n] > max_sign) { max_sign =  signals[1][n]; } }
	ESP_LOGI("Max","%d", max_sign);
	return 1;
}

static int handler_command(char *string) {
	if(!strncmp(string,"sm_max",6)) {		//which driver? adc?
		return reading_adc1_smooth_max_period();
	} else if(!strncmp(string,"reboot",6)) {	//rebooting esp32?
			ESP_LOGI("handler_cmd", "Rebooting ESP32...");
			esp_restart();
		} else {
			ESP_LOGE("handler_cmd", "no such driver: %s", string); return -2;
		}	//error= -2 (no such driver)
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
                    	for(int n = 0; n < ARRAY_SIZE; n++) { signals[1][n] = 0; }	//clear data in array signal
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
    vTaskDelete(NULL);
}

void app_main(void)
{
	printf("ADC_2D_THz_scaner+FM start v_4.1 ...\n");
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

