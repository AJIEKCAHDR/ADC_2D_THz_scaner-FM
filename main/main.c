/*
 * main.c (ADC_2D_THz_scaner+FM project)
 *
 *  Created on: 3 November. 2019.
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

// Testing Aproximation
// int test_signal[1500] = {2352,	2368,	2418,	2394,	2355,	2390,	2389,	2410,	2433,	2429,	2439,	2448,	2458,	2465,	2423,	2486,	2527,	2487,	2525,	2514,	2709,	2526,	2515,	2544,	2487,	2558,	2531,	2556,	2598,	2609,	2623,	2604,	2607,	2615,	2608,	2637,	2640,	2640,	2654,	2673,	2676,	2701,	2686,	2701,	2702,	2734,	2709,	2707,	2736,	2715,	2731,	2723,	2775,	2751,	2747,	2747,	2751,	2825,	2787,	2674,	2767,	2815,	2816,	2822,	2812,	2824,	2801,	2831,	2822,	2853,	2853,	2832,	2861,	2729,	2866,	2911,	2844,	2800,	2893,	2914,	2902,	2896,	3059,	2885,	2923,	2935,	2925,	2943,	2990,	3023,	2942,	2962,	2991,	2927,	2961,	2963,	2966,	2967,	2973,	2983,	3029,	2982,	2992,	2981,	2971,	3003,	2993,	3025,	3003,	3049,	2971,	3005,	3010,	3039,	3007,	3007,	3211,	3031,	3056,	3003,	3056,	3046,	3047,	3027,	3071,	3022,	3088,	3060,	3056,	3040,	3081,	3070,	3087,	3089,	3106,	3103,	3099,	3059,	3112,	3088,	3103,	3151,	3031,	3167,	3082,	3108,	3130,	3086,	3162,	3008,	3130,	3121,	3068,	3123,	3082,	3087,	3183,	3121,	3066,	3108,	3095,	3131,	3149,	3120,	3113,	3107,	3119,	3109,	3136,	3130,	3265,	3156,	3123,	3120,	3249,	3119,	3101,	3119,	3111,	3110,	3104,	3103,	2960,	3107,	3098,	3090,	3014,	3119,	3088,	3059,	3059,	3168,	3055,	2999,	2978,	3042,	3070,	3031,	3039,	3033,	3026,	3019,	2933,	3026,	3047,	3007,	3001,	3012,	3015,	2996,	3002,	2823,	2983,	2860,	2971,	2943,	2954,	2929,	2928,	2922,	2909,	2926,	2907,	2864,	2889,	2893,	2864,	2858,	2866,	2827,	2831,	2812,	2828,	2816,	2776,	2771,	2797,	2771,	2715,	2748,	2975,	2692,	2741,	2712,	2711,	2672,	2644,	2673,	2650,	2609,	2641,	2625,	2512,	2615,	2586,	2595,	2559,	2551,	2559,	2495,	2522,	2512,	2495,	2479,	2448,	2414,	2445,	2416,	2358,	2416,	2577,	2384,	2370,	2352,	2283,	2303,	2301,	2290,	2290,	2256,	2256,	2241,	2222,	2243,	2206,	2160,	2157,	2160,	2133,	2097,	2122,	2113,	2055,	2073,	2030,	2028,	2023,	1915,	2006,	2013,	1949,	1935,	1929,	1856,	1904,	1930,	1830,	1862,	1833,	1847,	1775,	1776,	1718,	1712,	1728,	1680,	1681,	1649,	1680,	1631,	1595,	1616,	1510,	1529,	1551,	1510,	1513,	1494,	1491,	1447,	1427,	1542,	1386,	1367,	1360,	1337,	1319,	1282,	1280,	1269,	1233,	1253,	1194,	1193,	1164,	1127,	1159,	1095,	1078,	1105,	912,	1005,	998,	981,	998,	944,	898,	889,	839,	809,	849,	896,	801,	752,	757,	737,	727,	720,	656,	691,	636,	624,	594,	541,	517,	637,	522,	478,	451,	463,	521,	414,	462,	310,	278,	331,	296,	311,	285,	183,	235,	251,	145,	189,	93,	122,	171,	102,	64,	47,	48,	0,	23,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	8,	21,	17,	70,	0,	91,	66,	149,	173,	226,	176,	198,	230,	242,	246,	259,	292,	260,	346,	326,	374,	390,	400,	432,	396,	471,	400,	518,	521,	539,	563,	583,	567,	627,	640,	680,	663,	681,	710,	727,	628,	784,	772,	782,	804,	832,	862,	771,	871,	880,	925,	944,	951,	880,	985,	989,	970,	1071,	1040,	1077,	1073,	1093,	1117,	1005,	1148,	1162,	1186,	1168,	1180,	1218,	1231,	1301,	1239,	1273,	1295,	1315,	1331,	1323,	1347,	1367,	1387,	1430,	1415,	1441,	1456,	1467,	1488,	1476,	1442,	1520,	1475,	1559,	1566,	1471,	1602,	1611,	1631,	1637,	1645,	1654,	1687,	1650,	1712,	1757,	1723,	1741,	1724,	1759,	1776,	1737,	1795,	1752,	1829,	1811,	1867,	1869,	1885,	1899,	1895,	1895,	1935,	1942,	1897,	1953,	1975,	1957,	1995,	2015,	2007,	2023,	2078,	2070,	2075,	2077,	2105,	2122,	2059,	2102,	2192,	2152,	2138,	2129,	2145,	2095,	2180,	2215,	2212,	2214,	2307,	2246,	2240,	2253,	2267,	2257,	2271,	2353,	2317,	2317,	2334,	2327,	2345,	2347,	2368,	2352,	2384,	2407,	2391,	2471,	2349,	2435,	2422,	2333,	2457,	2307,	2622,	2478,	2483,	2531,	2533,	2512,	2559,	2550,	2419,	2571,	2543,	2546,	2578,	2586,	2597,	2594,	2691,	2637,	2621,	2493,	2631,	2627,	2641,	2645,	2626,	2635,	2714,	2666,	2734,	2691,	2676,	2734,	2711,	2741,	2679,	2745,	2661,	2759,	2736,	2771,	2768,	2770,	2749,	2626,	2785,	2786,	2767,	2796,	2844,	2849,	2821,	2764,	2862,	2796,	2826,	2834,	2970,	2824,	2862,	2863,	2884,	2865,	2909,	2917,	2885,	2903,	2880,	2881,	2902,	2947,	2960,	2950,	2913,	2918,	2935,	2974,	2921,	2959,	2899,	2987,	2940,	2982,	3025,	2978,	2964,	2945,	2993,	2944,	3002,	3015,	3003,	2973,	2999,	3029,	2902,	3052,	3024,	3024,	3083,	3031,	3117,	3030,	3059,	3043,	3088,	3033,	3053,	3093,	3061,	3071,	3061,	3094,	3046,	3099,	2990,	3069,	3083,	3051,	3099,	3111,	3126,	3095,	3117,	3098,	3087,	3091,	3102,	3121,	3193,	3126,	3101,	3125,	3088,	3133,	3109,	3103,	3125,	3125,	3019,	3120,	3116,	3134,	3041,	3126,	3136,	3179,	3108,	3131,	3051,	3136,	3134,	3143,	3129,	3124,	3110,	3136,	3198,	3087,	3114,	3130,	3105,	3123,	3136,	3141,	3117,	3088,	3121,	3088,	3120,	3085,	3089,	3079,	3118,	3090,	3065,	3018,	3056,	3043,	3055,	3041,	3027,	3063,	3063,	3041,	2992,	2976,	3005,	2999,	2992,	2986,	2983,	2963,	2983,	2962,	2988,	2946,	2941,	2968,	2922,	2950,	2918,	2867,	2911,	2887,	2887,	2895,	2865,	2810,	2871,	2829,	2843,	2755,	2765,	2813,	2818,	2742,	2768,	2763,	2779,	2765,	2749,	2739,	2736,	2687,	2704,	2671,	2687,	2553,	2670,	2661,	2649,	2637,	2633,	2610,	2596,	2602,	2623,	2533,	2606,	2550,	2539,	2495,	2497,	2497,	2521,	2490,	2451,	2442,	2447,	2401,	2410,	2395,	2273,	2390,	2362,	2363,	2331,	2261,	2326,	2295,	2278,	2276,	2205,	2225,	2249,	2213,	2197,	2198,	2160,	2138,	2102,	2128,	2160,	2096,	2095,	2095,	2073,	2002,	2096,	2001,	1994,	1979,	1946,	2029,	1963,	1920,	1901,	1891,	1843,	1854,	1810,	1810,	1797,	1786,	1756,	1721};

#define BUFFER_SIZE	    64	//Set max buffer reading data for UART
#define BUZZ_PIN		 4  //pin for buzzer
#define WINDOW_APROX	30  //Set the window for approximating the signal values

static const char* UART = "uart", * PIN = "gpio", * ADC = "adc";

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

int ** getData(int n) {
  int **f;
  f = (int**) malloc(2 * sizeof(int*));
  f[0] = (int*)malloc(n * sizeof(int));	//Сreate an array for the ordinal number of the value
  f[1] = (int*)malloc(n * sizeof(int));	//Сreate an array for the value
  if (f[0] == NULL) { ESP_LOGE(ADC, "failed create dynamic massive [0]"); return NULL; }
  if (f[1] == NULL) { ESP_LOGE(ADC, "failed create dynamic massive [1]"); return NULL; }
  for (int i = 0; i < n; i++) {
    f[0][i] = (int)i;
    // f[1][i] = test_signal[i];																			// Testing Aproximation
    f[1][i] = adc1_get_raw((adc1_channel_t)channel);
    if(f[1][i] == -1) { ESP_LOGE(ADC, "failed reading ADC"); return (int**)-11; }
  }
  return f;
}
void getApprox(int **x, int *a, int *b, int n, int step) {
  int sumx = 0, sumy = 0, sumx2 = 0, sumxy = 0;
  for (int i = n; i < n+step; i++) {
    sumx += x[0][i];
    sumy += x[1][i];
    sumx2 += x[0][i] * x[0][i];
    sumxy += x[0][i] * x[1][i];
  }
  *a = (step*sumxy - (sumx*sumy)) / (step*sumx2 - sumx*sumx);
  *b = (sumy - *a*sumx) / step;
  return;
}
int reading_adc1_smooth_max_period(char *str) {
	if(false == shift_and_verify_string_number(str, 7)) {return -1;} //string verify and left shift by 7 symbol
	int size_array = string_to_number(str);
	if (size_array <= 0) { ESP_LOGE(ADC, "invalid value: size_array"); return -13; }
	int **data, a, b, max_sign = 0;
	data = getData(size_array);
	if (data == NULL) { free(data); return -11; }
	for (int i = 0, step = WINDOW_APROX; i < size_array; i+=step) {
		if(size_array-i < WINDOW_APROX) step = size_array-i;
		getApprox(data, &a, &b, i, step);
		for (int n = i; n < i+step; n++) { data[1][n] = a*n + b; }
	}
	// for (int i = 0; i<size_array; i++) ESP_LOGI(ADC,"%d : %d", data[0][i], data[2][i]);					// Testing Aproximation
	for (int n = 0; n < size_array; n++) { if(data[1][n] > max_sign) { max_sign =  data[1][n]; } }
	ESP_LOGI(ADC,"Max: %d", max_sign);
	free(data);
	return 1;
}
int command_adc(char *str) {
	shift_string(str, 4);		//string left shift by 4 symbol
	if(!strncmp(str,"sm_max:",7))	{	//reading mode? maximum level line smooth signal in periodic reading adc
		return reading_adc1_smooth_max_period(str);
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
        ESP_LOGI(UART, "Open DONE!");
        while (1) {
            int s; fd_set rfds;
            struct timeval tv = { .tv_sec = 5, .tv_usec = 0, };		//Set timeout data 5 second, 0 microsecond
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            s = select(fd + 1, &rfds, NULL, NULL, &tv);				//Waiting new data in VFS from UART0
            if (s < 0) { ESP_LOGE(UART, "Select failed: errno %d", errno); break; }                          //Failed select data
			else if (s == 0) { /*ESP_LOGI(UART, "Timeout has been reached and nothing has been received");*/ }   //Timeout, no data
            else {
                if (FD_ISSET(fd, &rfds)) {							//Checking file descriptor readiness
                    if (read(fd, &buf, BUFFER_SIZE) > 0) {	//Reading received data success
                    	//ESP_LOGW(UART, "%s", buf);
                    	unsigned int error_h;
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
    vTaskDelete(NULL);
}

void app_main(void)
{
	printf("ADC_2D_THz_scaner+FM start...\n");
	xTaskCreate(uart_select_task, "uart_select_task", 4096, NULL, 6, NULL);		//start task uart communication
	xTaskCreate(adc1_initialize, "adc1_initialize", 4096, NULL, 7, NULL);	//task for initialize adc1
	for (int i = 0; i < 2; i++) {	//tone for success start 2D_THz_scaner
		begin_tone(BUZZ_PIN, 2500, 80);
		vTaskDelay(90 / portTICK_PERIOD_MS);
		if(i==1) begin_tone(BUZZ_PIN, 2500, 80);
		vTaskDelay(90 / portTICK_PERIOD_MS);
	}
}

