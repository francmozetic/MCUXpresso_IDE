#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_RAW && LWIP_SOCKET

#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "ethernetif.h"
#include "lwip/apps/mqtt.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "timers.h"
#include "fsl_rtc.h"
#include "fsl_adc16.h"
#include "fsl_port.h"
#include "fsl_device_registers.h"

#include <stdio.h>

/*************************************************************************************************
 * Definitions
 *
 *************************************************************************************************/
/* IP address configuration done. */
#define configIP_addr0 192
#define configIP_addr1 168
#define configIP_addr2 1
#define configIP_addr3 2

/* Netmask configuration done. */
#define configNM_addr0 255
#define configNM_addr1 255
#define configNM_addr2 255
#define configNM_addr3 0

/* Gateway address configuration done. */
#define configGW_addr0 192
#define configGW_addr1 168
#define configGW_addr2 1
#define configGW_addr3 1

/* IP address for mqtt server done. */
#define serverIP_addr0 192
#define serverIP_addr1 168
#define serverIP_addr2 1
#define serverIP_addr3 3

/* ADC16 configuration done. */
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 23U /* PTE30, ADC0_SE23 */
#define DEMO_ADC16_IRQ_ID ADC0_IRQn

/* Task priorities set. */
#define helloTaskPriority (configMAX_PRIORITIES - 1)
#define mqttTaskPriority (configMAX_PRIORITIES - 1)

/*************************************************************************************************
* Variables
*
**************************************************************************************************/
volatile uint32_t adcValue = 0;
volatile bool g_secsFlag = false;
volatile bool conversionCompleted = false;

TimerHandle_t xBacklightTimer = NULL;

mqtt_client_t *client1;
mqtt_client_t *client2;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;
uint32_t adcValues[5000];
char adcstr[32];
char bytestr[8192];

/*************************************************************************************************
 * Code
 *
 *************************************************************************************************/
void SetRtcClockSource(void)
{
    /* Enable the RTC 32KHz oscillator. */
    RTC->CR |= RTC_CR_OSCE_MASK;
}

void RTC_Seconds_IRQHandler(void)
{
    g_secsFlag = true;
}

void ADC0_IRQHandler(void)
{
    adcValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
    conversionCompleted = true;
}

void vBacklightTimerCallback( TimerHandle_t pxTimer )
{
	PRINTF("Timer callback period = 100 ms.\r\n");
}

static void mqtt_pub_request_cb(void *arg, err_t result)
{
	if (result == ERR_OK) {
		PRINTF("Publish request was successful.\r\n");
	}
	else {
		PRINTF("Publish result: %d.\r\n", result);
	}
}

static void mqtt_pub_request_cb_last(void *arg, err_t result)
{
	if (result == ERR_OK) {
		PRINTF("Publish request was successful.\r\n");

		xTimerStop(xBacklightTimer, 0);
	}
	else {
		PRINTF("Publish result: %d.\r\n", result);
	}
}

static void mqtt_sub_request_cb(void *arg, err_t result)
{
	if (result == ERR_OK) {
		PRINTF("Subscription request was successful.\r\n");

		xTimerStop(xBacklightTimer, 0);
	}
	else {
		PRINTF("Subscribe result: %d.\r\n", result);
	}
}

static void mqtt_connection1_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
	if (status == MQTT_CONNECT_ACCEPTED) {
		PRINTF("mqtt_connection1_cb: Successfully connected.\r\n");

		err_t err;
    	u8_t qos = 1;
    	u8_t retain = 0;

    	int D0val;

    	int i = 0;

    	sprintf(bytestr, "0 25000 1000");
        while (i < 1000) {
        	D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
            sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
            strcat(bytestr, adcstr);
        	i++;
        }

        err = mqtt_publish(client, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
    	if (err != ERR_OK) {
    		PRINTF("Failed to publish (0) to MQTT server, return code: %d.\r\n", err);
    	}

        sprintf(bytestr, "1 25000 1000");
        while (i < 2000) {
        	D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
            sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
            strcat(bytestr, adcstr);
        	i++;
        }

        err = mqtt_publish(client, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
    	if (err != ERR_OK) {
    		PRINTF("Failed to publish (1) to MQTT server, return code: %d.\r\n", err);
    	}

        sprintf(bytestr, "2 25000 1000");
        while (i < 3000) {
        	D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
            sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
            strcat(bytestr, adcstr);
        	i++;
        }

        err = mqtt_publish(client, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
    	if (err != ERR_OK) {
    		PRINTF("Failed to publish (2) to MQTT server, return code: %d.\r\n", err);
    	}

        sprintf(bytestr, "3 25000 1000");
        while (i < 4000) {
        	D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
            sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
            strcat(bytestr, adcstr);
        	i++;
        }

        err = mqtt_publish(client, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
    	if (err != ERR_OK) {
    		PRINTF("Failed to publish (3) to MQTT server, return code: %d.\r\n", err);
    	}

        sprintf(bytestr, "4 25000 1000");
        while (i < 5000) {
        	D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
            sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
            strcat(bytestr, adcstr);
        	i++;
        }

        err = mqtt_publish(client, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb_last, arg);
    	if (err != ERR_OK) {
    		PRINTF("Failed to publish (4) to MQTT server, return code: %d.\r\n", err);
    	}
	}
	else {
		PRINTF("mqtt_connection1_cb: Disconnected, reason: %d.\r\n", status);
	}
}

static void mqtt_task1(void *pvParameters)
{
	PRINTF("MQTT_task1.\r\n");
	err_t err;

	client1 = mqtt_client_new();

    if (client1 != NULL)
    {
        ip_addr_t *ip_addr;
        ip_addr = (ip_addr_t *)mem_malloc(sizeof(ip_addr_t));
        ip_addr->addr = ipaddr_addr("192.168.1.3");

    	/* Setup an empty client info structure and dynamically allocate memory for it. */
    	struct mqtt_connect_client_info_t *ci =
    			(struct mqtt_connect_client_info_t *)mem_malloc(sizeof(struct mqtt_connect_client_info_t));
    	memset(ci, 0, sizeof(struct mqtt_connect_client_info_t));

    	ci->client_id = "Klein2013";
    	ci->client_user = "galileo";
    	ci->client_pass = "teNeues2013";

    	err = mqtt_client_connect(client1, ip_addr, MQTT_PORT, mqtt_connection1_cb, 0, ci);
    	if	(err != ERR_OK)
    	{
    		PRINTF("Failed to connect to MQTT server, return code: %d.\r\n", err);
    	}

    	mem_free(ci);
    	mem_free(ip_addr);
    }
	vTaskSuspend(NULL);
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	PRINTF("Incoming publish at topic %s with total length %u.\r\n", topic, (unsigned int)tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
	char control_data[16];
	PRINTF("Incoming publish payload with length %d, flags %u.\r\n", length, (unsigned int)flags);
	if (length > 0) {
		PRINTF("mqtt_incoming_data_cb: %s.\r\n", (const char *)data);
		sprintf(control_data, "%s", (const char *)data);
		if (strcmp(control_data, "ClockDivider1") == 0) {
		    adc16ConfigStruct.clockDivider = kADC16_ClockDivider1;
		    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
		}
		if (strcmp(control_data, "ClockDivider2") == 0) {
		    adc16ConfigStruct.clockDivider = kADC16_ClockDivider2;
		    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
		}
		if (strcmp(control_data, "ClockDivider4") == 0) {
		    adc16ConfigStruct.clockDivider = kADC16_ClockDivider4;
		    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
		}
		if (strcmp(control_data, "ClockDivider8") == 0) {
		    adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
		    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
		}
		PRINTF("ADC16 conversion init.\r\n");

		int i = 0;

		while (i < 5000) {
			conversionCompleted = false;
			ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
			while (!conversionCompleted) {
			}
			adcValues[i] = adcValue;
			i++;
		}

		PRINTF("ADC16 conversion finished.\r\n");

		int D0val;
		err_t err;
		u8_t qos = 1;
		u8_t retain = 0;

		i = 0;

		sprintf(bytestr, "0 25000 1000");
		while (i < 1000) {
			D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
			sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
			strcat(bytestr, adcstr);
			i++;
		}

		err = mqtt_publish(client2, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
		if (err != ERR_OK) {
			PRINTF("Failed to publish (0) to MQTT server, return code: %d.\r\n", err);
		}

		sprintf(bytestr, "1 25000 1000");
		while (i < 2000) {
			D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
			sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
			strcat(bytestr, adcstr);
			i++;
		}

		err = mqtt_publish(client2, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
		if (err != ERR_OK) {
			PRINTF("Failed to publish (1) to MQTT server, return code: %d.\r\n", err);
		}

		sprintf(bytestr, "2 25000 1000");
		while (i < 3000) {
			D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
			sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
			strcat(bytestr, adcstr);
			i++;
		}

		err = mqtt_publish(client2, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
		if (err != ERR_OK) {
			PRINTF("Failed to publish (2) to MQTT server, return code: %d.\r\n", err);
		}

		sprintf(bytestr, "3 25000 1000");
		while (i < 4000) {
			D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
			sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
			strcat(bytestr, adcstr);
			i++;
		}

		err = mqtt_publish(client2, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
		if (err != ERR_OK) {
			PRINTF("Failed to publish (3) to MQTT server, return code: %d.\r\n", err);
		}

		sprintf(bytestr, "4 25000 1000");
		while (i < 5000) {
			D0val = (int)(adcValues[i] * (330000.0 / 65536.0));
			sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
			strcat(bytestr, adcstr);
			i++;
		}

		err = mqtt_publish(client2, "samples", bytestr, strlen(bytestr), qos, retain, mqtt_pub_request_cb, arg);
		if (err != ERR_OK) {
			PRINTF("Failed to publish (4) to MQTT server, return code: %d.\r\n", err);
		}

		PRINTF("ADC16 samples published.\r\n");
	}
}

static void mqtt_connection2_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
	if (status == MQTT_CONNECT_ACCEPTED) {
		PRINTF("mqtt_connection2_cb: Successfully connected.\r\n");

		err_t err;
    	u8_t qos = 1;

    	/* Setup callback for incoming publish requests. */
    	mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    	err = mqtt_sub_unsub(client, "control", qos, mqtt_sub_request_cb, arg, 1);
    	if (err != ERR_OK) {
    		PRINTF("Failed to subscribe to MQTT server, return code: %d.\r\n", err);
    	}
	}
	else {
		PRINTF("mqtt_connection2_cb: Disconnected, reason: %d.\r\n", status);
	}
}

static void mqtt_task2(void *pvParameters)
{
	PRINTF("MQTT_task2.\r\n");
	err_t err;

	client2 = mqtt_client_new();

    if (client2 != NULL)
    {
        ip_addr_t *ip_addr;
        ip_addr = (ip_addr_t *)mem_malloc(sizeof(ip_addr_t));
        ip_addr->addr = ipaddr_addr("192.168.1.3");

    	/* Setup an empty client info structure and dynamically allocate memory for it. */
    	struct mqtt_connect_client_info_t *ci =
    			(struct mqtt_connect_client_info_t *)mem_malloc(sizeof(struct mqtt_connect_client_info_t));
    	memset(ci, 0, sizeof(struct mqtt_connect_client_info_t));

    	ci->client_id = "Klein2016";
    	ci->client_user = "galileo";
    	ci->client_pass = "teNeues2013";

    	err = mqtt_client_connect(client2, ip_addr, MQTT_PORT, mqtt_connection2_cb, 0, ci);
    	if	(err != ERR_OK)
    	{
    		PRINTF("Failed to connect to MQTT server, return code: %d.\r\n", err);
    	}

    	mem_free(ci);
    	mem_free(ip_addr);
    }
	vTaskSuspend(NULL);
}

static void hello_task(void *pvParameters)
{
	PRINTF("Hello world.\r\n");
	vTaskSuspend(NULL);
}

/* @brief Main function
 */
int main(void)
{
	static struct netif fsl_netif0;
	ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

	rtc_config_t rtcConfig;
	rtc_datetime_t date;

	RTC_GetDefaultConfig(&rtcConfig);

	/* Ungates the RTC clock and configures the peripheral for basic operation. */
	RTC_Init(RTC, &rtcConfig);

    /* Select RTC clock source. */
    SetRtcClockSource();

	RTC_StopTimer(RTC);

	date.year   = 2016U;
	date.month  = 12U;
	date.day    = 25U;
	date.hour   = 0U;
	date.minute = 0U;
	date.second = 0U;

	RTC_SetDatetime(RTC, &date);

    NVIC_EnableIRQ(RTC_Seconds_IRQn);

	RTC_StartTimer(RTC);

    RTC_EnableInterrupts(RTC, kRTC_SecondsInterruptEnable);

    IP4_ADDR(&fsl_netif0_ipaddr, configIP_addr0, configIP_addr1, configIP_addr2, configIP_addr3);
    IP4_ADDR(&fsl_netif0_netmask, configNM_addr0, configNM_addr1, configNM_addr2, configNM_addr3);
    IP4_ADDR(&fsl_netif0_gw, configGW_addr0, configGW_addr1, configGW_addr2, configGW_addr3);

    tcpip_init(NULL, NULL);

    netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, NULL, ethernetif_init, tcpip_input);
    netif_set_default(&fsl_netif0);
    netif_set_up(&fsl_netif0);

    PRINTF("MQTT Client\r\n");
    PRINTF("IPv4 Address_____: %u.%u.%u.%u\r\n",
    		((u8_t *)&fsl_netif0_ipaddr)[0],
			((u8_t *)&fsl_netif0_ipaddr)[1],
			((u8_t *)&fsl_netif0_ipaddr)[2],
			((u8_t *)&fsl_netif0_ipaddr)[3]);
    PRINTF("IPv4 Subnet mask_: %u.%u.%u.%u\r\n",
    		((u8_t *)&fsl_netif0_netmask)[0],
			((u8_t *)&fsl_netif0_netmask)[1],
			((u8_t *)&fsl_netif0_netmask)[2],
			((u8_t *)&fsl_netif0_netmask)[3]);
    PRINTF("IPv4 Gateway_____: %u.%u.%u.%u\r\n",
    		((u8_t *)&fsl_netif0_gw)[0],
			((u8_t *)&fsl_netif0_gw)[1],
			((u8_t *)&fsl_netif0_gw)[2],
			((u8_t *)&fsl_netif0_gw)[3]);

	PRINTF("ADC16 conversion samples.\r\n");

    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
    adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
    adc16ConfigStruct.enableAsynchronousClock = true;
    adc16ConfigStruct.clockDivider = kADC16_ClockDivider4;
    adc16ConfigStruct.resolution = kADC16_Resolution16Bit;
    adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
    adc16ConfigStruct.enableHighSpeed = false;
    adc16ConfigStruct.enableLowPower = 1;
    adc16ConfigStruct.enableContinuousConversion = false;

    EnableIRQ(DEMO_ADC16_IRQ_ID);

    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);

    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false);

	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE)) {
    	PRINTF("ADC16 auto calibration successful.\r\n");
    }
    else {
    	PRINTF("ADC16 auto calibration failed.\r\n");
    }
	#endif

    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;
	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    	adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif

    /* Enable the clock to the PORT module that the LED is on. */
    CLOCK_EnableClock(kCLOCK_PortB);

    // Setup the red LED pin as GPIO
	PORT_SetPinMux(BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, kPORT_MuxAsGpio);
	PORT_SetPinMux(BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, kPORT_MuxAsGpio);

    // Initialize the RGB to ON/OFF condition
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);

    int count = 1;
    if (adc16ConfigStruct.clockDivider == kADC16_ClockDivider1) {
        count = 2;
    }
    if (adc16ConfigStruct.clockDivider == kADC16_ClockDivider2) {
        count = 4;
    }
    if (adc16ConfigStruct.clockDivider == kADC16_ClockDivider4) {
        count = 8;
    }
    if (adc16ConfigStruct.clockDivider == kADC16_ClockDivider8) {
        count = 16;
    }

    while (count != 0) {
    	if (g_secsFlag == true) {
    		LED_BLUE_TOGGLE();
    		g_secsFlag = false;
    		count = count - 1;
    	}
	}
    count = 1;
    while (count != 0) {
    	if (g_secsFlag == true) {
    		LED_RED_TOGGLE();
    		g_secsFlag = false;
    		count = count - 1;
    	}
	}

    for (int i=0; i<5000; ++i) {
    	conversionCompleted = false;
    	ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    	while (!conversionCompleted) {
    	}
    	adcValues[i] = adcValue;
    	/*
    	D0val = (int)(adcValue * (330000.0 / 65536.0));
        sprintf(adcstr, " %d.%05d", D0val / 100000, D0val % 100000);
        strcat(bytestr, adcstr);

        D0val = (float)(adcValue * (3.300 / 65536.0));
        PRINTF("ADC16 decimal = %0.3f\r\n", D0val);
        PRINTF("ADC16 decimal = %d\r\n", D0val);
        PRINTF("ADC16 decimal = %d.%03d\r\n", D0val / 1000, D0val % 1000); */
    }

    count = 1;
    while (count != 0) {
    	if (g_secsFlag == true) {
    		LED_RED_TOGGLE();
    		g_secsFlag = false;
    		count = count - 1;
    	}
	}

	RTC_StopTimer(RTC);

	xTaskCreate(hello_task, "Hello", configMINIMAL_STACK_SIZE, NULL, helloTaskPriority, NULL);
	xTaskCreate(mqtt_task1, "MQTT$Client$1", configMINIMAL_STACK_SIZE, NULL, mqttTaskPriority, NULL);
	xTaskCreate(mqtt_task2, "MQTT$Client$2", configMINIMAL_STACK_SIZE, NULL, mqttTaskPriority, NULL);

	xBacklightTimer = xTimerCreate
	(
		/* Just a text name, not used by the RTOS kernel. */
        "BacklightTimer",
        /* The period of the timer is set to 100ms. */
        ( 100 / portTICK_PERIOD_MS ),
		/* The timer will expire repeatedly. */
		pdTRUE,
		/* The id is not used by the callback so can take any value. */
		0,
		/* The callback function that switches the LCD back-light off. */
		vBacklightTimerCallback
	);

	if ( xBacklightTimer == NULL )
	{
	}
	else
	{
		/* Start the timer. No block time is specified, and even if one was
		 * it would be ignored because the RTOS scheduler has not yet been started. */
		if ( xTimerStart( xBacklightTimer, 0 ) != pdPASS )
		{
			/* The timer could not be set into the active state. */
		}
	}

	/* Start the real time scheduler. */

	vTaskStartScheduler();

	/* Will not get here unless a task calls vTaskEndScheduler(). */

    return 0;
}
#endif
