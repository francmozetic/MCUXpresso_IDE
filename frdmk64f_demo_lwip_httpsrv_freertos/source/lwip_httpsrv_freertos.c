/* Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*************************************************************************************************
 --> Includes
 *************************************************************************************************/
#include "lwip/opt.h"

#if LWIP_SOCKET
#include "lwip/tcpip.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "netif/ethernet.h"
#include "ethernetif.h"

#include "tcpecho/tcpecho.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_rtc.h"
#include "fsl_port.h"
#include "fsl_adc16.h"
#include "fsl_device_registers.h"

/*************************************************************************************************
 --> Definitions
 *************************************************************************************************/
#define USE_DHCP	0

/* IP address configuration. */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 1
#define configIP_ADDR3 2

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Gateway address configuration. */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
#define configGW_ADDR2 1
#define configGW_ADDR3 1

/* IP address for MQTT server. */
#define serverIP_addr0 192
#define serverIP_addr1 168
#define serverIP_addr2 1
#define serverIP_addr3 3

#define configPHY_ADDRESS 1

#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 23U /* PTE30, ADC0_SE23 */
#define DEMO_ADC16_IRQ_ID ADC0_IRQn

#define helloTaskPriority (configMAX_PRIORITIES - 1)
#define mqttTaskPriority (configMAX_PRIORITIES - 1)

#ifndef HTTP_STACKSIZE
#define HTTP_STACKSIZE DEFAULT_THREAD_STACKSIZE
#endif
#ifndef HTTP_PRIORITY
#define HTTP_PRIORITY DEFAULT_THREAD_PRIO
#endif
#ifndef DEBUG_WS
#define DEBUG_WS 0
#endif

volatile uint32_t adcValue = 0;
volatile bool g_secsFlag = false;
volatile bool conversionCompleted = false;

uint32_t adcValues[10000];
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;

/*************************************************************************************************
 --> Code
 *************************************************************************************************/
void SetRtcClockSource(void)
{
    /* Enable the RTC 32KHz oscillator. */
    RTC->CR |= RTC_CR_OSCE_MASK;
}

/* @brief RTC Interrupt handler
 */
void RTC_Seconds_IRQHandler(void)
{
    g_secsFlag = true;
}

/* @brief ADC Interrupt handler
 */
void ADC0_IRQHandler(void)
{
    adcValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
    conversionCompleted = true;
}

#if USE_DHCP
/* @brief Prints DHCP status of the interface when it has changed from last status.
 */
static void print_dhcp_state(struct netif *netif)
{
   static u8_t dhcp_last_state = DHCP_STATE_OFF;
   struct dhcp *dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

   if (dhcp_last_state != dhcp->state)
   {
       dhcp_last_state = dhcp->state;

       PRINTF("DHCP state       : ");
       switch (dhcp_last_state)
       {
           case DHCP_STATE_OFF:
               PRINTF("OFF");
               break;
           case DHCP_STATE_REQUESTING:
               PRINTF("REQUESTING");
               break;
           case DHCP_STATE_INIT:
               PRINTF("INIT");
               break;
           case DHCP_STATE_REBOOTING:
               PRINTF("REBOOTING");
               break;
           case DHCP_STATE_REBINDING:
               PRINTF("REBINDING");
               break;
           case DHCP_STATE_RENEWING:
               PRINTF("RENEWING");
               break;
           case DHCP_STATE_SELECTING:
               PRINTF("SELECTING");
               break;
           case DHCP_STATE_INFORMING:
               PRINTF("INFORMING");
               break;
           case DHCP_STATE_CHECKING:
               PRINTF("CHECKING");
               break;
           case DHCP_STATE_BOUND:
               PRINTF("BOUND");
               break;
           case DHCP_STATE_BACKING_OFF:
               PRINTF("BACKING_OFF");
               break;
           default:
               PRINTF("%u", dhcp_last_state);
               assert(0);
               break;
       }

       if (dhcp_last_state == DHCP_STATE_BOUND)
       {
           PRINTF("IPv4 Address_____: %u.%u.%u.%u", ((u8_t *)&netif->ip_addr.addr)[0], ((u8_t *)&netif->ip_addr.addr)[1],
        		   ((u8_t *)&netif->ip_addr.addr)[2], ((u8_t *)&netif->ip_addr.addr)[3]);
           PRINTF("IPv4 Subnet mask_: %u.%u.%u.%u", ((u8_t *)&netif->netmask.addr)[0], ((u8_t *)&netif->netmask.addr)[1],
        		   ((u8_t *)&netif->netmask.addr)[2], ((u8_t *)&netif->netmask.addr)[3]);
           PRINTF("IPv4 Gateway_____: %u.%u.%u.%u\r\n", ((u8_t *)&netif->gw.addr)[0], ((u8_t *)&netif->gw.addr)[1],
        		   ((u8_t *)&netif->gw.addr)[2], ((u8_t *)&netif->gw.addr)[3]);
       }
   }
}

/* @brief Initializes lwIP stack.
 */
static void dhcp_init(void)
{
    static struct netif fsl_netif0; /* network interface structure */
    ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;

    PRINTF("Setting interrupt priorities\r\n");
    NVIC_SetPriority(ENET_1588_Timer_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(ENET_Transmit_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(ENET_Receive_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    tcpip_init(NULL, NULL);

    IP4_ADDR(&fsl_netif0_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&fsl_netif0_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&fsl_netif0_gw, 0U, 0U, 0U, 0U);

    PRINTF("Adding interfaces\r\n");
    netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, NULL, ethernetif_init, ethernet_input);
    netif_set_default(&fsl_netif0);
    netif_set_up(&fsl_netif0);

    dhcp_start(&fsl_netif0);

    while (1)
    {
        /* Poll the driver and get any outstanding frames. */
        ethernetif_input(&fsl_netif0);

        /* Handle all system timeouts for all core protocols. */

        /* Print DHCP progress. */
        print_dhcp_state(&fsl_netif0);
    }

    tcpip_init(NULL, NULL);

    netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, NULL, ethernetif_init, tcpip_input);
    netif_set_default(&fsl_netif0);
    netif_set_up(&fsl_netif0);

    LWIP_PLATFORM_DIAG(("IPv4 Address_____: %u.%u.%u.%u", ((u8_t *)&fsl_netif0_ipaddr)[0], ((u8_t *)&fsl_netif0_ipaddr)[1],
    		((u8_t *)&fsl_netif0_ipaddr)[2], ((u8_t *)&fsl_netif0_ipaddr)[3]));
    LWIP_PLATFORM_DIAG(("IPv4 Subnet mask_: %u.%u.%u.%u", ((u8_t *)&fsl_netif0_netmask)[0], ((u8_t *)&fsl_netif0_netmask)[1],
    		((u8_t *)&fsl_netif0_netmask)[2], ((u8_t *)&fsl_netif0_netmask)[3]));
    LWIP_PLATFORM_DIAG(("IPv4 Gateway_____: %u.%u.%u.%u", ((u8_t *)&fsl_netif0_gw)[0], ((u8_t *)&fsl_netif0_gw)[1],
    		((u8_t *)&fsl_netif0_gw)[2], ((u8_t *)&fsl_netif0_gw)[3]));
}

/* @brief The main function containing server thread.
 */
static void dhcp_thread(void *arg)
{
    LWIP_UNUSED_ARG(arg);

    dhcp_init();

    vTaskDelete(NULL);
}
#else

/* @brief Initializes lwIP stack.
 */
static void stack_init(void)
{
    static struct netif fsl_netif0;
    ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;

    tcpip_init(NULL, NULL);

    IP4_ADDR(&fsl_netif0_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&fsl_netif0_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&fsl_netif0_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, NULL, ethernetif_init, tcpip_input);
    netif_set_default(&fsl_netif0);
    netif_set_up(&fsl_netif0);

    LWIP_PLATFORM_DIAG(("IPv4 Address_____: %u.%u.%u.%u", ((u8_t *)&fsl_netif0_ipaddr)[0], ((u8_t *)&fsl_netif0_ipaddr)[1],
    		((u8_t *)&fsl_netif0_ipaddr)[2], ((u8_t *)&fsl_netif0_ipaddr)[3]));
    LWIP_PLATFORM_DIAG(("IPv4 Subnet mask_: %u.%u.%u.%u", ((u8_t *)&fsl_netif0_netmask)[0], ((u8_t *)&fsl_netif0_netmask)[1],
    		((u8_t *)&fsl_netif0_netmask)[2], ((u8_t *)&fsl_netif0_netmask)[3]));
    LWIP_PLATFORM_DIAG(("IPv4 Gateway_____: %u.%u.%u.%u", ((u8_t *)&fsl_netif0_gw)[0], ((u8_t *)&fsl_netif0_gw)[1],
    		((u8_t *)&fsl_netif0_gw)[2], ((u8_t *)&fsl_netif0_gw)[3]));
}

/* @brief The main function containing server thread.
 */
static void main_thread(void *arg)
{
    LWIP_UNUSED_ARG(arg);

    stack_init();

    tcpecho_init();

    vTaskDelete(NULL);
}
#endif

static void hello_task(void *pvParameters)
{
    PRINTF("Hello world.\r\n");
    vTaskSuspend(NULL);
}

static void light1_task(void *pvParameters)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)pvParameters;

    if (sem == NULL) { /* should not be NULL */
        for(;;){}
    }

    /* it's an infinite loop */
    for(;;) {
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdPASS) {
            PRINTF("xSemaphoreTake #1.\r\n");
        }
    }
}

static void light2_task(void *pvParameters)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)pvParameters;

    if (sem == NULL) { /* should not be NULL */
        for(;;){}
    }

    /* it's an infinite loop */
    for(;;) {
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdPASS) {
            PRINTF("xSemaphoreTake #2.\r\n");
        }
    }
}

static void light3_task(void *pvParameters)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)pvParameters;

    if (sem == NULL) { /* should not be NULL */
        for(;;){}
    }

    /* it's an infinite loop */
    for(;;) {
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdPASS) {
            PRINTF("xSemaphoreTake #3.\r\n");
        }
    }
}

static void master_task(void *pvParameters)
{
    (void)pvParameters; /* parameters not used */

    SemaphoreHandle_t sem = xSemaphoreCreateBinary();
    if (sem == NULL) { /* semaphore creation failed */
        for(;;){} /* error */
    }
    vQueueAddToRegistry(sem, "IPC_Sem");

    if (xTaskCreate(light1_task, "Light", 500/sizeof(StackType_t), sem, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
        for(;;){} /* error */
    }
    if (xTaskCreate(light2_task, "Light", 500/sizeof(StackType_t), sem, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
        for(;;){} /* error */
    }
    if (xTaskCreate(light3_task, "Light", 500/sizeof(StackType_t), sem, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
        for(;;){} /* error */
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* it's an infinite loop */
    for(;;) {
        (void)xSemaphoreGive(sem); /* give control to other tasks */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* @brief Main function.
 */
int main(void)
{
    SYSMPU_Type *base = SYSMPU;
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;

    rtc_config_t rtcConfig;
    rtc_datetime_t date;

    RTC_GetDefaultConfig(&rtcConfig);

    /* Ungates the RTC clock and configures the peripheral for basic operation. */
    RTC_Init(RTC, &rtcConfig);

    /* Select RTC clock source. */
    SetRtcClockSource();

    RTC_StopTimer(RTC);

    date.year = 2018U;
    date.month = 11U;
    date.day = 7U;
    date.hour = 0U;
    date.minute = 0U;
    date.second = 0U;

    RTC_SetDatetime(RTC, &date);

    NVIC_EnableIRQ(RTC_Seconds_IRQn);

    RTC_StartTimer(RTC);

    RTC_EnableInterrupts(RTC, kRTC_SecondsInterruptEnable);

    PRINTF("ADC16 conversion samples.\r\n");

    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
    adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
    adc16ConfigStruct.enableAsynchronousClock = true;
    adc16ConfigStruct.clockDivider = kADC16_ClockDivider1;
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
    PRINTF("ADC16 enable interrupt on conversion completed.\r\n");
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

    for (int i=0; i<10000; ++i) {
    	conversionCompleted = false;
    	ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    	while (!conversionCompleted) {}
    	adcValues[i] = adcValue;
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

    xTaskCreate(master_task, "Master", 500/sizeof(StackType_t), NULL, tskIDLE_PRIORITY+1, NULL);

    /* Initialize lwIP from thread. */
    #if USE_DHCP
    if(sys_thread_new("main", dhcp_thread, NULL, HTTP_STACKSIZE, HTTP_PRIORITY) == NULL) {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }
    #else
    if(sys_thread_new("main", main_thread, NULL, HTTP_STACKSIZE, HTTP_PRIORITY) == NULL) {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }
    #endif

	/* Start the real time scheduler. */
	vTaskStartScheduler();

	/* Will not get here unless a task calls vTaskEndScheduler(). */

	return 0;
}

#endif // LWIP_SOCKET
