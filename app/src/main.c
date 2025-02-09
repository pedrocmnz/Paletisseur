/*
 * main.c
 *
 *  Created on: 01/04/2018
 *      Author: Laurent
 */

#include "main.h"

// Write
typedef struct
{
	uint8_t on_off; 		// Turn On or Off
	uint8_t actuator;		// The desired actuator
} commande_message_t;

// Read
typedef struct {
    uint8_t sem_id;        // Semaphore ID to use for publication
    uint8_t sensor_id;     // Awaited sensor ID
    uint8_t sensor_state;  // Awaited sensor state
} subscribe_message_t;

// Static functions
static uint8_t 	SystemClock_Config       (void);
void            ActivateActuator         (uint8_t bit);
void      	    DeactivateActuator       (uint8_t bit);
BaseType_t      subscribe                (uint8_t sem_id, uint8_t sensor_id, uint8_t sensor_state);
BaseType_t      writing                  (uint8_t on_off, uint8_t actuator);
static void     updateSubs               (subscribe_message_t *subs, subscribe_message_t *new_sub);
static void     print_subscription_table (subscribe_message_t *subs);
static void     publish                  (subscribe_message_t *subs);


// FreeRTOS tasks
void vTask_Palette   (void *pvParameters);
//void vTask_Clamper   (void *pvParameters);
void vTask_Write     (void *pvParameters);
void vTask_Read      (void *pvParameters);

// Kernel objects
xQueueHandle	 xCommandeQueue;
xQueueHandle     xSubscribeQueue;
xSemaphoreHandle sems[MAX_SEMAPHORE];
xSemaphoreHandle xSem_DMA_TC;
xSemaphoreHandle xConsoleMutex;

// Define Event Group flags
//#define	BIT_RED	    ( (EventBits_t)( 0x01 <<0) )   // This is not mandatory but it provides
//#define BIT_GREEN	( (EventBits_t)( 0x01 <<1) )   // friendly alias for individual event
//#define BIT_BLUE	( (EventBits_t)( 0x01 <<2) )

// Trace User Events Channels
// traceString ue1;

// Global variables
static uint32_t actuator_state = 0;

// Main function
int main()
{
	// Configure System Clock
	SystemClock_Config();

	// Initialize LED and PB pins
	BSP_LED_Init();
	BSP_PB_Init();

	// Initialize Debug Console
	BSP_Console_Init();
	my_printf("\r\nConsole Ready!\r\n");
	my_printf("SYSCLK = %d Hz\r\n", SystemCoreClock);

	// Initialize NVIC
	// BSP_NVIC_Init();

	// Start Trace Recording
	xTraceEnable(TRC_START);

	// Create Queue to hold console messages
	xCommandeQueue = xQueueCreate(MAX_SUBS, sizeof(commande_message_t));
	xSubscribeQueue = xQueueCreate(MAX_SUBS, sizeof(subscribe_message_t));

	// Give a name to the Queue in the trace recorder
	vTraceSetQueueName(xCommandeQueue, "Commande Queue");
	vTraceSetQueueName(xSubscribeQueue, "Subscribe Queue");

	// Create Semaphore object
	//xSem_DMA_TC =  xSemaphoreCreateBinary();
	for(uint8_t i = 0; i < MAX_SEMAPHORE; i++) {
		sems[i] = xSemaphoreCreateBinary();
	}

	// Give a name to the Semaphore in the trace recorder
	//vTraceSetSemaphoreName(xSem_DMA_TC, "xSemDMA_TC");

	// Create a Mutex for accessing the console
	//xConsoleMutex = xSemaphoreCreateMutex();

	// Give a nice name to the Mutex in the trace recorder
	//vTraceSetMutexName(xConsoleMutex, "Console Mutex");

	// Create Tasks
	xTaskCreate(vTask_Palette, "Task_Palette", 256, NULL, 1, NULL);
	//xTaskCreate(vTask_Clamper, "Task_Clamper", 256, NULL, 1, NULL);
	xTaskCreate(vTask_Read,    "Task_Read",    256, NULL, 2, NULL);
	xTaskCreate(vTask_Write,   "Task_Write",   256, NULL, 3, NULL);

	// Start the Scheduler
	vTaskStartScheduler();

	while(1)
	{
		// No code here
	}
}

/*
 *	Task to create and manage the palette location
 */

void vTask_Palette (void *pvParameters)
{
	while(1)
	{
		//Prepare the message and request for the activation of an actuator
		writing(1, distribuition_palette);
		writing(1, tapis_distribuition_palette);
		writing(1, tapis_palette_vers_ascenseur);

		vTaskDelay(300);

		writing(0, distribuition_palette);

		//while (FACTORY_IO_Sensors_Get(1 << entree_palette)==0);

		my_printf("Esperando sensor entree_palette \r\n");
		//Send a subscribe and wait for a response from the read task
		subscribe(1, entree_palette, 1);
		xSemaphoreTake(sems[entree_palette],portMAX_DELAY);

		writing(0, tapis_distribuition_palette);
		writing(0, tapis_palette_vers_ascenseur);
		writing(1, charger_palette);

		//while (FACTORY_IO_Sensors_Get(1 << sortie_palette)==0);

		my_printf("Esperando sensor sortie_palette");
		subscribe(1, sortie_palette, 1);
		xSemaphoreTake(sems[sortie_palette],portMAX_DELAY);

		writing(0, charger_palette);
		writing(1, monter_ascenseur);
		writing(1, ascenseur_to_limit);

		//while (FACTORY_IO_Sensors_Get(1 << ascenseur_etage_1)==0);

		subscribe(1, ascenseur_etage_1, 1);
		xSemaphoreTake(sems[ascenseur_etage_1],portMAX_DELAY);

		writing(0, monter_ascenseur);
		writing(0, ascenseur_to_limit);

		//while(IsSensorActive(porte_ouverte)==1);
		while(BSP_PB_GetState() == 0);

		writing(1, descendre_ascenseur);

		//while(IsSensorActive(porte_ouverte)==1);
		while(BSP_PB_GetState() == 1);

		writing(1, ascenseur_to_limit);

		//while (FACTORY_IO_Sensors_Get(1 << ascenseur_etage_rdc)==0);

		subscribe(1, ascenseur_etage_rdc, 1);
		xSemaphoreTake(sems[ascenseur_etage_rdc],portMAX_DELAY);

		writing(0, descendre_ascenseur);
		writing(0, ascenseur_to_limit);
		writing(1, charger_palette);
		writing(1, tapis_fin);
	}
}

/*
 *	Task to control the clamper and the door
 */
/*
void vTask_Clamper (void *pvParameters)
{
	while(1)
	{
		ActivateActuator(poussoir);

		while(BSP_PB_GetState() == 0);

		while(IsSensorActive(limite_poussoir)==0);
		DeactivateActuator(poussoir);
		ActivateActuator(clamp);
		while(IsSensorActive(clamped)==0);
		DeactivateActuator(clamp);

		ActivateActuator(porte);
		while(IsSensorActive(ascenseur_en_mouvement)==0);
		DeactivateActuator(porte);

		while(IsSensorActive(limite_porte)==1);
	}
}
*/

/*
 *	Task_Write
 */

void vTask_Write (void *pvParameters)
{
	// Set DMA interrupt priority
	//NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, configMAX_API_CALL_INTERRUPT_PRIORITY + 1);

	// Enable DMA interrupt on NVIC
	//NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

	// Read all states from the scene
	FACTORY_IO_update();

	//The removal of the palettes is done by humans and thus is always active
	ActivateActuator(remover);

	commande_message_t actuator;

	while(1)
	{
		// Wait for something in the message Queue
		xQueueReceive(xCommandeQueue, &actuator, portMAX_DELAY);

		// Take Mutex
		//xSemaphoreTake(xConsoleMutex, portMAX_DELAY);

		if(actuator.on_off == 1) {
			// Activate the actuator
			ActivateActuator(actuator.actuator);
		}

		if(actuator.on_off == 0) {
			// Deactivate the actuator
			DeactivateActuator(actuator.actuator);
		}
		// Release Mutex
		//xSemaphoreGive(xConsoleMutex);
	}
}

/*
 *	Task_Read
 */

void vTask_Read (void *pvParameters)
{
	// Set DMA interrupt priority on NVIC
	//NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, configMAX_API_CALL_INTERRUPT_PRIORITY + 1);

	// Enable DMA interrupt on NVIC
	//NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

	BaseType_t subscription_received;

	// Initialize timing
	portTickType xLastWakeTime = xTaskGetTickCount();

	subscribe_message_t subscription_table[MAX_SUBS] = {0};
	subscribe_message_t msg;
	size_t i;

	// Reseting the message
	for(i = 0; i < MAX_SUBS; i++) {
		subscription_table[i].sem_id = SUBSCRIPTION_EMPTY;
		subscription_table[i].sensor_id = SUBSCRIPTION_EMPTY;
		subscription_table[i].sensor_state = SUBSCRIPTION_EMPTY;
	}

	while(1)
	{
		// CHeck if there is something in the Subscribe Queue
		subscription_received = xQueueReceive(xSubscribeQueue, &msg, 0);

		if(subscription_received) {
			// Update the subscription table
			updateSubs(subscription_table, &msg);

			// Print the table in the console
			print_subscription_table(subscription_table);
		}
		// Take Mutex
		//xSemaphoreTake(xConsoleMutex, portMAX_DELAY);

		//Wait for sensor interrupt
		//xSemaphoreTake(xSem_DMA_TC, portMAX_DELAY);

		// Verify and tell the respective Task when the sensor triggers
		publish(subscription_table);

		// Release Mutex
		//xSemaphoreGive(xConsoleMutex);

		vTaskDelayUntil (&xLastWakeTime, (200/portTICK_RATE_MS));
	}
}


// Function to activate a specific actuator without modifying others
void ActivateActuator(uint8_t bit)
{
    actuator_state |= (1 << bit); // Set the specific bit to 1
    FACTORY_IO_Actuators_Set(actuator_state);
}

// Function to deactivate a specific actuator without modifying others
void DeactivateActuator(uint8_t bit)
{
    actuator_state &= ~(1 << bit); // Set the specific bit to 0
    FACTORY_IO_Actuators_Set(actuator_state);
}


BaseType_t writing(uint8_t on_off, uint8_t actuator)
{
	commande_message_t data = {
		.on_off = on_off,
		.actuator = actuator
	};

	return xQueueSendToBack(xCommandeQueue, &data, 0);
}


BaseType_t subscribe(uint8_t sem_id, uint8_t sensor_id, uint8_t sensor_state)
{
	subscribe_message_t data = {
		.sem_id = sem_id,
		.sensor_id = sensor_id,
		.sensor_state = sensor_state
	};

	return xQueueSendToBack(xSubscribeQueue, &data, 0);
}


static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub) {
    size_t i;

    my_printf("Subscribing...");

    // Check for duplicates
    for (i = 0; i < MAX_SUBS; i++) {
        if (subs[i].sem_id == new_sub->sem_id &&
            subs[i].sensor_id == new_sub->sensor_id &&
            subs[i].sensor_state == new_sub->sensor_state) {
            my_printf("Subscription already exists\r\n");
            return;
        }
    }

    // Add the new subscription to the first available slot
    for (i = 0; i < MAX_SUBS; i++) {
        if (subs[i].sensor_id == SUBSCRIPTION_EMPTY) {
            subs[i] = *new_sub;
            my_printf("Adding subscription in slot [%d]\r\n", i);
            return;
        }
    }

    // If the table is full
    my_printf("No available slots for new subscription\r\n");
}

static void print_subscription_table(subscribe_message_t *subs) {
    for (int i = 0; i < MAX_SUBS; i++) {
        my_printf("[%d] %d %d %d\r\n", i, subs[i].sem_id, subs[i].sensor_id, subs[i].sensor_state);
    }
}

/*
 * Function that
 */

static void publish(subscribe_message_t *subs)
{
    size_t i;

    // Go through all subscription slots
    for (i = 0; i < MAX_SUBS; i++) {
        uint8_t sensor_id = subs[i].sensor_id;
        // Get the actual state of the sensor using the mask (1 << sensor_id)
        uint8_t current_state = FACTORY_IO_Sensors_Get(1 << sensor_id);

        // If the sensor is still in the expected state, go to the next slot
        if (current_state != subs[i].sensor_state)
            continue;

        // If any changes happened, notify the subscriber to give the correnponding semaphore
        xSemaphoreGive(sems[subs[i].sensor_id]);
        my_printf("Published to subscription in slot [%d]\r\n", i);

        // Delete the subscription
        subs[i].sem_id = SUBSCRIPTION_EMPTY;
        subs[i].sensor_id = SUBSCRIPTION_EMPTY;
        subs[i].sensor_state = SUBSCRIPTION_EMPTY;
    }
}

/*
 * 	Clock configuration for the Nucleo STM32F072RB board
 * 	HSE input Bypass Mode 			-> 8MHz
 * 	SYSCLK, AHB, APB1 			-> 48MHz
 *  	PA8 as MCO with /16 prescaler 		-> 3MHz
 *
 *  Laurent Latorre - 05/08/2017
 */
static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout;

	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1);	// HSE error


	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;

	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PLL error


	// Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

	// Enable FLASH Prefetch Buffer and set Flash Latency (required for high speed)
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL becomes main switch input
	timeout = 1000;

	do
	{
		status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3);	// SW error


	// Set MCO source as SYSCLK (48MHz)
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOSEL_SYSCLK;

	// Set MCO prescaler to /16 -> 3MHz
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV16;

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA8 as Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);

	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);

	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
	return (0);
}



/*
 * Assertion Handler
 */
void vAssertCalled( char *file, int line )
{
	taskDISABLE_INTERRUPTS();

	my_printf("Assertion Failed\r\n");
	my_printf("File %s\r\n", file);
	my_printf("Line %d\r\n", line);

	while(1);
}


/*
 * Malloc failed Basic Hook
 */
void vApplicationMallocFailedHook()
{
	my_printf("Malloc Failed\r\n");

	while(1);
}
