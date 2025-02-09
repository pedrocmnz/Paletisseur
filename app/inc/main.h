/*
 * main.h
 *
 *  Created on: Dec 24, 2023
 *      Author: Laurent
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_

#define MAX_SUBS 12                 // Maximun number of actutors request at the same time
#define SUBSCRIPTION_EMPTY 0xFF		// Correction for the case whith sensor 0
#define MAX_SEMAPHORE 20            // Number of subscribe semaphores

//Sensors Up_Task
#define CARTONDISTRIBUE     0
#define CARTONENVOYE        1
#define ENTREEPALETIZOR     2
#define PORTEOUVERTE		3
#define LIMITEPOUSSOIR		4
#define CLAMPED				5

//Actuators Up_Task
#define CARTON              0
#define TAPISCARTON         1
#define BLOCAGE             2
#define PORTE				3
#define POUSSOIR			4
#define CLAMP				5
#define TAPISCARTONPALET    12
#define CHARGERPALET		16

//Sensors Palette
#define ascenseur_etage_rdc 6
#define entree_palette 13
#define sortie_palette 10
#define ascenseur_en_mouvement 12
#define ascenseur_etage_2 9
#define ascenseur_etage_1 8

//Actuators Palette
#define distribuition_palette 10
#define tapis_distribuition_palette 19
#define tapis_palette_vers_ascenseur 18
#define decharger_palette 17
#define charger_palette 11
#define descendre_ascenseur 8
#define monter_ascenseur 6
#define ascenseur_to_limit 9
#define tapis_fin 20
#define remover 21

//Sensors Clamper
#define limite_poussoir 4
#define clamped 5
#define limite_porte 11
#define porte_ouverte 3

//Actuators Clamper
#define poussoir 4
#define clamp 5
#define porte 3

// Device header
#include "stm32f0xx.h"

// BSP functions
#include "bsp.h"

// FreeRTOS headers
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "factory_io.h"
#include "delay.h"
#include <string.h>

// Global functions
int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);

#endif /* INC_MAIN_H_ */
