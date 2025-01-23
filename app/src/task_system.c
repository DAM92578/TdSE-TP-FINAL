/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @file   : task_system.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes. */
#include "main.h"

/* Demo includes. */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes. */
#include "board.h"
#include "app.h"

#include "task_system_attribute.h"
#include "task_system_interface.h"

#include "task_actuator_attribute.h"
#include "task_actuator_interface.h"

#include "display.h"
#include "task_adc_interface.h"
#include "task_adc_attribute.h"
/********************** macros and definitions *******************************/
#define G_TASK_SYS_CNT_INI			0ul
#define G_TASK_SYS_TICK_CNT_INI		0ul

#define DEL_SYS_XX_MIN				0ul
#define DEL_SYS_XX_MED				50ul
#define DEL_SYS_XX_MAX				500ul

////////////////////////// Estos son placeholders los tiene que levantar del set up menu ////////////////
#define MAX_TICK_ALARM				2000ul
#define MAX_TICK_SWITCH				10000ul
#define TEMP_MAX_USER				50

/********************** internal data declaration ****************************/
task_system_dta_t task_system_dta =
	{DEL_SYS_XX_MIN, ST_SYS_XX_MONITOR, EV_SYS_BTN_ON_IDLE, false};

#define SYSTEM_DTA_QTY	(sizeof(task_system_dta)/sizeof(task_system_dta_t))

uint32_t counter_tick= 0;
bool 	switch_motors_flag = false;
/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/
const char *p_task_system 		= "Task System (System Statechart)";
const char *p_task_system_ 		= "Non-Blocking & Update By Time Code";

uint32_t temp_amb_raw=0;
uint32_t temp_uC_raw=0;
uint32_t temp_amb=0;
uint32_t temp_uC=0;


/********************** external data declaration ****************************/
uint32_t g_task_system_cnt;
volatile uint32_t g_task_system_tick_cnt;

/********************** external functions definition ************************/
void task_system_init(void *parameters)
{
	task_system_dta_t 	*p_task_system_dta;
	task_system_st_t	state;
	task_system_ev_t	event;
	bool b_event;

	/* Print out: Task Initialized */
	LOGGER_LOG("  %s is running - %s\r\n", GET_NAME(task_system_init), p_task_system);
	LOGGER_LOG("  %s is a %s\r\n", GET_NAME(task_system), p_task_system_);

	g_task_system_cnt = G_TASK_SYS_CNT_INI;

	/* Print out: Task execution counter */
	LOGGER_LOG("   %s = %lu\r\n", GET_NAME(g_task_system_cnt), g_task_system_cnt);

	init_queue_event_task_system();

	/* Update Task Actuator Configuration & Data Pointer */
	p_task_system_dta = &task_system_dta;

	/* Print out: Task execution FSM */
	state = p_task_system_dta->state;
	LOGGER_LOG("   %s = %lu", GET_NAME(state), (uint32_t)state);

	event = p_task_system_dta->event;
	LOGGER_LOG("   %s = %lu", GET_NAME(event), (uint32_t)event);

	b_event = p_task_system_dta->flag;
	LOGGER_LOG("   %s = %s\r\n", GET_NAME(b_event), (b_event ? "true" : "false"));

	g_task_system_tick_cnt = G_TASK_SYS_TICK_CNT_INI;
}

void task_system_update(void *parameters)
{
	task_system_dta_t *p_task_system_dta;
	bool b_time_update_required = false;

    char display_str[16];


	/* Update Task System Counter */
	g_task_system_cnt++;

	/* Protect shared resource (g_task_system_tick) */
	__asm("CPSID i");	/* disable interrupts*/
    if (G_TASK_SYS_TICK_CNT_INI < g_task_system_tick_cnt)
    {
    	g_task_system_tick_cnt--;
    	b_time_update_required = true;
    }
    __asm("CPSIE i");	/* enable interrupts*/

    while (b_time_update_required)
    {
		/* Protect shared resource (g_task_system_tick) */
		__asm("CPSID i");	/* disable interrupts*/
		if (G_TASK_SYS_TICK_CNT_INI < g_task_system_tick_cnt)
		{
			g_task_system_tick_cnt--;
			b_time_update_required = true;
		}
		else
		{
			b_time_update_required = false;
		}
		__asm("CPSIE i");	/* enable interrupts*/

    	/* Update Task System Data Pointer */
		p_task_system_dta = &task_system_dta;

		if (true == any_event_task_system())
		{
			p_task_system_dta->flag = true;
			p_task_system_dta->event = get_event_task_system();
		}

		//// ACA LEVANTAR EL ADC Y LIMITE DEL USUARIO -> PASAR A EV_SYS_FAILURE_ACTIVE SI PASA EL LIMITE DEL USUARIO

		if ( true == any_value_task_adc())
		{
			temp_uC_raw  = get_value_task_adc();
			temp_amb_raw = get_value_task_adc();

			temp_amb = (3.30 * 100 * temp_amb_raw)/(4096);
			LOGGER_LOG("temp_uC_raw:%lu\r\n",temp_uC_raw);
			LOGGER_LOG("temp_amb_raw:%lu\r\n",temp_amb);

			if( temp_amb > TEMP_MAX_USER ){
				p_task_system_dta->event = EV_SYS_FAILURE_ACTIVE;
			}
		}

		 if(HAL_GPIO_ReadPin(SWITCH_FAILURE_PORT, SWITCH_FAILURE_PIN) == SWITCH_FAILURE_PRESSED)
		 {
		 	p_task_system_dta->event = EV_SYS_FAILURE_ACTIVE;
		 }

		/// verifico que el switch de off no este activado si lo esta overwrite de evento a off.

		 if(HAL_GPIO_ReadPin(SWITCH_OFF_PORT, SWITCH_OFF_PIN) == SWITCH_OFF_HOVER)
		 {
		 	p_task_system_dta->event = EV_SYS_SWITCH_OFF_ACTIVE;
		 }

		switch (p_task_system_dta->state)
		{
			case ST_SYS_XX_OFF:
				//if (true == p_task_system_dta->flag)
				{
					p_task_system_dta->flag = false;
					if(HAL_GPIO_ReadPin(SWITCH_OFF_PORT, SWITCH_OFF_PIN) == SWITCH_OFF_PRESSED){

						put_event_task_actuator(EV_LED_XX_ON, ID_LED_USER_A);
						put_event_task_actuator(EV_LED_XX_BLINK, ID_LED_AIRE_A);
						p_task_system_dta->tick = MAX_TICK_SWITCH;
						p_task_system_dta->state = ST_SYS_XX_MONITOR;

					}
				}
				break;

			case ST_SYS_XX_IDLE: //el usuario esta interactuando con el menu solo sale del menu cuando detecte el btn on active ignora todo excepto el switch off

				//if (true == p_task_system_dta->flag)
				{

					p_task_system_dta->flag = false;
					switch(p_task_system_dta->event)
					{
						case EV_SYS_SWITCH_OFF_ACTIVE:	//apagar todo

							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_USER_B);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_USER_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_B);
							put_event_task_actuator(EV_LED_XX_OFF, ID_BUZZER);
							p_task_system_dta->state = ST_SYS_XX_OFF;
							break;

						case EV_SYS_BTN_ON_ACTIVE: //levanta la nueva temp del usuario y pasa a state monitor

							//LEVANTAR NUEVA TEMP MAX DE USUARIO y tiempo de switch

							//reinicio a sistema inicial prendo el aire A
							put_event_task_actuator(EV_LED_XX_ON, ID_LED_USER_A);
							put_event_task_actuator(EV_LED_XX_BLINK, ID_LED_AIRE_A);

							//apago la luz de config de B
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_USER_B);

							//reinicio el clock y paso a estado de monitoreo
							p_task_system_dta->tick = MAX_TICK_SWITCH;
							p_task_system_dta->state = ST_SYS_XX_MONITOR;
							break;

						default: // mientras el usuario esta en menu parpadea las luces de usuario y deja todo lo demas apagado

							put_event_task_actuator(EV_LED_XX_BLINK, ID_LED_USER_B);
							put_event_task_actuator(EV_LED_XX_BLINK, ID_LED_USER_A);

							break;
					}
				}
				break;

			case ST_SYS_XX_ACTIVE:
			//	if (true == p_task_system_dta->flag)
				{
					p_task_system_dta->flag = false;
					switch(p_task_system_dta->event){

						case EV_SYS_SWITCH_OFF_ACTIVE://apagar todo

							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_USER_B);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_USER_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_B);
							put_event_task_actuator(EV_LED_XX_OFF, ID_BUZZER);
							p_task_system_dta->state = ST_SYS_XX_OFF;
							break;

						case EV_SYS_FAILURE_IDLE:
						case EV_SYS_SWITCH_AIRE_IDLE:
							if(p_task_system_dta->tick == 0 && !switch_motors_flag){
								p_task_system_dta->tick = MAX_TICK_SWITCH; // ACA restarle el tiempo de falla al reloj de conmutacion
								p_task_system_dta->state = ST_SYS_XX_MONITOR;
							}
							break;

						case EV_SYS_FAILURE_ACTIVE: //si el cambio de motores fue por una falla prendo el buzzer
						case EV_SYS_SWITCH_AIRE_ACTIVE:
							if(p_task_system_dta->tick == 0){
								put_event_task_actuator(EV_LED_XX_PULSE, ID_BUZZER);
							}

						default:
							if(p_task_system_dta->tick > 0)
							{
								p_task_system_dta->tick--;
							}
							else
							{
								//hago el switch de motores y reinicio el tiempo al seteado por el usuario
								if (HAL_GPIO_ReadPin(LED_AIRE_B_PORT, LED_AIRE_B_PIN) == LED_AIRE_B_ON && HAL_GPIO_ReadPin(SWITCH_AIRE_B_PORT, SWITCH_AIRE_B_PIN) == SWITCH_AIRE_B_PRESSED )
								{ //si el aire B esta prendido y habilitado por su switch

									put_event_task_actuator(EV_LED_XX_OFF,ID_LED_USER_B);
									put_event_task_actuator(EV_LED_XX_ON,ID_LED_USER_A);

									put_event_task_actuator(EV_LED_XX_OFF,ID_LED_AIRE_B);
									put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_AIRE_A);

									p_task_system_dta->tick = MAX_TICK_SWITCH;
									p_task_system_dta->state = ST_SYS_XX_MONITOR;
								}
								else if (HAL_GPIO_ReadPin(SWITCH_AIRE_A_PORT, SWITCH_AIRE_A_PIN) == SWITCH_AIRE_A_PRESSED )
								{
									put_event_task_actuator(EV_LED_XX_ON,ID_LED_USER_B);
									put_event_task_actuator(EV_LED_XX_OFF,ID_LED_USER_A);

									put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_AIRE_B);
									put_event_task_actuator(EV_LED_XX_OFF,ID_LED_AIRE_A);

									p_task_system_dta->tick = MAX_TICK_SWITCH;
									p_task_system_dta->state = ST_SYS_XX_MONITOR;
								}
								else {
									// ningun aire funciona prendo el buzzer y doy un mensaje por display
									put_event_task_actuator(EV_LED_XX_PULSE, ID_BUZZER);
						     	 	displayCharPositionWrite(0, 0);
						     	 	displayStringWrite("ERROR: AC NOT FOUND");
								}

							}
							break;
					}
				}
				break;

			default: // modo de monitoreo

				//if(true == p_task_system_dta->flag) aca no solo deberia entrar cuando se pulsa un sensor
				{

					p_task_system_dta->flag = false;

					switch(p_task_system_dta->event) //si se activa el switch de falla comienza a contar el tiempo de falla
					{
						case EV_SYS_SWITCH_OFF_ACTIVE: // apaga todo

							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_USER_B);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_USER_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_B);
							put_event_task_actuator(EV_LED_XX_OFF, ID_BUZZER);
							p_task_system_dta->state = ST_SYS_XX_OFF;
							break;

						//case EV_MEN_ENT_ACTIVE:
						case EV_SYS_BTN_ON_ACTIVE:
							// entro en estado idle mientras se usa el menu apago todo y pongo las leds de user en modo config (blink)

							put_event_task_actuator(EV_LED_XX_BLINK, ID_LED_USER_B);
							put_event_task_actuator(EV_LED_XX_BLINK, ID_LED_USER_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_A);
							put_event_task_actuator(EV_LED_XX_OFF, ID_LED_AIRE_B);
							put_event_task_actuator(EV_LED_XX_OFF, ID_BUZZER);

							p_task_system_dta->state = ST_SYS_XX_IDLE;
							break;


						case EV_SYS_SWITCH_AIRE_ACTIVE:
						case EV_SYS_FAILURE_ACTIVE:
							//entrar a estado de espera y pasar a estado active
							p_task_system_dta->tick = MAX_TICK_ALARM;
							p_task_system_dta->state = ST_SYS_XX_ACTIVE;
							break;

						default:
							if(p_task_system_dta->tick > 0)
							{
								//parpadea el aire que este prendido y actualiza el display
								if (HAL_GPIO_ReadPin(LED_AIRE_B_PORT, LED_AIRE_B_PIN) == LED_AIRE_B_ON ){ //si el aire B esta prendido

								//	put_event_task_actuator(EV_LED_XX_ON,ID_LED_USER_B);
									put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_AIRE_B);
								}
								else{
								//	put_event_task_actuator(EV_LED_XX_ON,ID_LED_USER_A);
									put_event_task_actuator(EV_LED_XX_BLINK,ID_LED_AIRE_A);
								}
								p_task_system_dta->tick--;
							}
							else{
								switch_motors_flag = true;
								p_task_system_dta->state = ST_SYS_XX_ACTIVE;
							}

							//////////////// ACTUALIZA DISPLAY ////////////////////

							//LOGGER_LOG("temp_uC_raw:%lu\r\n",temp_uC_raw);
							//LOGGER_LOG("temp_amb_raw:%lu\r\n",temp_amb_raw);

							displayCharPositionWrite(0, 0);
							temp_amb = (3.30 * 100 * temp_amb_raw)/(4096);
							temp_uC  = ((1700-temp_uC_raw)/4.3 )+25;


							snprintf(display_str, sizeof(display_str),"Ent/Nxt T uC:%lu ",temp_uC);
							displayStringWrite(display_str);

							displayCharPositionWrite(0,1);
							snprintf(display_str, sizeof(display_str),"Tamb:%lu Tset:%i ",temp_amb,TEMP_MAX_USER);//temp_amb,p_task_menu_set_up_dta->set_point_temperatura);
							displayStringWrite(display_str);

							break;
					}// switch
				}// if(true == p_task_system_dta->flag)
					break;
			}// MAIN SWITCH
		}// MAIN WHILE
    }// END OF FUNCTION



/********************** end of file ******************************************/
