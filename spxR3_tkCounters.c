/*
 * spx_counters.c
 *
 *
 */

#include "spxR3.h"

BaseType_t xHigherPriorityTaskWokenDigital = pdFALSE;

static st_counter_inputs_t cframe;

static bool wakeup_for_C0, wakeup_for_C1;

static void pv_tkCounter_init(void);

// La tarea puede estar hasta 10s en standby
#define WDG_COUNT_TIMEOUT	30

//------------------------------------------------------------------------------------
void tkCounter(void * pvParameters)
{

( void ) pvParameters;
uint32_t ulNotificationValue;
const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10000 );

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	pv_tkCounter_init();

	xprintf_P( PSTR("starting tkCounter..\r\n\0"));

	// loop
	for( ;; )
	{

		// Paso c/10s plt 30s es suficiente.
		pub_ctl_watchdog_kick(WDG_COUNT, WDG_COUNT_TIMEOUT);

		// Cuando la interrupcion detecta un flanco, solo envia una notificacion
		// Espero que me avisen. Si no me avisaron en 10s salgo y repito el ciclo.
		// Esto es lo que me permite entrar en tickless.
		ulNotificationValue = ulTaskNotifyTake( pdFALSE, xMaxBlockTime );

		if( ulNotificationValue != 0 ) {
			// Fui notificado: llego algun flanco que determino.

			if ( wakeup_for_C0 ) {
				// El contador C0 solo puede estar en D1
				cframe.counters[0]++;
				wakeup_for_C0 = false;
			}

			if ( wakeup_for_C1 ) {
				// El contador C1 solo puede estar en D2
				cframe.counters[1]++;
				wakeup_for_C1 = false;
			}

			if ( systemVars.debug == DEBUG_COUNTER) {
				xprintf_P( PSTR("COUNTERS: C0=%d,C1=%d\r\n\0"),(uint16_t) cframe.counters[0], (uint16_t) cframe.counters[1]);
			}
			// Espero 10ms de debounced
			//vTaskDelay( ( TickType_t)( 50 / portTICK_RATE_MS ) );
			vTaskDelay( ( TickType_t)( systemVars.counter_debounce_time / portTICK_RATE_MS ) );

			IO_clr_CLRD();		// Borro el latch llevandolo a 0.
			IO_set_CLRD();		// Lo dejo en reposo en 1

		} else   {
			// Expiro el timeout de la tarea. Por ahora no hago nada.
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_tkCounter_init(void)
{
	// Configuracion inicial de la tarea

uint8_t counter;

	// Configuracion de las interrupciones que genera el contador
	// PA2, PB2.
	// Los pines ya estan configurados como entradas.
	//
	PORTA.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;	// Sensa rising edge
	PORTA.INT0MASK = PIN2_bm;
	PORTA.INTCTRL = PORT_INT0LVL0_bm;

	PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;	// Sensa rising edge
	PORTB.INT0MASK = PIN2_bm;
	PORTB.INTCTRL = PORT_INT0LVL0_bm;

	wakeup_for_C0 = false;
	wakeup_for_C1 = false;

	for ( counter = 0; counter < NRO_DIGITAL_CHANNELS; counter++) {
		cframe.counters[ counter ] = 0;
	}

	IO_clr_CLRD();	// Borro el latch llevandolo a 0.
	IO_set_CLRD();	// Lo dejo en reposo en 1

}
//------------------------------------------------------------------------------------
ISR(PORTA_INT0_vect)
{
	// Esta ISR se activa cuando el contador D2 (PA2) genera un flaco se subida.
	// Solo avisa a la tarea principal ( que esta dormida ) que se levante y cuente
	// el pulso y haga el debounced.
	// Dado que los ISR de los 2 contadores son los que despiertan a la tarea, debo
	// indicarle de donde proviene el llamado
	wakeup_for_C0 = true;
	vTaskNotifyGiveFromISR( xHandle_tkCounter , &xHigherPriorityTaskWokenDigital );
	PORTA.INTFLAGS = PORT_INT0IF_bm;
}
//------------------------------------------------------------------------------------
ISR(PORTB_INT0_vect)
{
	// Esta ISR se activa cuando el contador D1 (PB2) genera un flaco se subida.
	// Solo avisa a la tarea principal ( que esta dormida ) que se levante y cuente
	// el pulso y haga el debounced.
	// Dado que los ISR de los 2 contadores son los que despiertan a la tarea, debo
	// indicarle de donde proviene el llamado
	wakeup_for_C1 = true;
	vTaskNotifyGiveFromISR( xHandle_tkCounter , &xHigherPriorityTaskWokenDigital );
	PORTB.INTFLAGS = PORT_INT0IF_bm;
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_counters_read_frame( st_counter_inputs_t cntframe[], bool reset_counters )
{

	// Esta funcion la invoca tkData al completar un frame para agregar los datos
	// digitales.
	// Leo los niveles de las entradas digitales y copio a dframe.
	// Respecto de los contadores, no leemos pulsos sino magnitudes por eso antes lo
	// convertimos a la magnitud correspondiente.

uint8_t i;

	// Convierto los contadores a las magnitudes (todos, por ahora no importa cuales son contadores )
	// Siempre multiplico por magPP. Si quiero que sea en mt3/h, en el server debo hacerlo (  * 3600 / systemVars.timerPoll )
	for (i = 0; i < NRO_COUNTER_CHANNELS; i++) {
		cframe.counters[i] = cframe.counters[i] * systemVars.c_ch_magpp[i];
	}

	// Copio el resultado
	memcpy(cntframe, &cframe, sizeof(st_counter_inputs_t) );

	// Borro los contadores para iniciar un nuevo ciclo.
	if ( reset_counters ) {
		for (i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
			cframe.counters[i] = 0.0;
		}
	}

}
//------------------------------------------------------------------------------------
void pub_counters_load_defaults(void)
{

	// Realiza la configuracion por defecto de los canales digitales.
	snprintf_P( systemVars.c_ch_name[0], PARAMNAME_LENGTH, PSTR("C0\0") );
	snprintf_P( systemVars.c_ch_name[1], PARAMNAME_LENGTH, PSTR("C1\0") );

	// Magnitud por pulso
	systemVars.c_ch_magpp[0] = 0.1;
	systemVars.c_ch_magpp[1] = 0.1;

	// Debounce Time
	systemVars.counter_debounce_time = 50;

}
//------------------------------------------------------------------------------------
bool pub_counters_config_channel( uint8_t channel,char *s_param0, char *s_param1 )
{

	// {0..1} dname magPP

bool retS = false;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	if ( ( channel >=  0) && ( channel < NRO_COUNTER_CHANNELS ) ) {

		// NOMBRE
		pub_control_string(s_param0);
		snprintf_P( systemVars.c_ch_name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_param0 );

		// MAGPP
		if ( s_param1 != NULL ) { systemVars.c_ch_magpp[channel] = atof(s_param1); }

		retS = true;
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);

}
//------------------------------------------------------------------------------------
void pub_counter_config_cdtime( char *s_sensortime )
{
	// Configura el tiempo de debounce del conteo de pulsos

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	systemVars.counter_debounce_time = atoi(s_sensortime);

	if ( systemVars.counter_debounce_time < 1 )
		systemVars.pwr_settle_time = 50;

	xSemaphoreGive( sem_SYSVars );
	return;
}
//------------------------------------------------------------------------------------


