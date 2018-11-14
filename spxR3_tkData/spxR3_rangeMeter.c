/*
 * spx_tkPulses.c
 *
 *  Created on: 11 de abr. de 2018
 *      Author: pablo
 */


#include "spxR3.h"

typedef enum { RISING_EDGE = 0, FALLING_EDGE } t_edge_sensing;

#define MAX_RANGEMETER_STACK 32

static struct {
	uint8_t ptr;
	int32_t stack[MAX_RANGEMETER_STACK];
} s_rangeMeter_stack;


void pv_rangeMeter_start(void);
void pv_rangeMeter_stop(void);

void pv_flush_stack_rangeMeter(void);
void pv_push_stack_rangeMeter(uint16_t counter);
int16_t pv_rangeMeter_calcular_distancia(void);
void pv_rangeMeter_statistics(uint16_t *avg, double *var);

static uint8_t PULSE_TIMER_PRESCALER;
static uint8_t USxTICK;
static bool midiendo;

//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_rangeMeter_init(void)
{
	// Apagamos la medicion de pulsos.
	pv_rangeMeter_stop();

	// Calculo el prescaler del timer.
	switch(SYSMAINCLK) {
	case 32:
		// clocksys = 32MHz. El prescaler utilizado es 5 de modo que divido por 64 y c/tick
		// equivale a 2us.
		PULSE_TIMER_PRESCALER = TC_CLKSEL_DIV64_gc;
		USxTICK = 2;
		break;
	case 8:
		// 8Mhz: Divido por8 y c/tick es 1us
		PULSE_TIMER_PRESCALER = TC_CLKSEL_DIV8_gc;
		USxTICK = 1;
		break;
	case 2:
		// 2Mhz: Divido por2 y c/tick es 1us
		PULSE_TIMER_PRESCALER = TC_CLKSEL_DIV2_gc;
		USxTICK = 1;
		break;
	}

	// Configuro la INT0 del port C para interrumpir por flanco del PIN C0.
	PORTC.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc;	// Sens both edge
	PORTC.INT0MASK = PIN0_bm;
	PORTC.INTCTRL = 0x00; 	// Interrupcion deshabilitada.

}
//------------------------------------------------------------------------------------
void pub_rangeMeter_ping(int16_t *range)
{
	// Mido durante 5 segundos y luego promedio las medidas.

int16_t ping = 0;

	// Si la funcion no esta habilitada retorno inmediatamente con 0.
	if ( systemVars.rangeMeter_enabled == false ) {
		*range = ping;
		return;
	}

	if ( systemVars.debug == DEBUG_RANGEMETER ) {
		xprintf_P( PSTR("RANGE: Start\r\n\0"));
	}

	// Prendo el sensor
	pv_rangeMeter_start();

	// Inicializo
	midiendo = false;
	pv_flush_stack_rangeMeter();

	// Habilto la interrupcion
	PORTC.INTCTRL = PORT_INT0LVL0_bm;
	//
	// Espero
	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
	//
	// Desbilto la interrupcion
	PORTC.INTCTRL = 0x00;
	//
	// Apago el sensor
	pv_rangeMeter_stop();
	//
	if ( systemVars.debug == DEBUG_RANGEMETER ) {
		xprintf_P( PSTR("RANGE: Stop\r\n\0"));
	}
	// Calculo valores y muestro resultados
	ping = pv_rangeMeter_calcular_distancia();

	*range = ping;
	return;

}
//------------------------------------------------------------------------------------
void pub_rangeMeter_load_defaults(void)
{

	// Realiza la configuracion por defecto del medidor de ancho de pulsos.
	// Por defecto esta apagado.
	systemVars.rangeMeter_enabled = modoRANGEMETER_OFF;
	systemVars.rangeMeter_factor = 58;

}
//------------------------------------------------------------------------------------
bool pub_rangeMeter_config( uint8_t modo, uint16_t factor )
{
bool retS;

	switch(modo) {
	case modoRANGEMETER_OFF:
		systemVars.rangeMeter_enabled =  modoRANGEMETER_OFF;
		retS = true;
		break;
	case modoRANGEMETER_ON:
		systemVars.rangeMeter_enabled =  modoRANGEMETER_ON;
		retS = true;
		break;
	default:
		systemVars.rangeMeter_enabled =  modoRANGEMETER_OFF;
		retS = false;
		break;
	}

	systemVars.rangeMeter_factor = factor;

	return(retS);
}

//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
void pv_rangeMeter_start(void)
{
	// Activo el sensor habilitando la señal UPULSE_RUN lo que hace
	// que el sensor comienze a medir con una frecuencia de 6hz.
	// En el pin 2 del sensor debo poner un 1 ( aunque tiene un pullup). Como paso
	// por un inversor, el micro debe poner un 0.

	IO_clr_UPULSE_RUN();

}
//------------------------------------------------------------------------------------
void pv_rangeMeter_stop(void)
{

	// El pin de RUN es el pin4 del sensor MAX-XL alimentado por medio de un inversor
	// En reposo el pin del sensor debe estar en 0 por lo tanto el micro pone un 1
	// antes del inversor.

	IO_set_UPULSE_RUN();

}
//------------------------------------------------------------------------------------
void pv_flush_stack_rangeMeter(void)
{

	// Inicicalizo el stack de datos

uint8_t i;

	for (i=0; i < MAX_RANGEMETER_STACK; i++) {
		s_rangeMeter_stack.stack[i] = -1;
	}
	s_rangeMeter_stack.ptr = 0;
}
//------------------------------------------------------------------------------------
void pv_push_stack_rangeMeter(uint16_t counter)
{
	// Agrego una medida al stack para luego poder hacer las estadisticas

	if( s_rangeMeter_stack.ptr < MAX_RANGEMETER_STACK ) {
		s_rangeMeter_stack.stack[s_rangeMeter_stack.ptr++] = counter;
	}

}
//------------------------------------------------------------------------------------
int16_t pv_rangeMeter_calcular_distancia(void)
{


uint16_t avg;
double var;
float us;
uint16_t distancia;
int16_t ping;

	pv_rangeMeter_statistics(&avg, &var);
	us = USxTICK * avg;						// Convierto a us.
	distancia = (uint16_t)( us / systemVars.rangeMeter_factor );		// Calculo la distancia ( 58us - 1cms )
	if ( (distancia > 0) && (distancia < 600) ) {
		ping = distancia;
	} else {
		ping = -1;
	}

	if ( systemVars.debug == DEBUG_RANGEMETER ) {
		xprintf_P( PSTR("RANGE: avg=%d, var=%.03f, us=%.1f, distancia=%d \r\n\0"),avg, var, us, distancia);
	}

	return(ping);

}
//------------------------------------------------------------------------------------
void pv_rangeMeter_statistics(uint16_t *avg, double *var)
{
	// Calculo el promedio de los datos del stack si sin validos.

uint32_t sum;
uint8_t i, items;

	// Promedio.
	sum = 0;
	items = 0;
	for ( i = 0; i < MAX_RANGEMETER_STACK; i++ ) {
		if ( s_rangeMeter_stack.stack[i] > 0) {
			sum =  sum + s_rangeMeter_stack.stack[i];
			items += 1;
		}

		if ( systemVars.debug == DEBUG_RANGEMETER ) {
			xprintf_P( PSTR("RANGE: [%02d][%02d] %d %lu\r\n\0"), i,items,s_rangeMeter_stack.stack[i], sum );
		}
	}
	*avg = (uint16_t) (sum / items);

	// Desviacion estandard
	sum = 0;
	items = 0;
	for ( i = 0; i < MAX_RANGEMETER_STACK; i++ ) {
		if ( s_rangeMeter_stack.stack[i] > 0) {
			sum =  sum + square( s_rangeMeter_stack.stack[i] - *avg);
			items += 1;
		}
	}
	*var = sqrt(sum / items);

	// Desviacion estandard

}
//------------------------------------------------------------------------------------
// ISR
// El ANCHO DEL PULSO SE MIDE EN EL PIN UPULSE_WIDTH.
// Tenemos una interrupcion de flanco.
//
ISR( PORTC_INT0_vect )
{
	// Detectamos cual flanco disparo la interrupcion.
	// Como pasamos el pulso por un inversor los flancos quedan cambiados.
	// Lo que hacemos es tomar como referencias los pulsos medidos en el micro y
	// no los generados en el sensor.

	if ( IO_read_UPULSE_WIDTH() == 0 ) {
		// Flanco de bajada: Arranca el pulso. Arranco el timer.
		TCC1.CNT = 0;
		TCC1.CTRLA = PULSE_TIMER_PRESCALER;
		midiendo = true;
		//IO_set_GPRS_PWR();

	} else {
		// Flanco de subida: Termino el pulso. Paro el timer
		TCC1.CTRLA = TC_CLKSEL_OFF_gc;	// Apago el timer.
		if (midiendo) {
			midiendo = false;
			pv_push_stack_rangeMeter(TCC1.CNT);
		}

		//IO_clr_GPRS_PWR();
	}

	// Borro la interrupcion
	PORTC.INTFLAGS = PORT_INT0IF_bm;

}
//------------------------------------------------------------------------------------
