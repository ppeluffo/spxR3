/*
 * main.c
 *
 *  Created on: 18 de oct. de 2016
 *      Author: pablo
 *
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e -U flash:w:spx.hex
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e
 *
 *  REFERENCIA: /usr/lib/avr/include/avr/iox256a3b.h
 *
 *  El watchdog debe estar siempre prendido por fuses.
 *  FF AA FD __ F5 D4
 *  FUSE 0: JTAGUID = 0xFF ( default )
 *  FUSE 1: WDGWINDOGTO | WDGTO = 0x0A (
 *  PROGRAMACION FUSES:
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Uflash:w:spx.hex:a -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *
 *  Para ver el uso de memoria usamos
 *  avr-nm -n spxR1.elf | more
 *
 *
 *------------------------------------------------------------------------------------------
 * - 2018-10-26: R1.0.0
 * Partimos de la version SPX_R1 V1.0.13 y la adaptamos a las modificaciones del hardware de
 * la version R3
 * - El bluetooth y USB no estan mas en la placa sino que van cableados a un conector externo
 * - Lo mismo los switches de TERM/BOOT/BAUD
 * Esto implica reprogramar los drivers del UART y las librerias de FRTOS-IO y printf.
 * Por ahora elimino el XBEE
 * Elimino el PROTOCOLO UTE.
 * En el modo SP5K solo trasmito los canales analogicos y contadores.
 *
 * Cambios de HARDWARE:
 * Las entradas digitales son fijas: 2 miden solo niveles y 2 son solo contadores por lo que
 * se elimina el parametro type del spx.h
 *
 * Leo la causa del reset y lo muestro en status y lo envio en el primer INIT.
 * Se borra al recibir el ACK del init del server
 *
 * WATCHDOGS
 * TERMINAL ( PRESENCE / BAUDRATE )
 *
 * BOTON: Con la configuracion actual siempre esta prendido !!!
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *------------------------------------------------------------------------------------------
 * -2018-10-13: R1.0.13
 * - En modo SP5K, cuando mide distancia, al trasmitir el frame no separa el campo DIST del
 * ultimo campo de canal digital, y luego de DIST pone 2 comas.
 * Se corrige en la funcion gprs_data::pv_tx_dataRecord_modo_SP5K().

 *------------------------------------------------------------------------------------------
 * -2018-10-06: R1.0.12
 * - En pv_process_init_response la salida por default es INIT_OK lo que hace que si no
 *   puede procesar la respuesta igual salga OK y pase a DATA.
 *   Ahora lo cambio el default a INIT_ERROR de modo que explicitamente la funcin deba cambiarlo
 *   a INIT_OK
 *------------------------------------------------------------------------------------------
 * -2018-09-27: R1.0.11
 * - Las funciones publicas del data, al invocarlas desde CMD me destrolan la secuencia del
 *   watchdog por lo tanto debo pasar un parametro para indicar si tocan o no el watchdog.
 * - Hago arreglos en XBEE tanto en formato de log como en configuracion de canales remotos.
 *------------------------------------------------------------------------------------------
 * -2018-09-26: R1.0.10
 * - Agrego un comando para leer los fuses y ver si estan bien configurados o no. Se usa porque
 * el fuse del watchdog si no esta bien puede causar problemas.
 * - Mejoro el watchdog de DATA para detectar fallas.
 * - Las funciones publicas del data, al invocarlas desde CMD me destrolan la secuencia del
 *   watchdog por lo tanto debo pasar un parametro para indicar si tocan o no el watchdog.
 *------------------------------------------------------------------------------------------
 * -2018-09-06: R1.0.8
 * - Agrego la variable uint8_t pwr_settle_time para configurar el tiempo de espera entre
 *   que prendo la fuente y que poleo. Se usa en los caudalimetros.
 * - Incorporo las funciones de rangeMeter. Paso un parametro que es
 *
 * *------------------------------------------------------------------------------------------
 * -2018-09-21: R1.0.9
 * Correcciones al modo OUTPUTS:
 * Separo la configuracion del seteo de las salidas / consignas.
 * Configuracion: Se hace desde cmd como desde gprs y ambos modifican los parametros para
 * invocar a l funcion estandar interna pub_outputs_config.
 *
 *------------------------------------------------------------------------------------------
 * -2018-09-05: R1.0.7
 * - Bug: Los caudales no se calculan con el timerpoll sino que solo multiplica los pulsos
 *   por magpp. Corrijo para que se expresen en mt/3.
 *   Si bien esto seria lo mismo que tenemos en las versiones de sp5K, el tema es que el
 *   firmware nos queda atado al calculo de caudal y no generico.
 *   Para dejarlo generico, multiplicamos por magPP solo, y para pasarlo a mt3/h usamos un
 *   factor adecuado pero siempre en el servidor.
 *------------------------------------------------------------------------------------------
 * -2018-08-31: R1.0.6
 * - Ajusto las salidas para que sea coherente con el esquema de los SP5K.
 * - La configuracion de las salidas a modo normal las deja por defecto en 0 y no las cambia.
 * - Para cambiarlas debo usar el comando write.
 * - Los canales se configuran como remotos solo si xbee esta en slave.
 *------------------------------------------------------------------------------------------
 * -2018-08-27: R1.0.5
 * - Arreglo bugs en configuracion local/online de canales digitales.
 * - Agrego un mensaje de error al usar parametros incorrectos en TDIAL.
 * - Arreglo bugs en xbee ( master / slave )
 *------------------------------------------------------------------------------------------
 *- 2018-08-19:
 *- Modifico funciones de l_printf para usar el BT en paralelo con USB.
 *- Modifico el driver UART para configurar el BT a 9600
 *------------------------------------------------------------------------------------------
 *- 2018-08-15:
 *- Creo un parametro el systemVars 'modo' que permite trabajar en modo SPX o SP5K.
 *- Envio en los init el SIMID. Para que se procese la version debe ser >5.3.0 por lo que
 *- en modo SP5K, la version pasa a ser 6.0.0 la que se envia.
 *------------------------------------------------------------------------------------------
 * 2018-07-26:
 * - Enviar UID en INIT y contemplar la reconfiguracion del dlgid.
 * - Ver cuando trasmito que el modem no este reseteado !!!
 *------------------------------------------------------------------------------------------
 * 2018-07-24:
 * - Cuando configuro las salidas solo lo hago en el systemVars. No las aplico sino que dejo
 * que la propia tarea lo haga luego.
 * Ver como mostrar la consigna aplicada !!!
 * - Incorporo la funcion de dailyReset.
 *------------------------------------------------------------------------------------------
 * 2018-07-20:
 * - El problema de perder datos se daria por el uso de las secciones criticas el leer datos
 * de los ringbuffers. Agrego un elemento a estos que indique si estan llegando datos.
 * La ISR de RX c/byte que llega lo prende.
 * Cuando voy a leer el RB veo si esta prendida. En este caso la apago y espero 10ms para volver
 * a leerla. Si esta apagada, indica que no hay acrtividad y leo. Si se prendio, salgo simulando
 * que aun no llego nada.
 * Los resultados son positivos ya que el frame de init ( 390 bytes ) se recibe por completo.
 *
 *------------------------------------------------------------------------------------------
 * 2018-07-19:
 * - La rutina de recepcion a veces pierde un caracter. Hay que perfeccionarla.
 * - Por la razon anterior, c/comando conviene chequearlo 3 veces antes de cancelar.
 * - Si no se conecta, reintentar a los 10 minutos una vez mas.
 *------------------------------------------------------------------------------------------
 * 2018-07-16:
 * Problema al imprimir un buffer (RX) mas grande que el de xprintf !!!
 * Reviso port.c para que trabaje con memoria extendida
 * Log de errores de I2C.
 * Incorporo la libreria l_ouptputs y revisar tkOutputs ( config modo )
 * Disminuyo el tiempo en el intercambio de consignas.
 * Revisar funciones de Cmd.
 *------------------------------------------------------------------------------------------
 * 2018-07-15:
 * Arreglar el tema de la lectura de los INA.
 * El tiempo por defecto para tomar los semaforos pasa a 5 ticks.
 * Modem: Analisis de rxdata/tamanio de buffers, etc, netopen fail
 *------------------------------------------------------------------------------------------
 * 2018-07-14:
 * Agrego buffers y estructura para BT y XBEE.
 * Modifico la libreria l_printf y elimino l_uarts.
 * Pongo un semaforo en l_file para acceder a la FAT
 * Paso los semaforos a estructuras estaticas.
 * Agrego la libreria l_ain
 * Modifico el menu para incorporar el concepto de usuario tecnico.
 * Los mensajes de init de las tareas los paso a luego de inicializarlas.
 * Reviso libreria NVM para poder leer el id
 * Agrego que se pueda ver el tiempo que falta para el siguiente poleo
 *------------------------------------------------------------------------------------------
 * 2018-07-13:
 * Implemento un sistema de impresion en consola xPrintf.
 * Eliminamos todos los buffers locales.
 * Elimino FRTOS-stdio
 * Revisamos las librerias l_i2c/l_eeprom/l_ina3221/l_rtc79410 y las funciones asociadas en tkCmd.
 *------------------------------------------------------------------------------------------
 * 2018-07-12:
 * - Previo elimino todo lo que tiene que ver con SPI que aun no esta implementado.
 * - En test probe el nuevo modelo de drivers y frtos en capas con servicios verticales.
 *   Hago la migracion.
 * - Migro el FRTOS a version 10.
 *------------------------------------------------------------------------------------------
 * 2018-06-27:
 * - Trasmito en los init la potencia del modem en DBM y no CSQ
 * - No trasmito la configuracion de los canales que estan apagados
 *
 * - Las lineas del CGI terminan en \r\n y no en \n. ( Apache 2.4)
 * 	https://en.wikipedia.org/wiki/HTTP_message_body
 * 	http://forum.arduino.cc/index.php?topic=200753.0
 * 	https://www3.ntu.edu.sg/home/ehchua/programming/webprogramming/HTTP_Basics.html
 * 	https://stackoverflow.com/questions/5757290/http-header-line-break-
 * ['0x53',
 '0x4b',
 '0x4f',
 '0x4b',
 '0x41',
 '0x7c',
 '0x46',
 '0x4c',
 '0x41',
 '0x56',
 '0x49',
 '0x41',
 '0x7c',
 '0x30',
 '0x39',
 '0x37',
 '0x34',
 '0x39',
 '0x36',
 '0x35',
 '0x33',
 '0x39']

 *
 *------------------------------------------------------------------------------------------
 * SCHEDULE:
 * *********
 *  Ampliar el buffer de GPRS_TX
 *
 * HACER:
 * Leer Memoria BD
 *
 * NOTAS DE DISENO:
 * ****************
 *  1- En FRTOS_Write_UART cambio el taskYIELD por taskDelay porque sino se cuelga.
 *     Este delay hacia que los mensajes del cmdmode fuesen lentos y entonces cambio en cmdline.c
 *     la forma de mostrarlos usando directamente FRTOS-IO.
 *
 * PENDIENTE:
 * **********
 * Hacer andar el watchdog
 * Cambiar la velocidad y reconffigurar el BT
 * Configuro el RTC.
 * Rutinas de calendario.
 *
 *
 * Features V0.0.1
 * - Si la terminal no esta conectada, CMD_write no escribe nada.
 * - El FRTOS siempre opera en modo tickless.
 *
 *  XBEE:
 *  Cuando el modo es slave, no uso la tarea de GPRS ( siempre duerme ) y transmito
 *  el frame al remoto.
 *  En esta version, los canales de slave se mapean en los del remoto de modo que el
 *  servidor no se entera de donde vienen.
 
 */


#include "spxR3.h"

//----------------------------------------------------------------------------------------
// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
// http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
//
// Function Pototype
//uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
//void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
/*
void wdt_init(void)
{
    // Como los fusibles estan para que el WDG siempre este prendido, lo reconfiguro a 8s lo
    // antes posible
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );

    return;
}
*/
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
int main( void )
{
	// Leo la causa del reset para trasmitirla en el init.
	wdg_resetCause = RST.STATUS;
	RST.STATUS = wdg_resetCause;
	//RST_PORF_bm | RST_EXTRF_bm | RST_BORF_bm | RST_WDRF_bm | RST_PDIRF_bm | RST_SRF_bm | RST_SDRF_bm;

	// Clock principal del sistema
	u_configure_systemMainClock();
	u_configure_RTC32();

	// Configuramos y habilitamos el watchdog a 8s.
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );
	if ( WDT_IsWindowModeEnabled() )
		WDT_DisableWindowMode();

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	initMCU();

	frtos_open(fdGPRS, 115200);
	frtos_open(fdTERM, 115200 );
	frtos_open(fdI2C, 100 );

	// Creo los semaforos
	sem_SYSVars = xSemaphoreCreateMutexStatic( &SYSVARS_xMutexBuffer );
	xprintf_init();
	FAT_init();

	startTask = false;

	xTaskCreate(tkCtl, "CTL", tkCtl_STACK_SIZE, NULL, tkCtl_TASK_PRIORITY,  &xHandle_tkCtl );
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);
	xTaskCreate(tkData, "DATA", tkData_STACK_SIZE, NULL, tkData_TASK_PRIORITY,  &xHandle_tkData);
	xTaskCreate(tkCounter, "COUNT", tkCounter_STACK_SIZE, NULL, tkCounter_TASK_PRIORITY,  &xHandle_tkCounter);
//	xTaskCreate(tkGprsRx, "RX", tkGprs_rx_STACK_SIZE, NULL, tkGprs_rx_TASK_PRIORITY,  &xHandle_tkGprsRx );
//	xTaskCreate(tkGprsTx, "TX", tkGprs_tx_STACK_SIZE, NULL, tkGprs_tx_TASK_PRIORITY,  &xHandle_tkGprsTx );
//	xTaskCreate(tkOutputs, "OUT", tkOutputs_STACK_SIZE, NULL, tkOutputs_TASK_PRIORITY,  &xHandle_tkOutputs );

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);

}
//-----------------------------------------------------------
void vApplicationIdleHook( void )
{
	// Como trabajo en modo tickless no entro mas en modo sleep aqui.
//	if ( sleepFlag == true ) {
//		sleep_mode();
//	}
}

//-----------------------------------------------------------
/* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */
//------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	// Es invocada si en algun context switch se detecta un stack corrompido !!
	// Cuando el sistema este estable la removemos.
	// En FreeRTOSConfig.h debemos habilitar
	// #define configCHECK_FOR_STACK_OVERFLOW          2

	xprintf_P( PSTR("PANIC:%s !!\r\n\0"),pcTaskName);

}
//------------------------------------------------------------------------------------

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
//------------------------------------------------------------------------------------

