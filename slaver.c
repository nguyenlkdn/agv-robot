//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"

#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "rfid.h"
#include "rfid.h"
#include "zigbee.h"
#include "t6963c.h"
#include "batterysensor.h"
#include "config.h"
#include "modbus-rtu.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator throught the UART.
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
char ZIGBEE_ID;
/*
 * [WARNING] System defination
 *
 */
uint32_t g_ui32CPUUsage;
uint32_t g_ui32Seconds;
uint32_t g_ui32SysClock;
uint8_t g_UART3RX1[UARTRX_BUFFER_SIZE];
uint8_t g_UART3RX2[UARTRX_BUFFER_SIZE];
uint8_t g_UART2RX1[UART2RX_BUFFER_SIZE];
uint8_t g_UART2RX2[UART2RX_BUFFER_SIZE];
//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
uint8_t sensor1[8];
uint8_t senso = 0;
int8_t xa = 0;
int32_t phai = 0;
int32_t trai = 0;
int den = 0;
int chieu = 1;
int loi = 0;
int loi1 = 1;
int loi2 = 0;
int loi10 = 1;
int dung = 1;
int di = 0;
int hienthi = 1;
int nan_ha = 0;
int tram0 = 0;
int tram1 = 1;
int boqua = 1;
int thongbao = 0;
int32_t time = 0;
int32_t speed = 10;
uint8_t ROBOT_STATE = 0;
uint32_t tocdo = 900; // toc do robot max 600
uint32_t biengiamtoc = 3; // gia tri giam toc
uint32_t bientantoc = 5000;  // gia tri tan toc
uint32_t tocdogiam = 300;   // tang giam thoi gian cham dan
uint8_t tocdotan = 10;    // tang giam thoi gian nhah dan
int i;
int ht_tram1 = 1;
int ht_tram0 = 1;
int binh = 1;
int xuong = 1;
//                0   1   2   3   4   5   6    7
//int32_t cap[8] = { 4500, 100, 200, 250, 390, 520, 690, 700 };
//int32_t cap[8] = { 5000, 200, 500, 1000, 1800, 2900, 3300, 4000 };
int32_t cap[8] = { 6000, 400, 900, 1500, 2300, 3200, 4300, 5500 };
//int32_t cap[8] = { 4500, 200, 500, 900, 1500, 2000, 2600, 3400 };
void runsenso2(void);
void stop1(void);
void dithang(void);

uint16_t data[10];

int8_t zigbeesentpackage[BUFFER_SIZE];
uint32_t rfid_location;

/*
 *
 */
// REGISTER 0 => Respond (Report cho Master)
// REGISTER 1 => Requested (Server yeu cau di den tram)
/*
 *
 */
uint32_t adcvalue[12];
//////////////reset  khi treo////////////////////////
//ROM_WatchdogEnable(WATCHDOG0_BASE);
/////////////////////////////////////////////
void main(void)

{
	init();
	ROM_IntMasterDisable();
	if (STATE == 0) {
		MotorInit();
		GPDSensorInit();
		LineSensorInit();
		rfidInit();
		zigbeeInit(1, UART3_BASE);
		ADCInit();
		GLCD_Initalize();
		STATE = 1;
	}
	ROM_IntMasterEnable();

	//
	// Enable processor interrupts.
	//
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	ROM_TimerEnable(TIMER3_BASE, TIMER_A);

	SysCtlDelay(g_ui32SysClock / 1000);

	GLCDPrintfNormal(0, 0, "  -----  ROBOTIC AGV -----");
	GLCDPrintfNormal(0, 3, "Err : 0              ");
//      GLCDPrintfNormal(0, 2, "Batery: %d (volt)  ", adcvalue / 124);

	// Loop forever while the timers run.

//     ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	//   MotorController(4000, 4000);
	UARTprintf("robot 0k \n");
	dung = 1;
	while (1) {

		//  UARTprintf("Batery %d \n",adcvalue[0]);
		//  SysCtlDelay(g_ui32SysClock / 10000);
		//////////////////////////////////////////////////////
		if (binh == 1) {
			GLCDPrintfNormal(5, 1, "%d", ROBOTRX_Buffer[0]);
			ADCGet(adcvalue);
			if (adcvalue[0] / 124 < 20) {
				GLCDPrintfNormal(0, 2, "Batery low : %2d (volt)  ",
						adcvalue[0] / 124);

			} else {
				GLCDPrintfNormal(0, 2, "Batery high: %2d (volt)  ",
						adcvalue[0] / 124);        /// 124
			}
			binh = 0;
		}
		///////////////////////////////////////////////////
		if (loi2 == 0) {
			if (loi1 != loi) {
				//GLCDPrintfNormal(0, 3, "Err : %d  ", loi);
				switch (loi) {
				case 1:
					GLCDPrintfNormal(0, 3, "Err : forewarning 1");
					break;

				case 2:
					GLCDPrintfNormal(0, 3, "Err : forewarning 2");
					break;

				case 3:
					GLCDPrintfNormal(0, 3, "Err : forewarning 3");
					break;

				case 4:
					GLCDPrintfNormal(0, 3, "Err : forewarning 4");
					break;

				case 5:
					GLCDPrintfNormal(0, 3, "Err : forewarning 5");
					break;

				case 9:
					GLCDPrintfNormal(0, 3, "Err : accident      ");
					break;

				case 0:
					GLCDPrintfNormal(0, 3, "Err : 0            ");
					break;

				}
				loi1 = loi;
				loi10 = 1;
			}
		} else {
			if (loi10 == 1) {
				GLCDPrintfNormal(0, 3, "Err : no line        ");
				loi10 = 0;
			}
		}
		///////////////////////////////////////////////////
		if (hienthi == 1) {
			hienthi = 0;

			if (dung == 1) {
				GLCDPrintfNormal(0, 4, "Robot : ready   ");
			} else {
				GLCDPrintfNormal(0, 4, "Robot :stop   ");
			}

		}
		if (ht_tram1 == 1) {
			GLCDPrintfNormal(0, 5, "Requesting Station: %d ", tram1);
			ht_tram1 = 0;
		}
		if (ht_tram0 == 1) {
			GLCDPrintfNormal(0, 6, "Current Station: %d  ", tram0);

			ht_tram0 = 0;
		}
		//////////////////// nan ha khay //////////////////////////

		if (tram1 == tram0) {
			if (nan_ha == 1 && tram0 == 5) {

				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
				xuong = 0;

			} else {
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);

				xuong = 1;
			}
		} else {
			nan_ha = 0;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
		}
////////////////////////////////////////////////////////////

	}
}

//*****************************************************************************
//
// System Interupt Handler
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void PORTJIntHandler(void) {
	uint32_t PortFmask = GPIOIntStatus(GPIO_PORTJ_BASE,
	GPIO_PIN_0 | GPIO_PIN_1);
	if (di == 1 && loi == 0) {
		if (PortFmask & GPIO_PIN_0) {
			/////////////////////////////////////////////////////////////////////////////////////////////
			UARTprintf("\n test loi");
//        loi = 1;
//         time = 0;
			/////////////////////////////////////////////////////////////////////////////////////////////
			SysCtlDelay(SysCtlClockGet() / 200);
			GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);
		}
		if (PortFmask & GPIO_PIN_1) {
			/////////////////////////////////////////////////////////////////////////////////////////////
			// UARTprintf("\n ha xuong !");
			/////////////////////////////////////////////////////////////////////////////////////////////
			SysCtlDelay(SysCtlClockGet() / 200);
			GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_1);
		}
	} else {
		GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);
	}

}
//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void Timer0IntHandler(void) {
////////////////////////////////////////////

	if (tram1 == tram0) {
		stop1();
	} else {
		if (boqua == 1) {
			if (GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6) == 64) {
				loi = 1;
				time = 0;
			}
			////////////
			if (

			GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7) == 128) {
				loi = 2;
				time = 0;
			}
			if (

			GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_0) == 1) {
				loi = 3;
				time = 0;
			}
			if (

			GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_3) == 8) {
				loi = 4;
				time = 0;
			}
			if (

			GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) == 4) {
				loi = 5;
				time = 0;
			}
		}
		if (GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1) == 2) {
			loi = 9;
			dung = 0;
			hienthi = 1;
			time = 0;
		}
		dithang();
	}
	//////////////////////////////////////////////

	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void Timer1IntHandler(void) {
	//ADCGet(&adcvalue);
	//
	// Clear the timer interrupt.  ///////7170BC2 // F3E0BA2

	/////0DA9BB2 // EB88975

	//
	//UARTprintf("Timer1IntHandler \n");
	if (RFID_ID[0] == ':') {
		binh = 1;
		//   UARTprintf("RFID ID%s\n", RFID_ID);
		if (strcmp(RFID_ID, ":1507A88") == 0) {
			UARTprintf("da toi tram 1\n");
			ROBOTTX_Buffer[0] = 1;
			tram0 = 1;
			ht_tram0 = 1;

		} else if (strcmp(RFID_ID, ":C4D1060") == 0) {
			UARTprintf("da toi tram 2\n");
			ROBOTTX_Buffer[0] = 2;
			tram0 = 2;
			ht_tram0 = 1;

		} else if (strcmp(RFID_ID, ":C41EBE0") == 0) {
			UARTprintf("da toi tram 3\n");
			tram0 = 3;
			ht_tram0 = 1;
			ROBOTTX_Buffer[0] = 3;

		} else if (strcmp(RFID_ID, ":C42A510") == 0) {
			UARTprintf("da toi tram 4\n");
			ROBOTTX_Buffer[0] = 4;
			tram0 = 4;
			ht_tram0 = 1;
		} else if (strcmp(RFID_ID, ":C476220") == 0) {
			ROBOTTX_Buffer[0] = 5;
			ht_tram0 = 1;
			UARTprintf("da toi tram 5\n");
			tram0 = 5;
		} else if (strcmp(RFID_ID, ":7170BC2") == 0 || strcmp(RFID_ID, ":F3E0BA2") == 0) {
			bientantoc = 4000;
			boqua = 0;

		} else if ( strcmp(RFID_ID, ":0DA9BB2") == 0 ||strcmp(RFID_ID, ":EB88975") == 0 ) {
			bientantoc = 10000;
			boqua = 1;

		}
//////////////////////////////////////////////////////////////////////
		else if (strcmp(RFID_ID, ":D395925") == 0
				//|| strcmp(RFID_ID, ":EB88975") == 0
				|| strcmp(RFID_ID, ":38C3BB2") == 0
				|| strcmp(RFID_ID, ":28AAB02") == 0
				|| strcmp(RFID_ID, ":FB1D935") == 0

				|| strcmp(RFID_ID, ":B40D935") == 0
				|| strcmp(RFID_ID, ":50E7925") == 0
				|| strcmp(RFID_ID, ":5CC2925") == 0
				|| strcmp(RFID_ID, ":5070925") == 0)    //1
						{
			UARTprintf("tang toc \n");
			bientantoc = 10000;
		}
		/////  ////////////               giam toc /////////////////////////
		else if (strcmp(RFID_ID, ":5D99925") == 0
				|| strcmp(RFID_ID, ":F636925") == 0
				|| strcmp(RFID_ID, ":65EB925") == 0
				|| strcmp(RFID_ID, ":62C4BA2") == 0
				|| strcmp(RFID_ID, ":EB84BB2") == 0
				|| strcmp(RFID_ID, ":EFAABB2") == 0
			//	|| strcmp(RFID_ID, ":F3E0BA2") == 0
				|| strcmp(RFID_ID, ":1BC7BB2") == 0

				|| strcmp(RFID_ID, ":4DEBBB2") == 0)    //1
						{
			UARTprintf("giam toc \n");
			bientantoc = 4000;
		}

//////////////////////////////////////////////////////////////////////
		else {
			UARTprintf("ERROR: Unknown RFID %s ????\n", RFID_ID);
		}
		RFID_ID[0] = 0;

	}
	rfid_location = tram0;
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void Timer3IntHandler(void) {
	//
	// Clear the timer interrupt.
	//
	ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	if (1) {
		if (ROBOTRX_Buffer[0] > 0) {
			ht_tram0 = 1;
			// UARTprintf("Received location: %d\n", ROBOTRX_Buffer[5]);
			if (ROBOTRX_Buffer[0] == 6 && tram1 != 4) {
				tram1 = 4;
                tram0 = 0;
                dung = 1;
				ht_tram1 = 1;
				UARTprintf("di tram 4\n");
			}

			if (ROBOTRX_Buffer[0] == 1 && tram1 != 1) {
				tram1 = 1;
				tram0 = 0;
				dung = 1;

				ht_tram1 = 1;
				UARTprintf("di tram 1\n");
			}
//             if (ROBOTRX_Buffer[0] == 2 && tram1 != 2)
//             {
//                 tram1 = 2;
//                 tram0 = 0;
//
//                      loi = 0;
//                 ht_tram1 = 1;
//                 UARTprintf("di tram 2\n");
//             }
			if (ROBOTRX_Buffer[0] == 3 && tram1 != 3) {
				tram1 = 3;
				tram0 = 0;
				dung = 1;
				loi = 0;
				ht_tram1 = 1;
				UARTprintf("di tram 3\n");
			}
			if (ROBOTRX_Buffer[0] == 4 && tram1 != 4) {
				tram1 = 4;
				tram0 = 0;
				ht_tram1 = 1;
				UARTprintf("di tram 4\n");
			}
			if (ROBOTRX_Buffer[0] == 5 && tram1 != 5) {
				tram1 = 5;
				tram0 = 0;
				dung = 1;
				loi = 0;
				ht_tram1 = 1;
				UARTprintf("di tram 5\n");
			}
//             if (ROBOTRX_Buffer[0] == 6 && dung != 1)
//             {
//
//
//                 dung = 1;
//                 hienthi = 1;
//
//                 UARTprintf("run\n");
//             }
			if (ROBOTRX_Buffer[0] == 7 && tram1 != 1) {

				tram1 = 1;
				tram0 = 0;
				dung = 1;
				ht_tram1 = 1;

				UARTprintf("di tram 1\n");

				UARTprintf("stop\n");
			}
			if (ROBOTRX_Buffer[0] == 8 && nan_ha == 0) {

				nan_ha = 1;

				UARTprintf("nan\n");
			}
			if (ROBOTRX_Buffer[0] == 9 && nan_ha == 1) {

				nan_ha = 0;

				UARTprintf("nan\n");
			}

//             if (ROBOTRX_Buffer[0] == )
//             {
//                 //      ROM_TimerDisable(TIMER0_BASE, TIMER_A);
//                 dung = 0;
//
//                 UARTprintf("stop\n");
//             }
		}
	} else {
		UARTprintf("[WARNING] Robot in processing!!!\n");
	}

}
//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************

void PORTBIntHandler(void) {
	uint32_t PortBmask = GPIOIntStatus(GPIO_PORTB_BASE,
	GPIO_PIN_2 | GPIO_PIN_3);
	SysCtlDelay(SysCtlClockGet() / 100);
	hienthi = 1;
	if (PortBmask & GPIO_PIN_2) {
		/////////////////////////////////////////////////////////////////////////////////////////////
		UARTprintf("\n run!");
		dung = 1;
		loi = 0;
		/////////////////////////////////////////////////////////////////////////////////////////////
		//  SysCtlDelay(SysCtlClockGet() / 100);
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2);
	}
	if (PortBmask & GPIO_PIN_3) {
		/////////////////////////////////////////////////////////////////////////////////////////////
		UARTprintf("\n stop !");
		dung = 0;

		/////////////////////////////////////////////////////////////////////////////////////////////
		// SysCtlDelay(SysCtlClockGet()/ 100);
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_3);
	}

}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void PORTCIntHandler(void) {
//    uint32_t PortCmask = GPIOIntStatus(
//    GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
//    if (di == 1 && loi == 0)
//    {
//    	SysCtlDelay(SysCtlClockGet() / 500);
//        if (PortCmask & GPIO_PIN_6)
//        {
//            /////////////////////////////////////////////////////////////////////////////////////////////
//
//            UARTprintf("\n vat can xa 3 !");
//            loi = 3;
//            time = 0;
//            UARTprintf("\n ma loi: %d", loi);
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            SysCtlDelay(SysCtlClockGet() / 1000);
//            GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_6);
//        }
//        if (PortCmask & GPIO_PIN_7)
//        {
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            UARTprintf("\n vat can xa 4 !");
//            loi = 4;
//            time = 0;
//            UARTprintf("\n ma loi: %d", loi);
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            SysCtlDelay(SysCtlClockGet() / 1000);
//            GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_7);
//        }
//    }
//    else
//    {
//        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_7);
//        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_6);
//
//    }
	///////////////////
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_7);
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_6);

}

void PORTHIntHandler(void) {
//    uint32_t PortHmask = GPIOIntStatus(
//            GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
//    if (di == 1 && loi == 0 )
//    {
//    	SysCtlDelay(SysCtlClockGet() / 500);
//        if (PortHmask & GPIO_PIN_0)
//        {
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            UARTprintf("\n vat can xa 5 !");
//            loi = 5;
//            time = 0;
//            UARTprintf("\n ma loi: %d", loi);
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            SysCtlDelay(SysCtlClockGet() / 10000);
//            GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_0);
//        }
//        if (PortHmask & GPIO_PIN_1)
//        {
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            UARTprintf("\n vat can xa 6!");
//            loi = 9;
//            dung = 0;
//            UARTprintf("\n ma loi: %d", loi);
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            SysCtlDelay(SysCtlClockGet() / 10000);
//            GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_1);
//        }
//        if (PortHmask & GPIO_PIN_2)
//        {
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            UARTprintf("\n vat can xa 2 !");
//            loi = 2;
//            time = 0;
//            UARTprintf("\n ma loi: %d", loi);
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            SysCtlDelay(SysCtlClockGet() / 10000);
//            GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
//        }
//        if (PortHmask & GPIO_PIN_3)
//        {
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            UARTprintf("\n vat can xa 1 !");
//            loi = 1;
//            time = 0;
//            UARTprintf("\n ma loi: %d", loi);
//            /////////////////////////////////////////////////////////////////////////////////////////////
//            SysCtlDelay(SysCtlClockGet() / 10000);
//            GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_3);
//        }
//    }
//    else
//    {
//        GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_3);
//        GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
//        GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_0);
//
//    }
//    if (PortHmask & GPIO_PIN_1)
//         {
//             /////////////////////////////////////////////////////////////////////////////////////////////
//             UARTprintf("\n vat can xa 6!");
//             loi = 9;
//             dung = 0;
//             UARTprintf("\n ma loi: %d", loi);
//             /////////////////////////////////////////////////////////////////////////////////////////////
//             SysCtlDelay(SysCtlClockGet() / 10000);
//             GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_1);
//         }
/////////////////////////////////////

	GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_3);
	GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
	GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_0);
	GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_1);

}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
//void
//Timer4IntHandler(void)
//{
//    //
//    // Clear the timer interrupt.
//    //
//    ROM_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
//}
//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void Timer5IntHandler(void) {

	//
	// Clear the timer interrupt.
	//
	ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
}

// Vector Rx/Tx UART0 from PC
void UART0IntHandler(void) {
	UARTIntClear(UART0_BASE, UART_INT_RX);
	while (UARTCharsAvail(UART0_BASE)) {
		switch (UARTCharGet(UART0_BASE)) {
		case 'a':

			UARTprintf("\n stop robot !");
			dung = 0;
			hienthi = 1;
			// loi = 1;
			break;
		case 'd':

			UARTprintf("\n run!");
			dung = 1;
			hienthi = 1;
			loi = 0;

			break;
		case 'w':

			UARTprintf("\n nan len");
			nan_ha = 1;

			break;
		case 's':

			UARTprintf("\n ha xuong");
			nan_ha = 0;

			break;
		case '1':
			ROBOTTX_Buffer[0] = 1;
			break;
		case '2':
			ROBOTTX_Buffer[0] = 2;
			break;
		case '3':
			ROBOTTX_Buffer[0] = 3;
			break;
		case '4':
			ROBOTTX_Buffer[0] = 4;
			break;
		default:
			UARTprintf("Unknow\n");
			break;
		}
	}
}

void stop1(void) {
	//  UARTprintf("\n giam: %d", biengiamtoc);
	di = 0;
	den = 0;
	if (biengiamtoc > 2000) {
		biengiamtoc = biengiamtoc - tocdogiam;
		tocdo = biengiamtoc;
	} else {
		biengiamtoc = 3;
	}

	MotorController(biengiamtoc, biengiamtoc);

}
void dithang(void) {

	if (dung == 1) {
		if (loi == 0) {

			runsenso2();

			den = 1;
			di = 1;

		} else {
			stop1();
			time = time + 1;
			//UARTprintf("\n time: %d", time);

			if (time >= 300) {
				loi = 0;
			}
//			} else {
//				if (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0
//						|| GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6) == 64
//						|| GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7) == 128
//						|| GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_0) == 1
//						|| GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1) == 2
//						|| GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) == 4
//						|| GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_3) == 8) {
//
////					UARTprintf("\n c6= %d",
////							GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6));
////					UARTprintf("\n c7= %d",
////							GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7));
////					UARTprintf("\n h0= %d",
////							GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_0));
////					UARTprintf("\n h1= %d",
////							GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1));
////					UARTprintf("\n h2= %d",
////							GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2));
////					UARTprintf("\n h3= %d",
////							GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_3));
//					time = 0;
//				}
//			}

		}
	} else {
		den = 0;
		trai = 3;
		phai = 3;
		MotorController(trai, phai);
		tocdo = 100;
	}

}
void runsenso2(void) {
	senso = GPIOPinRead(GPIO_PORTM_BASE, 0xff);

	unsigned short mask = 128;
	for (i = 0; i < 8; i++) {
		if (senso & mask) {
			//    UARTprintf("1");
			sensor1[i] = 1;
		} else {
			//   UARTprintf("0");
			sensor1[i] = 0;
		}
		mask >>= 1;
	}
	if (tocdo >= bientantoc) {

		// tocdo = 5500;
		tocdo = tocdo - 300;
		//tocdo = bientantoc;
	}
//	UARTprintf("\n");
	//	UARTprintf("%d %d %d %d %d %d %d %d",);
	//SysCtlDelay(SysCtlClockGet() / 1000);
	////////////////////////////
	//  UARTprintf("\n td: %d", tocdo);
//         tocdo = tocdo + tocdotan;
//         biengiamtoc = tocdo;
	/////////////////////////////////
	if (sensor1[3] == 1 && sensor1[4] == 1) {
		tocdo = tocdo + tocdotan;
		biengiamtoc = tocdo;

		phai = tocdo;
		trai = tocdo;

	} else {

		if (sensor1[2] == 1 && sensor1[3] == 1) {
			tocdo = tocdo + tocdotan;
			biengiamtoc = tocdo;
			phai = tocdo + cap[2];
			trai = tocdo - cap[2];
			loi2 = 0;
		} else {
			if (sensor1[1] == 1 && sensor1[2] == 1) {
				tocdo = tocdo + tocdotan;
				biengiamtoc = tocdo;
				phai = tocdo + cap[4];
				trai = tocdo - cap[4];

			} else {
				if (sensor1[0] == 1 && sensor1[1] == 1) {
					tocdo = tocdo + tocdotan;
					biengiamtoc = tocdo;
					phai = tocdo + cap[6];
					trai = tocdo - cap[6];

				} else {
					if (sensor1[3] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo + cap[1];
						trai = tocdo - cap[1];

					}
					if (sensor1[2] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo + cap[3];
						trai = tocdo - cap[3];

					}
					if (sensor1[1] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo + cap[5];
						trai = tocdo - cap[5];

					}
					if (sensor1[0] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo + cap[7];
						trai = tocdo - cap[7];
					}
				}
			}
		}

////////////////////////////////////////

		if (sensor1[4] == 1 && sensor1[5] == 1) {
			tocdo = tocdo + tocdotan;
			biengiamtoc = tocdo;
			phai = tocdo - cap[2];
			trai = tocdo + cap[2];
		} else {
			if (sensor1[5] == 1 && sensor1[6] == 1) {
				tocdo = tocdo + tocdotan;
				biengiamtoc = tocdo;
				phai = tocdo - cap[4];
				trai = tocdo + cap[4];

			} else {

				if (sensor1[6] == 1 && sensor1[7] == 1) {
					tocdo = tocdo + tocdotan;
					biengiamtoc = tocdo;
					phai = tocdo - cap[6];
					trai = tocdo + cap[6];

				}

				else {
					if (sensor1[7] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo - cap[7];
						trai = tocdo + cap[7];

					}
					if (sensor1[4] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo - cap[1];
						trai = tocdo + cap[1];

					}

					if (sensor1[6] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo - cap[5];
						trai = tocdo + cap[5];

					}
					if (sensor1[5] == 1) {
						tocdo = tocdo + tocdotan;
						biengiamtoc = tocdo;
						phai = tocdo - cap[3];
						trai = tocdo + cap[3];

					}
				}
			}

		}
	}

	if (sensor1[0] == 0 && sensor1[1] == 0 && sensor1[2] == 0 && sensor1[3] == 0
			&& sensor1[4] == 0 && sensor1[5] == 0 && sensor1[6] == 0
			&& sensor1[7] == 0)

			{
		stop1();
		loi2 = 10;
	} else {
		// UARTprintf( "\ntrai: %d phai: %d  \n", trai, phai);
		MotorController(phai, trai);
		//  UARTprintf("\n trai: %d  phai %d", phai , trai);
	}

}
// Vector Rx/Tx UART2 from RFID
void UART1IntHandler(void) {
	UARTIntClear(UART1_BASE, UART_INT_RX);
	int32_t incoming = 0;
	while (UARTCharsAvail(UART1_BASE)) {
		incoming = UARTCharGet(UART1_BASE);
		UARTprintf("INCOMING %3d: %x\n", incoming);

	}
}
