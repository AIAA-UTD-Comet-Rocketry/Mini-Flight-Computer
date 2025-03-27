/*
 * General flight logic that would apply to every flight computer firmware belongs
 * in the FlightState.c file.
 * This file is for hardware and application dependent code, such as redefining
 * the weak transition functions so they interface with sensor data streams.
 *
 * STATE_IDLE <---------
 *      |               ^
 * STATE_ARMED --> STATE_DISARM
 *      |
 * STATE_BURNING
 *      |
 * STATE_RISING
 *      |
 * STATE_APOGEE
 *      |
 * STATE_DROGUE_DESCENT
 *      |
 * STATE_MAIN_DESCENT
 *      |
 * STATE_LANDED
 */

#include "FlightState.h"

/* Extern data streams to include */
extern float gAltitude;
extern float gTotalAcc;
extern float gDegOffVert;
extern volatile uint32_t uwTick;

/* Private Variables */
float prevAlt = 0;
uint32_t transDelay = UINT32_MAX;

/* Redefined Callback Implementations, Called when new state is entered */
void enterIdle(void) { printf("New: Entering Idle state.\n"); }
void enterArmed(void) { printf("New: Entering Armed state.\n"); }
void enterDisarm(void) { printf("New: Entering Disarm state.\n"); }
void enterBurning(void) { printf("New: Entering Burning state.\n"); }
void enterRising(void) { printf("New: Entering Rising state.\n"); }
void enterApogee(void) { printf("New: Entering Apogee state.\n"); }
void enterDrogueDescent(void) { printf("New: Entering Drogue Descent state.\n"); }
void enterMainDescent(void) { printf("New: Entering Main Descent state.\n"); }
void enterLanded(void) { printf("New: Entering Landed state.\n"); }

/* Redefined Transition Functions, true moves to next */
bool idleTransition(void) { return true; }
bool armedTransition(void)
{
	if(gTotalAcc > 5.0)
	{
		if(uwTick > transDelay)
		{
			transDelay = UINT32_MAX;
			return true;
		}
		else if (transDelay == UINT32_MAX) //first high g reading, start timer
		{
			transDelay = uwTick + 100; //burn for 0.1 sec before trigger
		}
	}
	else
	{
		transDelay = UINT32_MAX; // one bad reading resets timer
	}

	return false;
}

bool disarmTransition(void) { return true; }
bool burningTransition(void) { return (gTotalAcc < 2.0); }
bool risingTransition(void)
{
	if(uwTick >= transDelay + 1){ // +1 wrap around to handle INTMAX
		if(gAltitude < prevAlt)
			return true; // no longer rising

		// still rising, set-up next alt comparison
		prevAlt = gAltitude;
		transDelay = uwTick + 500; // sample every 0.5 sec for apo
	}
	return false;
}

bool apogeeTransition(void) { return true; }
bool drogueDescentTransition(void) { return true; }
bool mainDescentTransition(void) { return true; }
bool landedTransition(void) { return true; }
