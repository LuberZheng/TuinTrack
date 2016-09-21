
#ifndef EXCLUDE_TEMPERATRURE
/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"

#include "temperatureservice.h"
#include "sensortag_temperature.h"
#include "SensorTmp007.h"
#include "SensorHdc1000.h"
#include "SensorTagTest.h"
#include "SensorUtil.h"
#include "board.h"
#include "util.h"

#include "string.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include <driverlib/aon_rtc.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define UNIXTIME_START          10000

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   5000
#define SENSOR_DISABLE_PERIOD   1000

// Delay from sensor enable to reading measurement
// (allow for 250 ms conversion time)
#define TEMP_MEAS_DELAY         500

// Time start measurement and data ready
#define HUM_DELAY_PERIOD        15

// Length of the data for this sensor
#define SENSOR_DATA_LEN         TEMPERATURESERVICE_DATA_LEN

// Event flag for this sensor
#define SENSOR_EVT              ST_TEMPERATURE_SENSOR_EVT

// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  600

#define TEMP_LOG_SIZE           512
#define TEMP_READ_PERIOD        5

// Key Values
#define SENSOR_KEY_LEFT                   0x01
#define SENSOR_KEY_RIGHT                  0x02

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;

// Semaphore used to post events to the application thread
static ICall_Semaphore sensorSem;

// Task setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

static Task_Struct temperatureTask;
static Char temperatureTaskStack[SENSOR_TASK_STACK_SIZE];

// Parameters
static uint8_t  sensorConfig;
static uint16_t sensorPeriod;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTaskFxn(UArg a0, UArg a1);
static void temperatureTaskFxn(UArg a0, UArg a1);

static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);

static char* stringAddInteger(int num, char *str);
static char* stringAddFloat(int num, int den, char *str);
static int   stringGetLength(char *str);

static void processGapStateChange(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorConfigChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagTemperature_createTask
 *
 * @brief   Task creation function for the SensorTag
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTemperature_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = temperatureTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&temperatureTask, temperatureTaskFxn, &taskParames, NULL);

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = sensorTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);

}

/*********************************************************************
 * @fn      SensorTagTemperature_processCharChangeEvt
 *
 * @brief   SensorTag Temperature event handling
 *
 * @param   paramID - identifies the characteristic that was changed
 *
 * @return  none
 *
 */
void SensorTagTemperature_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;

  switch (paramID)
  {
  case SENSOR_CONF:
    if ((SensorTag_testResult() & SENSOR_TMP_TEST_BM) == 0)
    {
      sensorConfig = ST_CFG_ERROR;
    }

    if (sensorConfig != ST_CFG_ERROR)
    {
      Temperature_getParameter(SENSOR_CONF, &newValue);

      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Reset characteristics
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);

        // Deactivate task
        Task_setPri(Task_handle(&sensorTask), -1);
      }
      else
      {
        Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);
      }

      sensorConfig = newValue;
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
    }
    break;

  case SENSOR_PERI:
    Temperature_getParameter(SENSOR_PERI, &newValue);
    sensorPeriod = newValue * 1000/*SENSOR_PERIOD_RESOLUTION*/;
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagTmp_init
 *
 * @brief   Initialize the module
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTemperature_init(void)
{
  // Add service
  Temperature_addService();

  // Register callbacks with profile
  Temperature_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;

  // Initialize characteristics and sensor driver
  SensorTagTemperature_reset();
  initCharacteristicValue(SENSOR_PERI, sensorPeriod/1000, sizeof(uint8_t));
}

/*********************************************************************
 * @fn      SensorTagTmp_reset
 *
 * @brief   Reset characteristics
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTemperature_reset(void)
{
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));

  // Initialize the driver
  SensorHdc1000_init();
}


/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the temperature readout task
 *
 * @param   a0 (not used)
 *
 * @param   a1 (not used)
 *
 * @return  none
 */

#define TEMP_HOUR_PERIOD         72
#define TEMP_SECONDS_INTERVAL    300 //5  //300 // 300s = 5min
#define TEMP_5MINUTE_INTERVAL    12  //3  //12  // 12*300s = 1 hour

static uint16_t gTempLogCounter = 0;
static int16_t  gTemperatureArray[TEMP_HOUR_PERIOD + 1];
       uint8_t  gMsgBuffer[TEMP_LOG_SIZE];
       uint16_t gMsgLength = 0;
static uint32_t gCurrSeconds;

static uint8_t  sensorTagkeys;

//static int      keyRightTimer;
//static int      keyLeftTimer;
extern uint16_t keyLeftTimer;
extern uint16_t keyRightTimer;
static Clock_Struct periodicClock;

#ifndef Board_KEY_RIGHT
#define Board_KEY_RIGHT         Board_BTN2
#endif

#ifndef Board_KEY_LEFT
#define Board_KEY_LEFT          Board_BTN1
#endif

#define RESET_PRESS_PERIOD     1  // 1s
#define POWER_PRESS_PERIOD     1 // 10s

static void sensorTaskFxn(UArg a0, UArg a1)
{
	int i, len;

	// Register task with BLE stack
	ICall_registerApp(&sensorSelfEntity, &sensorSem);

	// Deactivate task (active only when measurement is enabled)
	Task_setPri(Task_handle(&sensorTask), -1);

	// Task loop
	while (true)
	{

		if (sensorConfig == ST_CFG_SENSOR_DISABLE)
		{
			DELAY_MS(SENSOR_DISABLE_PERIOD);
			continue;
		}

#if 0
		gMsgBuffer[0] = 'T';
		gMsgBuffer[1] = 'M';
		gMsgBuffer[2] = 'P';
		gMsgBuffer[3] = ':';
		gMsgBuffer[4] = 0x30 +   gTemperatureArray[0]/100;
		gMsgBuffer[5] = 0x30 + ((gTemperatureArray[0]/10)%10);
		gMsgBuffer[6] = 0x30 +  (gTemperatureArray[0]%10);
		gMsgBuffer[7] = 0x00;
		Temperature_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, gMsgBuffer);
#endif

		len = gMsgLength;
		for(i=0;len>0;i++)
		{
			Temperature_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, &gMsgBuffer[i*SENSOR_DATA_LEN]);
			len = len - SENSOR_DATA_LEN;
			DELAY_MS(HUM_DELAY_PERIOD);
		}

		DELAY_MS(sensorPeriod);
	}
}

static void temperatureTaskFxn(UArg a0, UArg a1)
{
	int i;

	uint16_t rawTemp, rawHum;
	int16_t realTemp;
	int32_t intTemp;
	int16_t curTemperature;
	int16_t maxTemperature;

	uint8_t  prevTagkeys;

	uint32_t curr_rtc_sec, prev_rtc_sec, prev_1s_sec;
	uint32_t rtc_5min_counter;

	AONRTCEnable();

	memset(gTemperatureArray, 0, sizeof(gTemperatureArray));

	// Activate task
	Task_setPri(Task_handle(&temperatureTask), SENSOR_TASK_PRIORITY);

	prevTagkeys = sensorTagkeys = 0;

	//keyRightTimer = 0;
	//keyLeftTimer  = 0;

	maxTemperature = -40;
	rtc_5min_counter = 0;
	gTempLogCounter = 0;

	gCurrSeconds = prev_1s_sec = curr_rtc_sec = prev_rtc_sec = AONRTCSecGet();

	gMsgBuffer[0] = '#';
	gMsgBuffer[1] = '\0';
	stringAddInteger((UNIXTIME_START+gCurrSeconds), (char*)gMsgBuffer);
	stringAddInteger(gTempLogCounter, (char*)gMsgBuffer);
	gMsgLength = stringGetLength((char*)gMsgBuffer);

	// Task loop
	while (true)
	{
		gCurrSeconds = curr_rtc_sec = AONRTCSecGet();
		if(curr_rtc_sec < prev_rtc_sec)
			prev_rtc_sec = curr_rtc_sec;
		if(curr_rtc_sec < prev_1s_sec)
			prev_1s_sec = curr_rtc_sec;

		if(curr_rtc_sec - prev_1s_sec >= 1)
		{
		    // Both keys have been pressed for 6 seconds -> restore factory image
		    if (keyLeftTimer >= RESET_PRESS_PERIOD && keyRightTimer >= RESET_PRESS_PERIOD)
		    {
				//keyLeftTimer  = 0;
				//keyRightTimer = 0;
				gTempLogCounter = 0;

				memset(gMsgBuffer, 0, SENSOR_DATA_LEN);
				gMsgBuffer[0] = '#';
				stringAddInteger((UNIXTIME_START+gCurrSeconds), (char*)gMsgBuffer);
				stringAddInteger(gTempLogCounter, (char*)gMsgBuffer);
				gMsgLength = stringGetLength((char*)gMsgBuffer);
		    }

		    if(0 && prevTagkeys != sensorTagkeys)
		    {
				// Insert key state into advertising data
				if (gapProfileState == GAPROLE_ADVERTISING)
				{
					SensorTag_updateAdvertisingData(sensorTagkeys);
				}

				// Check if right key was pressed
				if ((sensorTagkeys & SENSOR_KEY_RIGHT)!=0 && (prevTagkeys & SENSOR_KEY_RIGHT)==0 && (sensorTagkeys & SENSOR_KEY_LEFT)==0)
				{
					if (gapProfileState != GAPROLE_CONNECTED)
					{
						// Not connected; change state immediately (power/right button)
						processGapStateChange();
					}
				}
				prevTagkeys = sensorTagkeys;
		    }

			if ( 0 & keyLeftTimer == 0 && keyRightTimer >= POWER_PRESS_PERIOD)
		    {
				// Insert key state into advertising data
				if (gapProfileState == GAPROLE_ADVERTISING)
				{
					SensorTag_updateAdvertisingData(SENSOR_KEY_RIGHT);
				}

				if (gapProfileState == GAPROLE_CONNECTED || gapProfileState == GAPROLE_ADVERTISING)
				{
					processGapStateChange();
				}
				else if (gapProfileState != GAPROLE_CONNECTED)
				{
					// Not connected; change state immediately (power/right button)
					processGapStateChange();
				}
		    }
		    prev_1s_sec = curr_rtc_sec;
		}

		if(curr_rtc_sec - prev_rtc_sec >= TEMP_SECONDS_INTERVAL)
		{
			prev_rtc_sec = curr_rtc_sec;
			rtc_5min_counter++;

			// 1. Start Humidity measurement
			SensorHdc1000_start();
			DELAY_MS(HUM_DELAY_PERIOD);

			// 2. Read data
			SensorHdc1000_read(&rawTemp, &rawHum);

			realTemp = (int16_t)rawTemp;
			intTemp  = (int32_t)realTemp;
			intTemp  = (intTemp*1650 / 65536) - 400;
			curTemperature = (int16_t)intTemp;

			if( maxTemperature < curTemperature )
				maxTemperature = curTemperature;

			if(rtc_5min_counter==TEMP_5MINUTE_INTERVAL)
			{
				rtc_5min_counter=0;
				if(gTempLogCounter==TEMP_HOUR_PERIOD)
				{
					gTempLogCounter = TEMP_HOUR_PERIOD - 1;
					for(i=0; i<TEMP_HOUR_PERIOD-1; i++)
						gTemperatureArray[i] = gTemperatureArray[i+1];
					gTemperatureArray[i] = maxTemperature;
				}

				gTemperatureArray[gTempLogCounter] = maxTemperature;
				gTempLogCounter++;
				maxTemperature = -40;

				memset(gMsgBuffer, 0, SENSOR_DATA_LEN);
				gMsgBuffer[0] = '#';
				stringAddInteger((UNIXTIME_START+gCurrSeconds), (char*)gMsgBuffer);
				stringAddInteger(gTempLogCounter, (char*)gMsgBuffer);

				for(i=0; i<gTempLogCounter; i++)
				{
					//if( (gTemperatureArray[i]%10) == 0)
					//	stringAddInteger((gTemperatureArray[i]/10), (char*)gMsgBuffer);
					//else
					stringAddFloat((gTemperatureArray[i]/10), (gTemperatureArray[i]%10), (char*)gMsgBuffer);
				}
				gMsgLength = stringGetLength((char*)gMsgBuffer);
			}
		}

		DELAY_MS(TEMP_MEAS_DELAY);
	}
}

static char* stringAddInteger(int num, char *str)
{
    char const digit[] = "0123456789";
    char *p = str;
    int shifter;

    if(p[0]=='#')
    	p++;

    if(p[0]!='\0')
    {
        while(*p++!='\0');
        p[-1]=',';
    }

    if(num<0){
        *p++ = '-';
        num *= -1;
    }
    shifter = num;

    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    } while(shifter);

    *p = '\0';

    do{ //Move back, inserting digits as u go
        *--p = digit[num%10];
        num = num/10;
    } while(num);

    return str;
}

static char* stringAddFloat(int num, int den, char *str)
{
    char const digit[] = "0123456789";
    char *p = str;
    int shifter;

    if(p[0]=='#')
    	p++;

    if(p[0]!='\0')
    {
        while(*p++!='\0');
        p[-1]=',';
    }

    if(num<0){
        *p++ = '-';
        num *= -1;
    }
    shifter = num;

    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    } while(shifter);

    p[0] = '.';
    p[1] = digit[den];
    p[2] = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[num%10];
        num = num/10;
    } while(num);

    return str;
}

static int stringGetLength(char *str)
{
	int len = 0;
    while(str[len++]!='\0');
    return len;
}

/*********************************************************************
 * @fn      SensorTagKeys_processKeyRight
 *
 * @brief   Interrupt handler for BUTTON 1(right)
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTemp_processKeyRight(void)
{
	if (PIN_getInputValue(Board_KEY_RIGHT))
	{
		sensorTagkeys &= ~SENSOR_KEY_RIGHT;
		//keyRightTimer=0;
	}
	else
	{
		sensorTagkeys |= SENSOR_KEY_RIGHT;
		//keyRightTimer++;
	}
}

/*********************************************************************
 * @fn      SensorTagKeys_processKeyLeft
 *
 * @brief   Interrupt handler for BUTTON 2 (left)
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTemp_processKeyLeft(void)
{
	if (PIN_getInputValue(Board_KEY_LEFT))
	{
		sensorTagkeys &= ~SENSOR_KEY_LEFT;
		//keyLeftTimer=0;
	}
	else
	{
		sensorTagkeys |= SENSOR_KEY_LEFT;
		//keyLeftTimer++;
	}
}

/*********************************************************************
 * @fn      processGapStateChange
 *
 * @brief   Change the GAP state.
 *          1. Connected -> disconnect and start advertising
 *          2. Advertising -> stop advertising
 *          3. Disconnected/not advertising -> start advertising
 *
 * @param   none
 *
 * @return  none
 */
static void processGapStateChange(void)
{
	if (gapProfileState != GAPROLE_CONNECTED)
	{
		uint8_t current_adv_enabled_status;
		uint8_t new_adv_enabled_status;

		// Find the current GAP advertising status
		GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status);

		if (current_adv_enabled_status == FALSE)
		{
			new_adv_enabled_status = TRUE;
		}
		else
		{
			new_adv_enabled_status = FALSE;
		}

		// Change the GAP advertisement status to opposite of current status
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
													 &new_adv_enabled_status);
	}

	if (gapProfileState == GAPROLE_CONNECTED)
	{
		uint8_t adv_enabled = TRUE;

		// Disconnect
		GAPRole_TerminateConnection();

		// Start advertising
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_enabled);
	}
}

/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from IR Temperature Service indicating a value change
 *
 * @param   paramID - identifies the characteristic that was changed
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_TEMPERATURE, paramID);
}


/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value to be initialized
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  uint8_t data[SENSOR_DATA_LEN];

  memset(data,value,paramLen);
  Temperature_setParameter(paramID, paramLen, data);
}

#endif // EXCLUDE_TMP

/*********************************************************************
*********************************************************************/

