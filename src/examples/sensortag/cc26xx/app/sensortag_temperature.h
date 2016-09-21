
#ifndef SENSORTAGTEMPERATURE_H
#define SENSORTAGTEMPERATURE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "sensortag.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

#ifndef EXCLUDE_TEMPERATURE
/*
 * Create the IR temperature sensor task
 */
extern void SensorTagTemperature_createTask(void);

/*
 * Initialize Temperature sensor module
 */
extern void SensorTagTemperature_init(void);

/*
 * Task Event Processor for characteristic changes
 */
extern void SensorTagTemperature_processCharChangeEvt(uint8_t paramID);

/*
 * Reset Temperature sensor module
 */
extern void SensorTagTemperature_reset(void);

extern void SensorTagTemp_processKeyRight(void);

extern void SensorTagTemp_processKeyLeft(void);

#else

/* Temperature module not included */

#define SensorTagTemperature_createTask()
#define SensorTagTemperature_init()
#define SensorTagTemperature_processCharChangeEvt(paramID)
#define SensorTagTemperature_reset()

#endif // EXCLUDE_TMP

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAGTEMPERATURE_H */
