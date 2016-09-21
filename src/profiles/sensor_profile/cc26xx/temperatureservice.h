
#ifndef TEMPERATURESERVICE_H
#define TEMPERATURESERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "st_util.h"

/*********************************************************************
 * CONSTANTS
 */

// Service UUID
#define TEMPERATURESERVICE_SERV_UUID         0xAA90
#define TEMPERATURESERVICE_DATA_UUID         0xAA91
#define TEMPERATURESERVICE_CONF_UUID         0xAA92
#define TEMPERATURESERVICE_PERI_UUID         0xAA93

// Length of sensor data in bytes
#define TEMPERATURESERVICE_DATA_LEN          20

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Temperature_addService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t Temperature_addService(void);

/*
 * Temperature_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Temperature_registerAppCBs(sensorCBs_t *appCallbacks);

/*
 * Temperature_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Temperature_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * Temperature_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Temperature_getParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TEMPERATURESERVICE_H */

