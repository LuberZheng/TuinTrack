
/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "string.h"

#include "temperatureservice.h"
#include "st_util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/* Service configuration values */
#define SENSOR_SERVICE_UUID     TEMPERATURESERVICE_SERV_UUID
#define SENSOR_DATA_UUID        TEMPERATURESERVICE_DATA_UUID
#define SENSOR_CONFIG_UUID      TEMPERATURESERVICE_CONF_UUID
#define SENSOR_PERIOD_UUID      TEMPERATURESERVICE_PERI_UUID

#define SENSOR_SERVICE          TEMPERATURESERVICE_SERVICE
#define SENSOR_DATA_LEN         TEMPERATURESERVICE_DATA_LEN

#define SENSOR_DATA_DESCR       "Temp. Data"
#define SENSOR_CONFIG_DESCR     "Temp. Conf."
#define SENSOR_PERIOD_DESCR     "Temp. Period"

// The temperature sensor does not support the 100 ms update rate
#undef SENSOR_MIN_UPDATE_PERIOD
#define SENSOR_MIN_UPDATE_PERIOD  1000 // Minimum 300 milliseconds

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
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID
static CONST uint8_t sensorServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_SERVICE_UUID),
};

// Characteristic UUID: data
static CONST uint8_t sensorDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA_UUID),
};

// Characteristic UUID: config
static CONST uint8_t sensorCfgUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG_UUID),
};

// Characteristic UUID: period
static CONST uint8_t sensorPeriodUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_PERIOD_UUID),
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static sensorCBs_t *sensor_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t sensorService = { TI_UUID_SIZE, sensorServiceUUID };

// Characteristic Value: data
static uint8_t sensorData[SENSOR_DATA_LEN] = { 0, };

// Characteristic Properties: data
static uint8_t sensorDataProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data
static gattCharCfg_t *sensorDataConfig;

// Characteristic Properties: configuration
static uint8_t sensorCfgProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8_t sensorCfg = 0;

// Characteristic Properties: period
static uint8_t sensorPeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period
static uint8_t sensorPeriod = 5; //5s

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t sensorAttrTable[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&sensorService                 /* pValue */
  },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorDataProps
    },

      // Characteristic Value "Data"
      {
        { TI_UUID_SIZE, sensorDataUUID },
        GATT_PERMIT_READ,
        0,
        sensorData
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&sensorDataConfig
      },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCfgProps
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, sensorCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorCfg
      },

     // Characteristic Declaration "Period"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorPeriodProps
    },

      // Characteristic Value "Period"
      {
        { TI_UUID_SIZE, sensorPeriodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorPeriod
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// IR Temperature Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and 
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the 
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an 
// operation on a characteristic requires authorization the Stack will call 
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be 
// made within these functions.
static CONST gattServiceCBs_t sensorCBs =
{
	sensor_ReadAttrCB,  // Read callback function pointer
	sensor_WriteAttrCB, // Write callback function pointer
	NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Temperature_addService
 *
 * @brief   Initializes the IR Temperature Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Temperature_addService(void)
{
	// Allocate Client Characteristic Configuration table
	sensorDataConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
													linkDBNumConns);
	if (sensorDataConfig == NULL)
	{
		return (bleMemAllocError);
	}

	// Register with Link DB to receive link status change callback
	GATTServApp_InitCharCfg(INVALID_CONNHANDLE, sensorDataConfig);

	// Register GATT attribute list and CBs with GATT Server App
	return GATTServApp_RegisterService(sensorAttrTable,
								  	  GATT_NUM_ATTRS (sensorAttrTable),
									  GATT_MAX_ENCRYPT_KEY_SIZE,
									  &sensorCBs);
}


/*********************************************************************
 * @fn      Temperature_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Temperature_registerAppCBs(sensorCBs_t *appCallbacks)
{
	if (sensor_AppCBs == NULL)
	{
		if (appCallbacks != NULL)
		{
			sensor_AppCBs = appCallbacks;
		}

		return (SUCCESS);
	}

	return (bleAlreadyInRequestedMode);
}

/*********************************************************************
 * @fn      Temperature_setParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Temperature_setParameter(uint8_t param, uint8_t len, void *value)
{
	bStatus_t ret = SUCCESS;

	switch (param)
	{
	case SENSOR_DATA:
		if (len == SENSOR_DATA_LEN)
		{
			memcpy(sensorData, value, SENSOR_DATA_LEN);
			// See if Notification has been enabled
			ret = GATTServApp_ProcessCharCfg(sensorDataConfig, sensorData, FALSE,
												sensorAttrTable,
												GATT_NUM_ATTRS(sensorAttrTable),
												INVALID_TASK_ID, sensor_ReadAttrCB);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;

	case SENSOR_CONF:
		if (len == sizeof(uint8_t))
		{
			sensorCfg = *((uint8_t*)value);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;

	case SENSOR_PERI:
		if (len == sizeof(uint8_t))
		{
			sensorPeriod = *((uint8_t*)value);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;

	default:
		ret = INVALIDPARAMETER;
		break;
	}

	return (ret);
}

/*********************************************************************
 * @fn      Temperature_getParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Temperature_getParameter(uint8_t param, void *value)
{
	bStatus_t ret = SUCCESS;

	switch (param)
	{
	case SENSOR_DATA:
		memcpy(value, sensorData, SENSOR_DATA_LEN);
		break;

	case SENSOR_CONF:
		*((uint8_t*)value) = sensorCfg;
		break;

	case SENSOR_PERI:
		*((uint8_t*)value) = sensorPeriod;
		break;

	default:
		ret = INVALIDPARAMETER;
		break;
	}

	return (ret);
}

/*********************************************************************
 * @fn          sensor_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
extern uint8_t  gMsgBuffer[];
extern uint16_t gMsgLength;

static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method)
{
  uint16_t uuid;
  bStatus_t status = SUCCESS;
  //uint16_t len;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case SENSOR_DATA_UUID:
		*pLen = SENSOR_DATA_LEN;
		memcpy(pValue, pAttr->pValue, SENSOR_DATA_LEN);
      break;

    case SENSOR_CONFIG_UUID:
    case SENSOR_PERIOD_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return (status);
}

/*********************************************************************
 * @fn      sensor_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method)
{
	bStatus_t status = SUCCESS;
	uint8_t notifyApp = 0xFF;
	uint16_t uuid;

	if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
		// Invalid handle
		return ATT_ERR_INVALID_HANDLE;
	}

	switch (uuid)
	{
	case SENSOR_DATA_UUID:
	// Should not get here
	break;

	case SENSOR_CONFIG_UUID:
		// Validate the value
		// Make sure it's not a blob oper
		if (offset == 0)
		{
			if (len != 1)
			{
				status = ATT_ERR_INVALID_VALUE_SIZE;
			}
		}
		else
		{
			status = ATT_ERR_ATTR_NOT_LONG;
		}

		// Write the value
		if (status == SUCCESS)
		{
			uint8_t *pCurValue = (uint8_t *)pAttr->pValue;

			*pCurValue = pValue[0];

			if (pAttr->pValue == &sensorCfg)
			{
				notifyApp = SENSOR_CONF;
			}
		}
		break;

	case SENSOR_PERIOD_UUID:
		// Validate the value
		// Make sure it's not a blob oper
		if (offset == 0)
		{
			if (len != 1)
			{
				status = ATT_ERR_INVALID_VALUE_SIZE;
			}
		}
		else
		{
			status = ATT_ERR_ATTR_NOT_LONG;
		}
		// Write the value
		if (status == SUCCESS)
		{
			if (pValue[0]>=(SENSOR_MIN_UPDATE_PERIOD/SENSOR_PERIOD_RESOLUTION))
			{

				uint8_t *pCurValue = (uint8_t *)pAttr->pValue;
				*pCurValue = pValue[0];

				if (pAttr->pValue == &sensorPeriod)
				{
					notifyApp = SENSOR_PERI;
				}
			}
			else
			{
				status = ATT_ERR_INVALID_VALUE;
			}
		}
		break;

	case GATT_CLIENT_CHAR_CFG_UUID:
		status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
					  	  	  	  	  	  	  	  offset, GATT_CLIENT_CFG_NOTIFY);
	break;

	default:
		// Should never get here!
		status = ATT_ERR_ATTR_NOT_FOUND;
	break;
	}

	// If a characteristic value changed then callback function
	// to notify application of change
	if ((notifyApp != 0xFF) && sensor_AppCBs && sensor_AppCBs->pfnSensorChange)
	{
		sensor_AppCBs->pfnSensorChange(notifyApp);
	}

	return (status);
}

/*********************************************************************
*********************************************************************/
