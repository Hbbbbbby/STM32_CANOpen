
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef SLAVENODE_SENSOR_H
#define SLAVENODE_SENSOR_H

#include "data.h"
#include "main.h"

/* Prototypes of function provided by object dictionnary */
UNS32 SlaveNode_Sensor_valueRangeTest (UNS8 typeValue, void * value);
const indextable * SlaveNode_Sensor_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data SlaveNode_Sensor_Data;
extern REAL32 AvgData_Temperature;		/* Mapped at index 0x2000, subindex 0x01 */
extern REAL32 AvgData_Humidity;		/* Mapped at index 0x2000, subindex 0x02 */

#endif // SLAVENODE_SENSOR_H
