/*
 * TMC262_1420.h
 *
 *  Created on: 06.07.2017
 *      Author: LK
 */

#ifndef TMC_IC_TMC262_1420_H_
#define TMC_IC_TMC262_1420_H_

#include "../../helpers/API_Header.h"
#include "../TMC262_1420/TMC262_1420_Constants.h"
#include "../TMC262_1420/TMC262_1420_Macros.h"
#include "../TMC262_1420/TMC262_1420_Mask_Shift.h"
#include "../TMC262_1420/TMC262_1420_Register.h"

#define TMC262_1420_REGISTER_COUNT   8
#define TMC262_1420_MOTORS           1
#define TMC262_1420_WRITE_BIT        0x08
#define TMC262_1420_ADDRESS_MASK     0x07
#define TMC262_1420_MAX_VELOCITY     s32_MAX
#define TMC262_1420_MAX_ACCELERATION u24_MAX

//// Fild access macros
//#define TMC262_1420_FIELD_READ(tdef, address, mask, shift)
//	FIELD_GET(tmc262_1420_readInt(tdef, address), mask, shift)
//#define TMC262_1420_FIELD_UPDATE(tdef, address, mask, shift, value)
//	(tmc262_1420_writeInt(tdef, address, FIELD_SET(tmc262_1420_readInt(tdef, address), mask, shift, value)))

#define TMC262_1420_FIELD_READ(motor, address, mask, shift) \
	FIELD_READ(tmc262_1420_readInt, motor, address, mask, shift)
#define TMC262_1420_FIELD_WRITE(motor, address, mask, shift, value) \
	FIELD_WRITE(tmc262_1420_writeInt, motor, address, mask, shift, value)
#define TMC262_1420_FIELD_UPDATE(motor, address, mask, shift, value) \
	FIELD_UPDATE(tmc262_1420_readInt, tmc262_1420_writeInt, motor, address | TMC262_1420_WRITE_BIT, mask, shift, value)

// Usage note: use 1 TypeDef per IC
typedef struct {
	uint8 standStillCurrentScale;
	uint32 standStillTimeout;
	uint8 isStandStillOverCurrent;
	uint8 isStandStillCurrentLimit;
	uint8 continuousModeEnable;
	uint8 runCurrentScale;
	uint8 coolStepInactiveValue;
	uint8 coolStepActiveValue;
	uint32 coolStepThreshold;
	int velocity;
	int oldX;
	uint32 oldTick;
	u8 registerAccess[TMC262_1420_REGISTER_COUNT];
	int32 registerResetState[TMC262_1420_REGISTER_COUNT];
} TMC262_1420TypeDef;

void tmc262_1420_initConfig(TMC262_1420TypeDef *TMC262_1420);
void tmc262_1420_periodicJob(u8 motor, uint32 tick, TMC262_1420TypeDef *TMC262_1420, ConfigurationTypeDef *TMC262_1420_config);
u8 tmc262_1420_reset(TMC262_1420TypeDef *TMC262_1420, ConfigurationTypeDef *TMC262_1420_config);
u8 tmc262_1420_restore(ConfigurationTypeDef *TMC262_1420_config);

#endif /* TMC_IC_TMC262_1420_H_ */
