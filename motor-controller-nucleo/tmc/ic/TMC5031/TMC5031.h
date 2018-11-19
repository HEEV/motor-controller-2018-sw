/*
 * TMC5031.h
 *
 *  Created on: 07.07.2017
 *      Author: LK
 *    Based on: TMC562-MKL.h (26.01.2012 OK)
 */

#ifndef TMC_IC_TMC5031_H_
#define TMC_IC_TMC5031_H_

#include "../../helpers/API_Header.h"
#include "TMC5031_Register.h"
#include "TMC5031_Mask_Shift.h"
#include "TMC5031_Constants.h"

#define TMC5031_FIELD_READ(motor, address, mask, shift) \
	FIELD_READ(tmc5031_readInt, motor, address, mask, shift)
#define TMC5031_FIELD_WRITE(motor, address, mask, shift, value) \
	FIELD_WRITE(tmc5031_writeInt, motor, address, mask, shift, value)
#define TMC5031_FIELD_UPDATE(motor, address, mask, shift, value) \
	FIELD_UPDATE(tmc5031_readInt, tmc5031_writeInt, motor, address, mask, shift, value)

// Usage note: use 1 TypeDef per IC
typedef struct {
	int velocity[2], oldX[2];
	uint32 oldTick;
	int32 registerResetState[TMC5031_REGISTER_COUNT];
	uint8 registerAccess[TMC5031_REGISTER_COUNT];
	bool vMaxModified[2];
} TMC5031TypeDef;

void tmc5031_initConfig(TMC5031TypeDef *TMC5031);
void tmc5031_periodicJob(u8 motor, uint32 tick, TMC5031TypeDef *TMC5031, ConfigurationTypeDef *TMC5031_config);
u8 tmc5031_reset(ConfigurationTypeDef *TMC5031_config);
u8 tmc5031_restore(ConfigurationTypeDef *TMC5031_config);

#endif /* TMC_IC_TMC5031_H_ */
