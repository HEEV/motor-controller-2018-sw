/*
 * TMC2041.h
 *
 *  Created on: 14.08.2017
 *      Author: LK
 */

#ifndef TMC_IC_TMC5072_H_
#define TMC_IC_TMC5072_H_

#include "../../helpers/API_Header.h"
#include "TMC2041_Register.h"
#include "TMC2041_Mask_Shift.h"

#define TMC2041_REGISTER_COUNT   TMC_REGISTER_COUNT
#define TMC2041_MOTORS           2
#define TMC2041_WRITE_BIT        TMC_WRITE_BIT
#define TMC2041_ADDRESS_MASK     TMC_ADDRESS_MASK
#define TMC2041_MAX_VELOCITY     s32_MAX
#define TMC2041_MAX_ACCELERATION u24_MAX

#define TMC2041_FIELD_READ(motor, address, mask, shift) \
	FIELD_READ(tmc2041_readInt, motor, address, mask, shift)
#define TMC2041_FIELD_WRITE(motor, address, mask, shift, value) \
	FIELD_WRITE(tmc2041_writeInt, motor, address, mask, shift, value)
#define TMC2041_FIELD_UPDATE(motor, address, mask, shift, value) \
	FIELD_UPDATE(tmc2041_readInt, tmc2041_writeInt, motor, address, mask, shift, value)

typedef struct {
	int32 registerResetState[TMC2041_REGISTER_COUNT];
	uint8 registerAccess[TMC2041_REGISTER_COUNT];
} TMC2041TypeDef;

void tmc2041_initConfig(TMC2041TypeDef *TMC2041);
void tmc2041_periodicJob(uint8 motor, uint32 tick, TMC2041TypeDef *TMC2041, ConfigurationTypeDef *TMC2041_config);
uint8 tmc2041_reset(ConfigurationTypeDef *TMC2041_config);
uint8 tmc2041_restore(ConfigurationTypeDef *TMC2041_config);

#endif /* TMC_IC_TMC5072_H_ */
