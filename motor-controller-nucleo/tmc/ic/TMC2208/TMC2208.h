/*
 * TMC2208.h
 *
 *  Created on: 07.07.2017
 *      Author: LK
 */

#ifndef TMC_IC_TMC2208_H_
#define TMC_IC_TMC2208_H_

#include "../../helpers/API_Header.h"
#include "TMC2208_Register.h"

#define TMC2208_MOTORS           1
#define TMC2208_REGISTER_COUNT   TMC_REGISTER_COUNT
#define TMC2208_WRITE_BIT        TMC_WRITE_BIT
#define TMC2208_ADDRESS_MASK     TMC_ADDRESS_MASK
#define TMC2208_MAX_VELOCITY     s32_MAX
#define TMC2208_MAX_ACCELERATION u24_MAX

// Usage note: use 1 TypeDef per IC
typedef struct {
	int velocity;
	int oldX;
	uint32 oldTick;
	int32 registerResetState[TMC2208_REGISTER_COUNT];
	uint8 registerAccess[TMC2208_REGISTER_COUNT];
	//bool vMaxModified;
} TMC2208TypeDef;

void tmc2208_initConfig(TMC2208TypeDef *TMC2208);
void tmc2208_periodicJob(u8 motor, uint32 tick, TMC2208TypeDef *TMC2208, ConfigurationTypeDef *TMC2208_config);
u8 tmc2208_reset(ConfigurationTypeDef *TMC2208_config);
u8 tmc2208_restore(ConfigurationTypeDef *TMC2208_config);

#endif /* TMC_IC_TMC2208_H_ */
