/*
 * TMC5072.h
 *
 *  Created on: 06.07.2017
 *      Author: LK
 *    Based on: TMC562-MKL.h (26.01.2012 OK)
 */

#ifndef TMC_IC_TMC5072_H_
#define TMC_IC_TMC5072_H_

#include "../../helpers/API_Header.h"
#include "TMC5072_Register.h"
#include "TMC5072_Mask_Shift.h"
#include "TMC5072_Constants.h"

#define TMC5072_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc5072_readInt(tdef, address), mask, shift)
#define TMC5072_FIELD_UPDATE(tdef, address, mask, shift, value) \
	(tmc5072_writeInt(tdef, address, FIELD_SET(tmc5072_readInt(tdef, address), mask, shift, value)))
#define TMC5072_FIELD_WRITE(tdef, address, mask, shift, value) \
	(tmc5072_writeInt(tdef, address, ((value)<<(shift)) & (mask)))

// Usage note: use 1 TypeDef per IC
typedef struct {
	int velocity[2], oldX[2]; // for periodic job
	uint32 oldTick; // dito
	int32 registerResetState[TMC5072_REGISTER_COUNT];
	uint8 registerAccess[TMC5072_REGISTER_COUNT];
	bool vMaxModified[2];
	u8 channel;
	int32 *shadowRegister;
} TMC5072TypeDef;

void tmc5072_writeDatagram(TMC5072TypeDef *tmc5072, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
void tmc5072_writeInt(TMC5072TypeDef *tmc5072, uint8 address, int value);
int tmc5072_readInt(TMC5072TypeDef *tmc5072, uint8 address);

void tmc5072_initConfig(TMC5072TypeDef *TMC5072);
void tmc5072_periodicJob(u8 channel, uint32 tick, TMC5072TypeDef *TMC5072, ConfigurationTypeDef *TMC5072_config);
u8 tmc5072_reset(ConfigurationTypeDef *TMC5072_config);
u8 tmc5072_restore(ConfigurationTypeDef *TMC5072_config);

u32 tmc5072_rotate(TMC5072TypeDef *tmc5072, u8 motor, s32 velocity);
u32 tmc5072_right(TMC5072TypeDef *tmc5072, u8 motor, s32 velocity);
u32 tmc5072_left(TMC5072TypeDef *tmc5072, u8 motor, s32 velocity);
u32 tmc5072_stop(TMC5072TypeDef *tmc5072, u8 motor);
u32 tmc5072_moveTo(TMC5072TypeDef *tmc5072, u8 motor, s32 position);
u32 tmc5072_moveBy(TMC5072TypeDef *tmc5072, uint8 motor, int32 *ticks);

#endif /* TMC_IC_TMC5072_H_ */
