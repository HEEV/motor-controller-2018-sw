/*
 * TMC5130.h
 *
 *  Created on: 03.07.2017
 *      Author: LK
 */

#ifndef TMC_IC_TMC5130_H_
#define TMC_IC_TMC5130_H_

#include "../../helpers/Constants.h"
#include "../../helpers/API_Header.h"
#include "TMC5130_Register.h"
#include "TMC5130_Mask_Shift.h"
#include "TMC5130_Constants.h"

// Helper macros
#define TMC5130_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc5130_readInt(tdef, address), mask, shift)
#define TMC5130_FIELD_UPDATE(tdef, address, mask, shift, value) \
	(tmc5130_writeInt(tdef, address, FIELD_SET(tmc5130_readInt(tdef, address), mask, shift, value)))

// Typedefs
typedef struct
{
	ConfigurationTypeDef *config;
	int velocity, oldX;
	uint32 oldTick;
	int32 registerResetState[TMC5130_REGISTER_COUNT];
	uint8 registerAccess[TMC5130_REGISTER_COUNT];
} TMC5130TypeDef;

typedef void (*tmc5130_callback)(TMC5130TypeDef*, ConfigState);

// Default Register Values
#define R10 0x00071703  // IHOLD_IRUN
#define R3A 0x00010000  // ENC_CONST
#define R6C 0x000101D5  // CHOPCONF

static const int32 tmc5130_defaultRegisterResetState[TMC5130_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	N_A, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

#undef R10
#undef R3A
#undef R6C

// Register access permissions:
//   0x00: none (reserved)
//   0x01: read
//   0x02: write
//   0x03: read/write
//   0x13: read/write, seperate functions/values for reading or writing
//   0x21: read, flag register (read to clear)
//   0x42: write, has hardware presets on reset
static const uint8 tmc5130_defaultRegisterAccess[TMC5130_REGISTER_COUNT] =
{
//  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x03, 0x21, 0x01, 0x02, 0x13, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x00 - 0x0F
	0x02, 0x02, 0x01, 0x02, 0x02, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x10 - 0x1F
	0x03, 0x03, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, ____, 0x02, 0x02, 0x02, 0x03, ____, ____, // 0x20 - 0x2F
	____, ____, ____, 0x02, 0x03, 0x21, 0x01, ____, 0x03, 0x03, 0x02, 0x21, 0x01, ____, ____, ____, // 0x30 - 0x3F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x40 - 0x4F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x50 - 0x5F
	0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01, // 0x60 - 0x6F
	0x42, 0x01, 0x02, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____  // 0x70 - 0x7F
};

// API Functions
// All functions act on one IC given by the TMC5130TypeDef struct

void tmc5130_writeDatagram(TMC5130TypeDef *tmc5130, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
void tmc5130_writeInt(TMC5130TypeDef *tmc5130, uint8 address, int32 value);
int32 tmc5130_readInt(TMC5130TypeDef *tmc5130, uint8 address);

void tmc5130_init(TMC5130TypeDef *tmc5130, uint8 channel, ConfigurationTypeDef *tmc5130_config, const int32 *registerResetState);
uint8 tmc5130_reset(TMC5130TypeDef *tmc5130);
uint8 tmc5130_restore(TMC5130TypeDef *tmc5130);
void tmc5130_setRegisterResetState(TMC5130TypeDef *tmc5130, const int32 *resetState);
void tmc5130_setCallback(TMC5130TypeDef *tmc5130, tmc5130_callback callback);
void tmc5130_periodicJob(TMC5130TypeDef *tmc5130, uint32 tick);

void tmc5130_rotate(TMC5130TypeDef *tmc5130, int32 velocity);
void tmc5130_right(TMC5130TypeDef *tmc5130, uint32 velocity);
void tmc5130_left(TMC5130TypeDef *tmc5130, uint32 velocity);
void tmc5130_stop(TMC5130TypeDef *tmc5130);
void tmc5130_moveTo(TMC5130TypeDef *tmc5130, int32 position, uint32 velocityMax);
void tmc5130_moveBy(TMC5130TypeDef *tmc5130, int32 *ticks, uint32 velocityMax);

#endif /* TMC_IC_TMC5130_H_ */
