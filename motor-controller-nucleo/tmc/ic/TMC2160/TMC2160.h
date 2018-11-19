/*
* TMC2160.h
*
*  Created on: 13.08.2018
*      Author: LK
*/

#ifndef TMC_IC_TMC2160_H_
#define TMC_IC_TMC2160_H_

#include "../../helpers/API_Header.h"
#include "TMC2160_Mask_Shift.h"
#include "TMC2160_Register.h"

#define TMC2160_REGISTER_COUNT   TMC_REGISTER_COUNT
#define TMC2160_MOTORS           1
#define TMC2160_WRITE_BIT        TMC_WRITE_BIT
#define TMC2160_ADDRESS_MASK     TMC_ADDRESS_MASK
#define TMC2160_MAX_VELOCITY     s32_MAX
#define TMC2160_MAX_ACCELERATION u24_MAX

// Helper macros
#define TMC2160_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc2160_readInt(tdef, address), mask, shift)
#define TMC2160_FIELD_UPDATE(tdef, address, mask, shift, value) \
	(tmc2160_writeInt(tdef, address, FIELD_SET(tmc2160_readInt(tdef, address), mask, shift, value)))

typedef struct
{
	ConfigurationTypeDef *config;
	int32 registerResetState[TMC2160_REGISTER_COUNT];
	uint8 registerAccess[TMC2160_REGISTER_COUNT];
} TMC2160TypeDef;

typedef void (*tmc2160_callback)(TMC2160TypeDef*, ConfigState);

// Default Register values
#define R10 0x00070A03  // IHOLD_IRUN
#define R6C 0x00410153  // CHOPCONF
#define R70 0xC40C001E  // PWMCONF

// Register access permissions:
// 0x00: none (reserved)
// 0x01: read
// 0x02: write
// 0x03: read/write
// 0x11: read to clear
// 0x42: write, has hardware presets on reset
static const uint8 tmc2160_defaultRegisterAccess[TMC2160_REGISTER_COUNT] = {
//  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x03, 0x11, ____, ____, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x00 - 0x0F
	0x02, 0x02, 0x01, 0x02, 0x02, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x10 - 0x1F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, 0x03, ____, ____, // 0x20 - 0x2F
	____, ____, ____, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x30 - 0x3F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x40 - 0x4F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x50 - 0x5F
	0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01, // 0x60 - 0x6F
	0x02, 0x01, 0x02, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____  // 0x70 - 0x7F
};

static const int32 tmc2160_defaultRegisterResetState[TMC2160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

#undef R10
#undef R6C
#undef R70

void tmc2160_writeDatagram(TMC2160TypeDef *tmc2160, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
void tmc2160_writeInt(TMC2160TypeDef *tmc2160, uint8 address, int32 value);
int32 tmc2160_readInt(TMC2160TypeDef *tmc2160, uint8 address);

void tmc2160_init(TMC2160TypeDef *tmc2160, uint8 channel, ConfigurationTypeDef *config, const int32 *registerResetState);
uint8 tmc2160_reset(TMC2160TypeDef *tmc2160);
uint8 tmc2160_restore(TMC2160TypeDef *tmc2160);
void tmc2160_setRegisterResetState(TMC2160TypeDef *tmc2160, const int32 *resetState);
void tmc2160_setCallback(TMC2160TypeDef *tmc2160, tmc2160_callback callback);
void tmc2160_periodicJob(TMC2160TypeDef *tmc2160, uint32 tick);

#endif /* TMC_IC_TMC2160_H_ */
