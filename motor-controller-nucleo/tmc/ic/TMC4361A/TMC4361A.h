/*
 * TMC4361A.h
 *
 *  Created on: 18.07.2017
 *      Author: LK
 */

#ifndef TMC_IC_TMC4361A_H_
#define TMC_IC_TMC4361A_H_

#include "../../helpers/API_Header.h"
#include "TMC4361A_Register.h"
#include "TMC4361A_Mask_Shift.h"
#include "TMC4361A_Constants.h"

// Helper macros
#define TMC4361A_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc4361A_readInt(tdef, address), mask, shift)
#define TMC4361A_FIELD_UPDATE(tdef, address, mask, shift, value) \
	(tmc4361A_writeInt(tdef, address, FIELD_SET(tmc4361A_readInt(tdef, address), mask, shift, value)))

// Typedefs
typedef struct
{
	ConfigurationTypeDef *config;
	int velocity;
	int oldX;
	uint32 oldTick;
	int32 registerResetState[TMC4361A_REGISTER_COUNT];
	uint8 registerAccess[TMC4361A_REGISTER_COUNT];
	//TMotorConfig motorConfig;
	//TClosedLoopConfig closedLoopConfig;
	uint8 status;
} TMC4361ATypeDef;

typedef void (*tmc4361A_callback)(TMC4361ATypeDef*, ConfigState);

// Default Register Values
#define R10 0x00040001  // STP_LENGTH_ADD
#define R20 0x00000001  // RAMPMODE

static const int32 tmc4361A_defaultRegisterResetState[TMC4361A_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	N_A, 0,   0,   0,   0,   0,   N_A, N_A, 0,   0,   N_A, N_A, 0,   0,   0,   0,   // 0x00 - 0x0F
	R10, 0,   N_A, 0,   0,   0,   0,   N_A, 0,   0,   0,   0,   N_A, 0,   0,   N_A, // 0x10 - 0x1F
	R20, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   // 0x20 - 0x2F
	0,   N_A, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   // 0x40 - 0x4F
	0,   0,   0,   N_A, 0,   0,   N_A, N_A, N_A, 0,   0,   0,   0,   0,   0,   0,   // 0x50 - 0x5F
	0,   0,   N_A, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   // 0x60 - 0x6F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   N_A, N_A, 0,   N_A, 0    // 0x70 - 0x7F
};

#undef R10
#undef R20

// Register access permissions:
//     0x00: none (reserved)
//     0x01: read
//     0x02: write
//     0x03: read/write
//     0x13: read/write, seperate functions/values for reading or writing
//     0x42: write, has hardware presets on reset
//     0x43: read/write, has hardware presets on reset
//     0x53: read/write, has hardware presets on reset, seperate functions/values for reading or writing
static const uint8 tmc4361A_defaultRegisterAccess[TMC4361A_REGISTER_COUNT] =
{
//  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x43, 0x03, 0x03, 0x03, 0x03, 0x03, 0x43, 0x43, 0x03, 0x03, 0x43, 0x43, 0x03, 0x03, 0x13, 0x01, // 0x00 - 0x0F
	0x03, 0x03, 0x43, 0x03, 0x03, 0x03, 0x03, 0x43, 0x03, 0x03, 0x03, 0x03, 0x43, 0x03, 0x03, 0x43, // 0x10 - 0x1F
	0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, // 0x20 - 0x2F
	0x03, 0x43, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, // 0x30 - 0x3F
	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, // 0x40 - 0x4F
	0x03, 0x13, 0x13, 0x42, 0x13, 0x02, 0x42, 0x42, 0x42, 0x03, 0x13, 0x13, 0x02, 0x13, 0x02, 0x02, // 0x50 - 0x5F
	0x02, 0x02, 0x42, 0x02, ____, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x01, 0x13, 0x13, 0x01, 0x01, // 0x60 - 0x6F
	0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x01, 0x01, 0x53, 0x53, 0x02, 0x42, 0x01  // 0x70 - 0x7F
};


// API Functions
// All functions act on one IC given by the TMC4361ATypeDef struct

// SPI Communication
void tmc4361A_writeDatagram(TMC4361ATypeDef *tmc4361A, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
void tmc4361A_writeInt(TMC4361ATypeDef *tmc4361A, uint8 address, int32 value);
int32 tmc4361A_readInt(TMC4361ATypeDef *tmc4361A, uint8 address);
void tmc4361A_readWriteCover(TMC4361ATypeDef *tmc4361A, uint8 *data, size_t length);

// Configuration
void tmc4361A_init(TMC4361ATypeDef *tmc4361A, uint8 channel, ConfigurationTypeDef *config, const int32 *registerResetState);
uint8 tmc4361A_reset(TMC4361ATypeDef *tmc4361A);
uint8 tmc4361A_restore(TMC4361ATypeDef *tmc4361A);
void tmc4361A_setRegisterResetState(TMC4361ATypeDef *tmc4361A, const int32 *resetState);
void tmc4361A_setCallback(TMC4361ATypeDef *tmc4361A, tmc4361A_callback callback);
void tmc4361A_periodicJob(TMC4361ATypeDef *tmc4361A, uint32 tick);

// Motion
void tmc4361A_rotate(TMC4361ATypeDef *tmc4361A, int32 velocity);
void tmc4361A_right(TMC4361ATypeDef *tmc4361A, int32 velocity);
void tmc4361A_left(TMC4361ATypeDef *tmc4361A, int32 velocity);
void tmc4361A_stop(TMC4361ATypeDef *tmc4361A);
void tmc4361A_moveTo(TMC4361ATypeDef *tmc4361A, int32 position, uint32 velocityMax);
void tmc4361A_moveBy(TMC4361ATypeDef *tmc4361A, int32 *ticks, uint32 velocityMax);

// Helper functions
int32 tmc4361A_discardVelocityDecimals(int32 value);
uint8 tmc4361A_calibrateClosedLoop(TMC4361ATypeDef *tmc4361A, uint8 worker0master1);

#endif /* TMC_IC_TMC4361A_H_ */
