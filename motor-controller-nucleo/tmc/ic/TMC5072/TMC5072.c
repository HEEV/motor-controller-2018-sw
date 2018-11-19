/*
 * TMC5072.c
 *
 *  Created on: 06.07.2017
 *      Author: LK
 *    Based on: TMC562-MKL.h (26.01.2012 OK)
 */

#include "TMC5072.h"

// Default Register Values
#define R30 0x00071703  // IHOLD_IRUN (Motor 1)
#define R32 0x00FFFFFF  // VHIGH      (Motor 1)
#define R3A 0x00010000  // ENC_CONST  (Motor 1)
#define R50 0x00071703  // IHOLD_IRUN (Motor 1)
#define R52 0x00FFFFFF  // VHIGH      (Motor 1)
#define R5A 0x00010000  // ENC_CONST  (Motor 1)
#define R60 0xAAAAB554  // MSLUT[0]
#define R61 0x4A9554AA  // MSLUT[1]
#define R62 0x24492929  // MSLUT[2]
#define R63 0x10104222  // MSLUT[3]
#define R64 0xFBFFFFFF  // MSLUT[4]
#define R65 0xB5BB777D  // MSLUT[5]
#define R66 0x49295556  // MSLUT[6]
#define R67 0x00404222  // MSLUT[7]
#define R68 0xFFFF8056  // MSLUTSEL
#define R69 0x00F70000  // MSLUTSTART
#define R6C 0x000101D5  // CHOPCONF (Motor 1)
#define R7C 0x000101D5  // CHOPCONF (Motor 2)

/* Register access permissions:
 * 0: none (reserved)
 * 1: read
 * 2: write
 * 3: read/write
 * 7: read^write (seperate functions/values)
 */
const u8 tmc5072_defaultRegisterAccess[TMC5072_REGISTER_COUNT] = {
//	0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	3, 1, 1, 2, 7, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x00 - 0x0F
	2, 1, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, // 0x10 - 0x1F
	3, 3, 1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 3, 0, 0, // 0x20 - 0x2F
	2, 2, 2, 2, 3, 1, 1, 0, 3, 3, 2, 1, 1, 0, 0, 0, // 0x30 - 0x3F
	3, 3, 1, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 3, 0, 0, // 0x40 - 0x4F
	2, 2, 2, 2, 3, 1, 1, 0, 3, 3, 2, 1, 1, 0, 0, 0, // 0x50 - 0x5F
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 3, 2, 2, 1, // 0x60 - 0x6F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 3, 2, 2, 1  // 0x70 - 0x7F
};

const s32 tmc5072_defaultRegisterResetState[TMC5072_REGISTER_COUNT] = {
//	0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	R30, 0,   R32, 0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	R50, 0,   R52, 0,   0,   0,   0,   0,   0,   0,   R5A, 0,   0,   0,   0,   0, // 0x50 - 0x5F
	R60, R61, R62, R63, R64, R65, R66, R67, R68, R69, 0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R7C, 0,   0,   0  // 0x70 - 0x7F
};

// => SPI wrapper
extern u8 tmc5072_readWrite(u8 channel, u8 data, u8 lastTransfer);
// <= SPI wrapper

void tmc5072_writeDatagram(TMC5072TypeDef *tmc5072, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4)
{
	tmc5072_readWrite(tmc5072->channel, address | TMC5072_WRITE_BIT, FALSE);
	tmc5072_readWrite(tmc5072->channel, x1, FALSE);
	tmc5072_readWrite(tmc5072->channel, x2, FALSE);
	tmc5072_readWrite(tmc5072->channel, x3, FALSE);
	tmc5072_readWrite(tmc5072->channel, x4, TRUE);

	int value = x1;
	value <<= 8;
	value |= x2;
	value <<= 8;
	value |= x3;
	value <<= 8;
	value |= x4;

	tmc5072->shadowRegister[address & 0x7F] = value;
}

void tmc5072_writeInt(TMC5072TypeDef *tmc5072, uint8 address, int value)
{
	tmc5072_writeDatagram(tmc5072,  address, 0xFF & (value>>24), 0xFF & (value>>16), 0xFF & (value>>8), 0xFF & (value>>0));
}

int tmc5072_readInt(TMC5072TypeDef *tmc5072, uint8 address)
{
	address &= 0x7F;

	// register not readable -> shadow register copy
	if(!TMC_IS_READABLE(tmc5072->registerAccess[address]))
		return tmc5072->shadowRegister[address];

	tmc5072_readWrite(tmc5072->channel, address, FALSE);
	tmc5072_readWrite(tmc5072->channel, 0, FALSE);
	tmc5072_readWrite(tmc5072->channel, 0, FALSE);
	tmc5072_readWrite(tmc5072->channel ,0, FALSE);
	tmc5072_readWrite(tmc5072->channel, 0, TRUE);

	tmc5072_readWrite(tmc5072->channel, address, FALSE);
	int value = tmc5072_readWrite(tmc5072->channel, 0, FALSE);
	value <<= 8;
	value |= tmc5072_readWrite(tmc5072->channel, 0, FALSE);
	value <<= 8;
	value |= tmc5072_readWrite(tmc5072->channel, 0, FALSE);
	value <<= 8;
	value |= tmc5072_readWrite(tmc5072->channel, 0, TRUE);

	return value;
}

void tmc5072_initConfig(TMC5072TypeDef *tmc5072)
{
	tmc5072->velocity[0]      = 0;
	tmc5072->velocity[1]      = 0;
	tmc5072->oldTick          = 0;
	tmc5072->oldX[0]          = 0;
	tmc5072->oldX[1]          = 0;
	tmc5072->vMaxModified[0]  = FALSE;
	tmc5072->vMaxModified[1]  = FALSE;

	int i;
	for(i=0; i < TMC5072_REGISTER_COUNT; i++)
	{
		tmc5072->registerAccess[i]      = tmc5072_defaultRegisterAccess[i];
		tmc5072->registerResetState[i]  = tmc5072_defaultRegisterResetState[i];
	}
}

void tmc5072_writeConfiguration(TMC5072TypeDef *tmc5072, ConfigurationTypeDef *TMC5072_config)
{
	uint8 *ptr = &TMC5072_config->configIndex;
	const int32 *settings = (TMC5072_config->state == CONFIG_RESTORE) ? TMC5072_config->shadowRegister : tmc5072->registerResetState;
	tmc5072->shadowRegister = TMC5072_config->shadowRegister; // TODO API 2: Find better position (LK)

	while((*ptr < TMC5072_REGISTER_COUNT) && !TMC_IS_WRITABLE(tmc5072->registerAccess[*ptr]))
		(*ptr)++;

	if(*ptr < TMC5072_REGISTER_COUNT)
	{
		tmc5072_writeInt(tmc5072, *ptr, settings[*ptr]);
		(*ptr)++;
	}
	else
	{
		TMC5072_config->state = CONFIG_READY;
	}
}

void tmc5072_periodicJob(u8 motor, uint32 tick, TMC5072TypeDef *tmc5072, ConfigurationTypeDef *TMC5072_config)
{
	int xActual;
	uint32 tickDiff;

	if(TMC5072_config->state != CONFIG_READY)
	{
		tmc5072_writeConfiguration(tmc5072, TMC5072_config);
		return;
	}

	if((tickDiff = tick - tmc5072->oldTick) >= 5)
	{

		xActual = tmc5072_readInt(0, TMC5072_XACTUAL(motor));
		TMC5072_config->shadowRegister[TMC5072_XACTUAL(motor)] = xActual;
		tmc5072->velocity[motor] = (int) ((float) (abs(xActual-tmc5072->oldX[motor]) / (float) tickDiff) * (float) 1048.576);
		tmc5072->oldX[motor] = xActual;

		// Not per motor:
		/*xActual = tmc5072_readInt(motor, TMC5072_XACTUAL_1);
		TMC562v3_config->shadowRegister[TMC5072_XACTUAL_1] = xActual;
		TMC562V3.velocityMotor1 = (int) ((float) (abs(xActual-oldX[0]) / (float) t) * (float) 1048.576);
		tmc5072->oldX = xActual;

		xActual = readInt(TMC5072_XACTUAL_2);
		TMC562v3_config->shadowRegister[TMC5072_XACTUAL_2] = xActual;
		TMC562V3.velocityMotor2 = (int) ((float) (abs(xActual-oldX[1]) / (float) t) * (float) 1048.576);
		tmc5072->oldX = xActual;*/

		tmc5072->oldTick = tick;
	}
}

uint8 tmc5072_reset(ConfigurationTypeDef *TMC5072_config)
{
	if(TMC5072_config->state != CONFIG_READY)
		return 0;

	TMC5072_config->state        = CONFIG_RESET;
	TMC5072_config->configIndex  = 0;

	return 1;
}

uint8 tmc5072_restore(ConfigurationTypeDef *TMC5072_config)
{
	if(TMC5072_config->state != CONFIG_READY)
		return 0;

	TMC5072_config->state        = CONFIG_RESTORE;
	TMC5072_config->configIndex  = 0;

	return 1;
}

u32 tmc5072_rotate(TMC5072TypeDef *tmc5072, u8 motor, s32 velocity)
{
	if(motor >= TMC5072_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc5072_writeInt(tmc5072, TMC5072_VMAX(motor), abs(velocity));
	tmc5072->vMaxModified[motor] = TRUE;
	tmc5072_writeDatagram(tmc5072, TMC5072_RAMPMODE(motor), 0, 0, 0, (velocity >= 0)? TMC5072_MODE_VELPOS:TMC5072_MODE_VELNEG);

	return TMC_ERROR_NONE;
}

u32 tmc5072_right(TMC5072TypeDef *tmc5072, u8 motor, s32 velocity)
{
	return tmc5072_rotate(tmc5072, motor, velocity);
}

u32 tmc5072_left(TMC5072TypeDef *tmc5072, u8 motor, s32 velocity)
{
	return tmc5072_rotate(tmc5072, motor, -velocity);
}

u32 tmc5072_stop(TMC5072TypeDef *tmc5072, u8 motor)
{
	return tmc5072_rotate(tmc5072, motor, 0);
}

u32 tmc5072_moveTo(TMC5072TypeDef *tmc5072, u8 motor, s32 position)
{
	if(motor >= TMC5072_MOTORS)
		return TMC_ERROR_MOTOR;

	if(tmc5072->vMaxModified[motor])
	{
		tmc5072_writeInt(tmc5072, TMC5072_VMAX(motor), tmc5072->shadowRegister[TMC5072_VMAX(motor)]);
		tmc5072->vMaxModified[motor] = FALSE;
	}

	tmc5072_writeInt(tmc5072, TMC5072_XTARGET(motor), position);
	tmc5072_writeDatagram(tmc5072, TMC5072_RAMPMODE(motor), 0, 0, 0, TMC5072_MODE_POSITION);

	return TMC_ERROR_NONE;

}

u32 tmc5072_moveBy(TMC5072TypeDef *tmc5072, uint8 motor, int32 *ticks)
{
	// determine actual position and add numbers of ticks to move
	*ticks = tmc5072_readInt(tmc5072, TMC5072_XACTUAL(motor)) + *ticks;

	return tmc5072_moveTo(tmc5072, motor, *ticks);
}
