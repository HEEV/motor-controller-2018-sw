/*
 * TMC2224.c
 *
 *  Created on: 07.07.2017
 *      Author: LK
 */

#include "TMC2224.h"

// Default Register values
#define R00 0x00000141  // GCONF
#define R10 0x00001F00  // IHOLD_IRUN
#define R11 0x00000014  // TPOWERDOWN
#define R6C 0x10000053  // CHOPCONF
#define R70 0xC10D0024  // PWMCONF

/* Register access permissions:
 * 0: none (reserved)
 * 1: read
 * 2: write
 * 3: read/write
 * 7: read^write (seperate functions/values)
 */
const u8 tmc2224_defaultRegisterAccess[TMC2224_REGISTER_COUNT] =
{
//  0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	3, 3, 1, 2, 2, 1, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, // 0x00 - 0x0F
	2, 2, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x10 - 0x1F
	0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x20 - 0x2F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x30 - 0x3F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x40 - 0x4F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x50 - 0x5F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 3, 0, 0, 1, // 0x60 - 0x6F
	3, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  // 0x70 - 0x7F
};

const s32 tmc2224_defaultRegisterResetState[TMC2224_REGISTER_COUNT] = {
//	0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	R00, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, R11, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  // 0x70 - 0x7F
};

// => SPI wrapper
extern void tmc2224_writeRegister(uint8 motor, uint8 address, int32 value);
extern void tmc2224_readRegister(uint8 motor, uint8 address, int32 *value);
// <= SPI wrapper

void tmc2224_initConfig(TMC2224TypeDef *tmc2224)
{
	tmc2224->velocity      = 0;
	tmc2224->oldTick       = 0;
	tmc2224->oldX          = 0;
	tmc2224->vMaxModified  = FALSE;

	int i;
	for(i = 0; i < TMC2224_REGISTER_COUNT; i++)
	{
		tmc2224->registerAccess[i]      = tmc2224_defaultRegisterAccess[i];
		tmc2224->registerResetState[i]  = tmc2224_defaultRegisterResetState[i];
	}
}

void tmc2224_writeConfiguration(TMC2224TypeDef *tmc2224, ConfigurationTypeDef *TMC2224_config)
{
	uint8 *ptr = &TMC2224_config->configIndex;
	const int32 *settings = (TMC2224_config->state == CONFIG_RESTORE) ? TMC2224_config->shadowRegister : tmc2224->registerResetState;

	while((*ptr < TMC2224_REGISTER_COUNT) && !TMC_IS_WRITABLE(tmc2224->registerAccess[*ptr]))
		(*ptr)++;

	if(*ptr < TMC2224_REGISTER_COUNT)
	{
		tmc2224_writeRegister(0, *ptr, settings[*ptr]);
		(*ptr)++;
	}
	else
	{
		TMC2224_config->state = CONFIG_READY;
	}
}

void tmc2224_periodicJob(u8 motor, uint32 tick, TMC2224TypeDef *tmc2224, ConfigurationTypeDef *TMC2224_config)
{
	UNUSED(motor);

	if(TMC2224_config->state != CONFIG_READY && (tick - tmc2224->oldTick) > 2)
	{
		tmc2224_writeConfiguration(tmc2224, TMC2224_config);
		tmc2224->oldTick = tick;
	}
}

uint8 tmc2224_reset(ConfigurationTypeDef *TMC2224_config)
{
	if(TMC2224_config->state != CONFIG_READY)
		return 0;

	TMC2224_config->state        = CONFIG_RESET;
	TMC2224_config->configIndex  = 0;

	return 1;
}

uint8 tmc2224_restore(ConfigurationTypeDef *TMC2224_config)
{
	if(TMC2224_config->state != CONFIG_READY)
		return 0;

	TMC2224_config->state        = CONFIG_RESTORE;
	TMC2224_config->configIndex  = 0;

	return 1;
}
