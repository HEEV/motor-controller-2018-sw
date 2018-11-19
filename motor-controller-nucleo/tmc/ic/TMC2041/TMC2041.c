/*
 * TMC2041.c
 *
 *  Created on: 14.08.2017
 *      Author: LK
 */

#include "TMC2041.h"

// Default Register values
#define R00 0x00000006  // GCONF

#define R30 0x00071703  // IHOLD_IRUN (Motor 1)
#define R50 0x00071703  // IHOLD_IRUN (Motor 2)

#define R6C 0x000101D5  // CHOPCONF   (Motor 1)
#define R7C 0x000101D5  // CHOPCONF   (Motor 2)


/* Register access permissions:
 * 0x00: none (reserved)
 * 0x01: read
 * 0x02: write
 * 0x03: read/write
 * 0x07: read^write (seperate functions/values)
 */
const u8 tmc2041_defaultRegisterAccess[TMC2041_REGISTER_COUNT] = {
//  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x03, 0x01, 0x01, 0x02, 0x07, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x00 - 0x0F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x10 - 0x1F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x20 - 0x2F
	0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x30 - 0x3F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x40 - 0x4F
	0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x50 - 0x5F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, 0x01, 0x01, 0x03, 0x02, ____, 0x01, // 0x60 - 0x6F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, 0x01, 0x01, 0x03, 0x02, ____, 0x01  // 0x70 - 0x7F
};

const s32 tmc2041_defaultRegisterResetState[TMC2041_REGISTER_COUNT] = {
//  0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	R00, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	R30, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	R50, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R7C, 0,   0,   0  // 0x70 - 0x7F
};

// => SPI wrapper
extern void tmc2041_writeDatagram(u8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
extern void tmc2041_writeInt(u8 motor, uint8 address, int value);
extern int tmc2041_readInt(u8 motor, uint8 address);
// <= SPI wrapper

void tmc2041_initConfig(TMC2041TypeDef *tmc2041)
{
	int i;
	for(i = 0; i < TMC2041_REGISTER_COUNT; i++)
	{
		tmc2041->registerAccess[i]      = tmc2041_defaultRegisterAccess[i];
		tmc2041->registerResetState[i]  = tmc2041_defaultRegisterResetState[i];
	}
}

void tmc2041_writeConfiguration(TMC2041TypeDef *tmc2041, ConfigurationTypeDef *TMC2041_config)
{
	uint8 *ptr = &TMC2041_config->configIndex;
	const int32 *settings = (TMC2041_config->state == CONFIG_RESTORE) ? TMC2041_config->shadowRegister : tmc2041->registerResetState;

	while((*ptr < TMC2041_REGISTER_COUNT) && !TMC_IS_WRITABLE(tmc2041->registerAccess[*ptr]))
		(*ptr)++;

	if(*ptr < TMC2041_REGISTER_COUNT)
	{
		tmc2041_writeInt(0, *ptr, settings[*ptr]);
		(*ptr)++;
	}
	else
	{
		TMC2041_config->state = CONFIG_READY;
	}
}

void tmc2041_periodicJob(uint8 motor, uint32 tick, TMC2041TypeDef *tmc2041, ConfigurationTypeDef *TMC2041_config)
{
	UNUSED(motor);
	UNUSED(tick);

	if(TMC2041_config->state != CONFIG_READY)
	{
		tmc2041_writeConfiguration(tmc2041, TMC2041_config);
		return;
	}
}

uint8 tmc2041_reset(ConfigurationTypeDef *TMC2041_config)
{
	if(TMC2041_config->state != CONFIG_READY)
		return 0;

	TMC2041_config->state        = CONFIG_RESET;
	TMC2041_config->configIndex  = 0;

	return 1;
}

uint8 tmc2041_restore(ConfigurationTypeDef *TMC2041_config)
{
	if(TMC2041_config->state != CONFIG_READY)
		return 0;

	TMC2041_config->state        = CONFIG_RESTORE;
	TMC2041_config->configIndex  = 0;

	return 1;
}
