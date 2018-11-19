/*
 * TMC262_1420.c
 *
 *  Created on: 11.07.2017
 *      Author: LK
 */

#include "../TMC262_1420/TMC262_1420.h"

const u8 tmc262_1420_defaultRegisterAccess[TMC262_1420_REGISTER_COUNT] =
{
	TMC_ACCESS_WRITE,  // 0: DRVCTRL
	TMC_ACCESS_NONE,   // 1: UNUSED
	TMC_ACCESS_NONE,   // 2: UNUSED
	TMC_ACCESS_NONE,   // 3: UNUSED
	TMC_ACCESS_WRITE,  // 4: CHOPCONF
	TMC_ACCESS_WRITE,  // 5: SMARTEN
	TMC_ACCESS_WRITE,  // 6: SGCSCONF
	TMC_ACCESS_WRITE   // 7: DRVCONF
};

const s32 tmc262_1420_defaultRegisterResetState[TMC262_1420_REGISTER_COUNT] =
{
	0x00000000,  // 0: DRVCTRL
	0x00000000,  // 1: UNUSED
	0x00000000,  // 2: UNUSED
	0x00000000,  // 3: UNUSED
	0x00091935,  // 4: CHOPCONF
	0x000A0000,  // 5: SMARTEN
	0x000D0505,  // 6: SGCSCONF
	0x000EF040   // 7: DRVCONF
};

// => SPI wrapper
extern void tmc262_1420_writeInt(uint8 motor, uint8 address, int value);
extern uint32 tmc262_1420_readInt(uint8 motor, uint8 address);
extern void tmc262_1420_readWrite(uint8 motor, uint32 value);
//extern void tmc262_1420_setField(uint8 motor, uint8 address, uint32 clearMask, uint32 field);
// <= SPI wrapper

static void standStillCurrentLimitation(TMC262_1420TypeDef *TMC262_1420)
{ // mark if current should be reduced in stand still if too high
	static uint32 errorTimer = 0;

	// check the standstill flag
	if(TMC262_1420_GET_STST(tmc262_1420_readInt(0, TMC262_1420_RESPONSE_LATEST)))
	{
		// check if current reduction is neccessary
		if(TMC262_1420->runCurrentScale > TMC262_1420->standStillCurrentScale)
		{
			TMC262_1420->isStandStillOverCurrent = 1;

			// count timeout
			if(errorTimer++ > TMC262_1420->standStillTimeout/10)
			{
				// set current limitation flag
				TMC262_1420->isStandStillCurrentLimit = 1;
				errorTimer = 0;
			}
			return;
		}
	}

	// No standstill or overcurrent -> reset flags & error timer
	TMC262_1420->isStandStillOverCurrent  = 0;
	TMC262_1420->isStandStillCurrentLimit = 0;
	errorTimer = 0;
}

static void continousSync(ConfigurationTypeDef *TMC262_1420_config)
{ // refreshes settings to prevent chip from loosing settings on brownout
	static uint8 write  = 0;
	static uint8 read   = 0;
	static uint8 rdsel  = 0;

	// rotational reading all replys to keep values up to date
	uint32 value, drvConf;

	// additional reading to keep all replies up to date
	value = drvConf = tmc262_1420_readInt(0, TMC262_1420_WRITE_BIT | TMC262_1420_DRVCONF);  // buffer value amd  drvConf to write back later
	value &= ~TMC262_1420_SET_RDSEL(-1);                                        // clear RDSEL bits
	value |= TMC262_1420_SET_RDSEL(rdsel % 3);                                  // clear set rdsel
	tmc262_1420_readWrite(0, value);
	tmc262_1420_readWrite(0, drvConf);

	// determine next read address
	read = (read + 1) % 3;

	// write settings from shadow register to chip.
	//readWrite(TMC262_1420_config->shadowRegister[TMC262_1420_WRITE_BIT | write]);
	tmc262_1420_readWrite(0, TMC262_1420_config->shadowRegister[TMC262_1420_WRITE_BIT | write]);

	// determine next write address - skip unused addresses
	write = (write == TMC262_1420_DRVCTRL) ? TMC262_1420_CHOPCONF : ((write + 1) % TMC262_1420_REGISTER_COUNT);
}

void tmc262_1420_initConfig(TMC262_1420TypeDef *tmc262_1420)
{
	tmc262_1420->velocity                  = 0;
	tmc262_1420->oldTick                   = 0;
	tmc262_1420->oldX                      = 0;
	tmc262_1420->continuousModeEnable      = 0;
	tmc262_1420->isStandStillCurrentLimit  = 0;
	tmc262_1420->isStandStillOverCurrent   = 0;
	tmc262_1420->runCurrentScale           = 0;
	tmc262_1420->coolStepActiveValue       = 0;
	tmc262_1420->coolStepInactiveValue     = 0;
	tmc262_1420->coolStepThreshold         = 0;
	tmc262_1420->standStillCurrentScale    = 0;
	tmc262_1420->standStillTimeout         = 0;

	int i;
	for(i = 0; i < TMC262_1420_REGISTER_COUNT; i++)
	{
		tmc262_1420->registerAccess[i]      = tmc262_1420_defaultRegisterAccess[i];
		tmc262_1420->registerResetState[i]  = tmc262_1420_defaultRegisterResetState[i];
	}
}

// Currently unused, we write the whole configuration as part of the reset/restore functions
void tmc262_1420_writeConfiguration(TMC262_1420TypeDef *tmc262_1420, ConfigurationTypeDef *TMC262_1420_config)
{
	// write one writeable register at a time - backwards to hit DRVCONF before DRVCTRL
	UNUSED(tmc262_1420);
	UNUSED(TMC262_1420_config);

	//uint8 *ptr = &TMC262_1420_config->configIndex;
	//const int32 *settings = (TMC262_1420_config->state == CONFIG_RESTORE) ? TMC262_1420_config->shadowRegister : tmc262_1420->registerResetState;

	//while((*ptr >= 0) && !IS_WRITEABLE(tmc262_1420->registerAccess[*ptr]))
		//(*ptr)--;

	//if(*ptr >= 0)
	//{
		//tmc262_1420_writeInt(0, *ptr, settings[*ptr]);
		//(*ptr)--;
	//}
	//else
	//{
		//TMC262_1420_config->state = CONFIG_READY;
	//}
}

void tmc262_1420_periodicJob(u8 motor, uint32 tick, TMC262_1420TypeDef *tmc262_1420, ConfigurationTypeDef *TMC262_1420_config)
{
	UNUSED(motor);

	if(tmc262_1420->continuousModeEnable)
	{ // continuously write settings to chip and rotate through all reply types to keep data up to date
		continousSync(TMC262_1420_config);
		if(tick - tmc262_1420->oldTick >= 10)
		{
			standStillCurrentLimitation(tmc262_1420);
			tmc262_1420->oldTick = tick;
		}
	}
}

uint8 tmc262_1420_reset(TMC262_1420TypeDef *TMC262_1420, ConfigurationTypeDef *TMC262_1420_config)
{
	UNUSED(TMC262_1420_config);

	tmc262_1420_writeInt(0, TMC262_1420_DRVCONF,  TMC262_1420->registerResetState[TMC262_1420_DRVCONF]);
	tmc262_1420_writeInt(0, TMC262_1420_DRVCTRL,  TMC262_1420->registerResetState[TMC262_1420_DRVCTRL]);
	tmc262_1420_writeInt(0, TMC262_1420_CHOPCONF, TMC262_1420->registerResetState[TMC262_1420_CHOPCONF]);
	tmc262_1420_writeInt(0, TMC262_1420_SMARTEN,  TMC262_1420->registerResetState[TMC262_1420_SMARTEN]);
	tmc262_1420_writeInt(0, TMC262_1420_SGCSCONF, TMC262_1420->registerResetState[TMC262_1420_SGCSCONF]);

	return 1;
}

uint8 tmc262_1420_restore(ConfigurationTypeDef *TMC262_1420_config)
{
	tmc262_1420_writeInt(0, TMC262_1420_DRVCONF,  TMC262_1420_config->shadowRegister[TMC262_1420_DRVCONF  | TMC262_1420_WRITE_BIT]);
	tmc262_1420_writeInt(0, TMC262_1420_DRVCTRL,  TMC262_1420_config->shadowRegister[TMC262_1420_DRVCTRL  | TMC262_1420_WRITE_BIT]);
	tmc262_1420_writeInt(0, TMC262_1420_CHOPCONF, TMC262_1420_config->shadowRegister[TMC262_1420_CHOPCONF | TMC262_1420_WRITE_BIT]);
	tmc262_1420_writeInt(0, TMC262_1420_SMARTEN,  TMC262_1420_config->shadowRegister[TMC262_1420_SMARTEN  | TMC262_1420_WRITE_BIT]);
	tmc262_1420_writeInt(0, TMC262_1420_SGCSCONF, TMC262_1420_config->shadowRegister[TMC262_1420_SGCSCONF | TMC262_1420_WRITE_BIT]);

	return 1;
}
