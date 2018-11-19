/*
 * TMC4331.c
 *
 *  Created on: 18.07.2017
 *      Author: LK
 */

#include "TMC4331.h"

// => SPI wrapper
// Send [length] bytes stored in the [data] array over SPI and overwrite [data]
// with the replies. data[0] is the first byte sent and received.
extern void tmc4331_readWriteArray(uint8 channel, uint8 *data, size_t length);
// <= SPI wrapper

// Writes (x1 << 24) | (x2 << 16) | (x3 << 8) | x4 to the given address
void tmc4331_writeDatagram(TMC4331TypeDef *tmc4331, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4)
{
	int value;
	uint8 data[5] = { address | TMC4331_WRITE_BIT, x1, x2, x3, x4 };

	tmc4331_readWriteArray(tmc4331->config->channel, &data[0], 5);

	tmc4331->status = data[0];

	value = (x1 << 24) | (x2 << 16) | (x3 << 8) | x4;

	// Write to the shadow register and mark the register dirty
	address = TMC_ADDRESS(address);
	tmc4331->config->shadowRegister[address] = value;
	tmc4331->registerAccess[address] |= TMC_ACCESS_DIRTY;
}

void tmc4331_writeInt(TMC4331TypeDef *tmc4331, uint8 address, int32 value)
{
	tmc4331_writeDatagram(tmc4331, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

int32 tmc4331_readInt(TMC4331TypeDef *tmc4331, uint8 address)
{
	int value;
	uint8 data[5];

	address = TMC_ADDRESS(address);

	if(!TMC_IS_READABLE(tmc4331->registerAccess[address]))
		return tmc4331->config->shadowRegister[address];

	data[0] = address;
	tmc4331_readWriteArray(tmc4331->config->channel, &data[0], 5);

	data[0] = address;
	tmc4331_readWriteArray(tmc4331->config->channel, &data[0], 5);

	tmc4331->status = data[0];
	value = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];

	return value;
}

// Send [length] bytes stored in the [data] array to a driver attached to the TMC4331
// and overwrite [data] with the replies. data[0] is the first byte sent and received.
void tmc4331_readWriteCover(TMC4331TypeDef *tmc4331, uint8 *data, size_t length)
{
	// Check if datagram length is valid
	if(length == 0 || length > 8)
		return;

	uint8 bytes[8] = { 0 };
	uint32 tmp;
	size_t i;

	// Copy data into buffer of maximum cover datagram length (8 bytes)
	for(i = 0; i < length; i++)
		bytes[i] = data[length-i-1];

	// Send the datagram
	if(length > 4)
		tmc4331_writeDatagram(tmc4331, TMC4331_COVER_HIGH_WR, bytes[7], bytes[6], bytes[5], bytes[4]);

	tmc4331_writeDatagram(tmc4331, TMC4331_COVER_LOW_WR, bytes[3], bytes[2], bytes[1], bytes[0]);

	// Wait for datagram completion
	// TODO CHECK 3: Get the waiting for cover completion done properly (LH)
	for(i = 0; i < 100; i++)
		tmp = ACCESS_ONCE(i);

	// Read the reply
	if(length > 4)
	{
		tmp = tmc4331_readInt(tmc4331, TMC4331_COVER_DRV_HIGH_RD);
		bytes[4] = BYTE(tmp, 0);
		bytes[5] = BYTE(tmp, 1);
		bytes[6] = BYTE(tmp, 2);
		bytes[7] = BYTE(tmp, 3);
	}
	tmp = tmc4331_readInt(tmc4331, TMC4331_COVER_DRV_LOW_RD);
	bytes[0] = BYTE(tmp, 0);
	bytes[1] = BYTE(tmp, 1);
	bytes[2] = BYTE(tmp, 2);
	bytes[3] = BYTE(tmp, 3);

	// Write the reply to the data array
	for(i = 0; i < length; i++)
	{
		data[length-i-1] = bytes[i];
	}
}

// Provide the init function with a channel index (sent back in the SPI callback), a pointer to a ConfigurationTypeDef struct
// and a pointer to a int32 array (size 128) holding the reset values that shall be used.
void tmc4331_init(TMC4331TypeDef *tmc4331, uint8 channel, ConfigurationTypeDef *config, const int32 *registerResetState)
{
	tmc4331->velocity  = 0;
	tmc4331->oldTick   = 0;
	tmc4331->oldX      = 0;
	tmc4331->config    = config;

	tmc4331->config->callback     = NULL;
	tmc4331->config->channel      = channel;
	tmc4331->config->configIndex  = 0;
	tmc4331->config->state        = CONFIG_READY;

	int i;
	for(i = 0; i < TMC4331_REGISTER_COUNT; i++)
	{
		tmc4331->registerAccess[i]      = tmc4331_defaultRegisterAccess[i];
		tmc4331->registerResetState[i]  = registerResetState[i];
	}
}

uint8 tmc4331_reset(TMC4331TypeDef *tmc4331)
{
	if(tmc4331->config->state != CONFIG_READY)
		return 0;

	int i;

	// Reset the dirty bits
	for(i = 0; i < TMC4331_REGISTER_COUNT; i++)
		tmc4331->registerAccess[i] &= ~TMC_ACCESS_DIRTY;

	tmc4331->config->state        = CONFIG_RESET;
	tmc4331->config->configIndex  = 0;

	return 1;
}

uint8 tmc4331_restore(TMC4331TypeDef *tmc4331)
{
	if(tmc4331->config->state != CONFIG_READY)
		return 0;

	tmc4331->config->state        = CONFIG_RESTORE;
	tmc4331->config->configIndex  = 0;

	return 1;
}

void tmc4331_setRegisterResetState(TMC4331TypeDef *tmc4331, const int32 *resetState)
{
	uint32 i;
	for(i = 0; i < TMC4331_REGISTER_COUNT; i++)
		tmc4331->registerResetState[i] = resetState[i];
}

void tmc4331_setCallback(TMC4331TypeDef *tmc4331, tmc4331_callback callback)
{
	tmc4331->config->callback = (tmc_callback_config) callback;
}

static void tmc4331_writeConfiguration(TMC4331TypeDef *tmc4331)
{
	uint8 *ptr = &tmc4331->config->configIndex;
	const int32 *settings;

	if(tmc4331->config->state == CONFIG_RESTORE)
	{
		settings = &tmc4331->config->shadowRegister[0];
		// Find the next restorable register
		while((*ptr < TMC4331_REGISTER_COUNT) && !TMC_IS_RESTORABLE(tmc4331->registerAccess[*ptr]))
			(*ptr)++;
	}
	else
	{
		settings = &tmc4331->registerResetState[0];
		// Find the next resettable register
		while((*ptr < TMC4331_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc4331->registerAccess[*ptr]))
			(*ptr)++;
	}

	if (*ptr < TMC4331_REGISTER_COUNT) {
		tmc4331_writeInt(tmc4331, *ptr, settings[*ptr]);
		(*ptr)++;
	}
	else
	{
		if(tmc4331->config->callback)
		{
			((tmc4331_callback)tmc4331->config->callback)(tmc4331, tmc4331->config->state);
		}

		tmc4331->config->state = CONFIG_READY;
	}
}

void tmc4331_periodicJob(TMC4331TypeDef *tmc4331, uint32 tick)
{
	if(tmc4331->config->state != CONFIG_READY)
	{
		tmc4331_writeConfiguration(tmc4331);
		return;
	}

	if((tick - tmc4331->oldTick) != 0)
	{
		tmc4331->oldTick = tick;
	}
}

void tmc4331_rotate(TMC4331TypeDef *tmc4331, int32 velocity)
{
	// Disable Position Mode
	TMC4331_FIELD_UPDATE(tmc4331, TMC4331_RAMPMODE, TMC4331_OPERATION_MODE_MASK, TMC4331_OPERATION_MODE_SHIFT, 0);

	tmc4331_writeInt(tmc4331, TMC4331_VMAX, tmc4331_discardVelocityDecimals(velocity));
}

void tmc4331_right(TMC4331TypeDef *tmc4331, int32 velocity)
{
	tmc4331_rotate(tmc4331, velocity);
}

void tmc4331_left(TMC4331TypeDef *tmc4331, int32 velocity)
{
	tmc4331_rotate(tmc4331, -velocity);
}

void tmc4331_stop(TMC4331TypeDef *tmc4331)
{
	tmc4331_rotate(tmc4331, 0);
}

void tmc4331_moveTo(TMC4331TypeDef *tmc4331, int32 position, uint32 velocityMax)
{
	// Enable Position Mode
	TMC4331_FIELD_UPDATE(tmc4331, TMC4331_RAMPMODE, TMC4331_OPERATION_MODE_MASK, TMC4331_OPERATION_MODE_SHIFT, 1);

	tmc4331_writeInt(tmc4331, TMC4331_VMAX, tmc4331_discardVelocityDecimals(velocityMax));

	tmc4331_writeInt(tmc4331, TMC4331_X_TARGET, position);
}

// The function will write the absolute target position to *ticks
void tmc4331_moveBy(TMC4331TypeDef *tmc4331, int32 *ticks, uint32 velocityMax)
{
	// determine actual position and add numbers of ticks to move
	*ticks += tmc4331_readInt(tmc4331, TMC4331_XACTUAL);

	tmc4331_moveTo(tmc4331, *ticks, velocityMax);
}

int32 tmc4331_discardVelocityDecimals(int32 value)
{
	if(abs(value) > 8000000)
	{
		value = (value < 0) ? -8000000 : 8000000;
	}
	return value << 8;
}
