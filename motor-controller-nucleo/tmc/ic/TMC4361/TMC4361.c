/*
 * TMC4361.c
 *
 *  Created on: 18.07.2017
 *      Author: LK
 */

#include "TMC4361.h"

// => SPI wrapper
// Send [length] bytes stored in the [data] array over SPI and overwrite [data]
// with the replies. data[0] is the first byte sent and received.
extern void tmc4361_readWriteArray(uint8 channel, uint8 *data, size_t length);
// <= SPI wrapper

// Writes (x1 << 24) | (x2 << 16) | (x3 << 8) | x4 to the given address
void tmc4361_writeDatagram(TMC4361TypeDef *tmc4361, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4)
{
	int value;
	uint8 data[5] = { address | TMC4361_WRITE_BIT, x1, x2, x3, x4 };

	tmc4361_readWriteArray(tmc4361->config->channel, &data[0], 5);

	tmc4361->status = data[0];

	value = (x1 << 24) | (x2 << 16) | (x3 << 8) | x4;

	// Write to the shadow register and mark the register dirty
	address = TMC_ADDRESS(address);
	tmc4361->config->shadowRegister[address] = value;
	tmc4361->registerAccess[address] |= TMC_ACCESS_DIRTY;
}

void tmc4361_writeInt(TMC4361TypeDef *tmc4361, uint8 address, int32 value)
{
	tmc4361_writeDatagram(tmc4361, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

int32 tmc4361_readInt(TMC4361TypeDef *tmc4361, uint8 address)
{
	int value;
	uint8 data[5];

	address = TMC_ADDRESS(address);

	if(!TMC_IS_READABLE(tmc4361->registerAccess[address]))
		return tmc4361->config->shadowRegister[address];

	data[0] = address;
	tmc4361_readWriteArray(tmc4361->config->channel, &data[0], 5);

	data[0] = address;
	tmc4361_readWriteArray(tmc4361->config->channel, &data[0], 5);

	tmc4361->status = data[0];
	value = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];

	return value;
}

// Send [length] bytes stored in the [data] array to a driver attached to the TMC4361
// and overwrite [data] with the replies. data[0] is the first byte sent and received.
void tmc4361_readWriteCover(TMC4361TypeDef *tmc4361, uint8 *data, size_t length)
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
		tmc4361_writeDatagram(tmc4361, TMC4361_COVER_HIGH_WR, bytes[7], bytes[6], bytes[5], bytes[4]);

	tmc4361_writeDatagram(tmc4361, TMC4361_COVER_LOW_WR, bytes[3], bytes[2], bytes[1], bytes[0]);

	// Wait for datagram completion
	// TODO CHECK 3: Get the waiting for cover completion done properly (LH)
	for(i = 0; i < 100; i++)
		tmp = ACCESS_ONCE(i);

	// Read the reply
	if(length > 4)
	{
		tmp = tmc4361_readInt(tmc4361, TMC4361_COVER_DRV_HIGH_RD);
		bytes[4] = BYTE(tmp, 0);
		bytes[5] = BYTE(tmp, 1);
		bytes[6] = BYTE(tmp, 2);
		bytes[7] = BYTE(tmp, 3);
	}
	tmp = tmc4361_readInt(tmc4361, TMC4361_COVER_DRV_LOW_RD);
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
void tmc4361_init(TMC4361TypeDef *tmc4361, uint8 channel, ConfigurationTypeDef *config, const int32 *registerResetState)
{
	tmc4361->velocity  = 0;
	tmc4361->oldTick   = 0;
	tmc4361->oldX      = 0;
	tmc4361->config    = config;

	tmc4361->config->callback     = NULL;
	tmc4361->config->channel      = channel;
	tmc4361->config->configIndex  = 0;
	tmc4361->config->state        = CONFIG_READY;

	int i;
	for(i = 0; i < TMC4361_REGISTER_COUNT; i++)
	{
		tmc4361->registerAccess[i]      = tmc4361_defaultRegisterAccess[i];
		tmc4361->registerResetState[i]  = registerResetState[i];
	}
}

uint8 tmc4361_reset(TMC4361TypeDef *tmc4361)
{
	if(tmc4361->config->state != CONFIG_READY)
		return 0;

	int i;

	// Reset the dirty bits
	for(i = 0; i < TMC4361_REGISTER_COUNT; i++)
		tmc4361->registerAccess[i] &= ~TMC_ACCESS_DIRTY;

	tmc4361->config->state        = CONFIG_RESET;
	tmc4361->config->configIndex  = 0;

	return 1;
}

uint8 tmc4361_restore(TMC4361TypeDef *tmc4361)
{
	if(tmc4361->config->state != CONFIG_READY)
		return 0;

	tmc4361->config->state        = CONFIG_RESTORE;
	tmc4361->config->configIndex  = 0;

	return 1;
}

void tmc4361_setRegisterResetState(TMC4361TypeDef *tmc4361, const int32 *resetState)
{
	uint32 i;
	for(i = 0; i < TMC4361_REGISTER_COUNT; i++)
		tmc4361->registerResetState[i] = resetState[i];
}

void tmc4361_setCallback(TMC4361TypeDef *tmc4361, tmc4361_callback callback)
{
	tmc4361->config->callback = (tmc_callback_config) callback;
}

static void tmc4361_writeConfiguration(TMC4361TypeDef *tmc4361)
{
	uint8 *ptr = &tmc4361->config->configIndex;
	const int32 *settings;

	if(tmc4361->config->state == CONFIG_RESTORE)
	{
		settings = &tmc4361->config->shadowRegister[0];
		// Find the next restorable register
		while((*ptr < TMC4361_REGISTER_COUNT) && !TMC_IS_RESTORABLE(tmc4361->registerAccess[*ptr]))
			(*ptr)++;
	}
	else
	{
		settings = &tmc4361->registerResetState[0];
		// Find the next resettable register
		while((*ptr < TMC4361_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc4361->registerAccess[*ptr]))
			(*ptr)++;
	}

	if (*ptr < TMC4361_REGISTER_COUNT) {
		tmc4361_writeInt(tmc4361, *ptr, settings[*ptr]);
		(*ptr)++;
	}
	else
	{
		if(tmc4361->config->callback)
		{
			((tmc4361_callback)tmc4361->config->callback)(tmc4361, tmc4361->config->state);
		}

		tmc4361->config->state = CONFIG_READY;
	}
}

void tmc4361_periodicJob(TMC4361TypeDef *tmc4361, uint32 tick)
{
	if(tmc4361->config->state != CONFIG_READY)
	{
		tmc4361_writeConfiguration(tmc4361);
		return;
	}

	if((tick - tmc4361->oldTick) != 0)
	{
		tmc4361_calibrateClosedLoop(tmc4361, 0);
		tmc4361->oldTick = tick;
	}
}

void tmc4361_rotate(TMC4361TypeDef *tmc4361, int32 velocity)
{
	// Disable Position Mode
	TMC4361_FIELD_UPDATE(tmc4361, TMC4361_RAMPMODE, TMC4361_OPERATION_MODE_MASK, TMC4361_OPERATION_MODE_SHIFT, 0);

	tmc4361_writeInt(tmc4361, TMC4361_VMAX, tmc4361_discardVelocityDecimals(velocity));
}

void tmc4361_right(TMC4361TypeDef *tmc4361, int32 velocity)
{
	tmc4361_rotate(tmc4361, velocity);
}

void tmc4361_left(TMC4361TypeDef *tmc4361, int32 velocity)
{
	tmc4361_rotate(tmc4361, -velocity);
}

void tmc4361_stop(TMC4361TypeDef *tmc4361)
{
	tmc4361_rotate(tmc4361, 0);
}

void tmc4361_moveTo(TMC4361TypeDef *tmc4361, int32 position, uint32 velocityMax)
{
	// Enable Position Mode
	TMC4361_FIELD_UPDATE(tmc4361, TMC4361_RAMPMODE, TMC4361_OPERATION_MODE_MASK, TMC4361_OPERATION_MODE_SHIFT, 1);

	tmc4361_writeInt(tmc4361, TMC4361_VMAX, tmc4361_discardVelocityDecimals(velocityMax));

	tmc4361_writeInt(tmc4361, TMC4361_X_TARGET, position);
}

// The function will write the absolute target position to *ticks
void tmc4361_moveBy(TMC4361TypeDef *tmc4361, int32 *ticks, uint32 velocityMax)
{
	// determine actual position and add numbers of ticks to move
	*ticks += tmc4361_readInt(tmc4361, TMC4361_XACTUAL);

	tmc4361_moveTo(tmc4361, *ticks, velocityMax);
}

int32 tmc4361_discardVelocityDecimals(int32 value)
{
	if(abs(value) > 8000000)
	{
		value = (value < 0) ? -8000000 : 8000000;
	}
	return value << 8;
}

static uint8 tmc4361_moveToNextFullstep(TMC4361TypeDef *tmc4361)
{
	int32 stepCount;

	// Motor must be stopped
	if(tmc4361_readInt(tmc4361, TMC4361_VACTUAL) != 0)
	{
		// Not stopped
		return 0;
	}

	// Position mode, hold mode, low velocity
	tmc4361_writeInt(tmc4361, TMC4361_RAMPMODE, 4);
	tmc4361_writeInt(tmc4361, TMC4361_VMAX, 10000 << 8);

	// Current step count
	stepCount = TMC4361_FIELD_READ(tmc4361, TMC4361_MSCNT_RD, TMC4361_MSCNT_MASK, TMC4361_MSCNT_SHIFT);
	// Get microstep value of step count (lowest 8 bits)
	stepCount = stepCount % 256;
	// Assume: 256 microsteps -> Fullsteps are at 128 + n*256
	stepCount = 128 - stepCount;

	if(stepCount == 0)
	{
		// Fullstep reached
		return 1;
	}

	// Fullstep not reached -> calculate next fullstep position
	stepCount += tmc4361_readInt(tmc4361, TMC4361_XACTUAL);
	// Move to next fullstep position
	tmc4361_writeInt(tmc4361, TMC4361_X_TARGET, stepCount);

	return 0;
}

uint8 tmc4361_calibrateClosedLoop(TMC4361TypeDef *tmc4361, uint8 worker0master1)
{
	static uint8 state = 0;
	static uint32 oldRamp;

	uint32 amax = 0;
	uint32 dmax = 0;

	if(worker0master1 && state == 0)
		state = 1;

	switch(state)
	{
	case 1:
		amax = tmc4361_readInt(tmc4361, TMC4361_AMAX);
		dmax = tmc4361_readInt(tmc4361, TMC4361_DMAX);

		// Set ramp and motion parameters
		oldRamp = tmc4361_readInt(tmc4361, TMC4361_RAMPMODE);
		tmc4361_writeInt(tmc4361, TMC4361_RAMPMODE, TMC4361_RAMP_POSITION | TMC4361_RAMP_HOLD);
		tmc4361_writeInt(tmc4361, TMC4361_AMAX, MAX(amax, 1000));
		tmc4361_writeInt(tmc4361, TMC4361_DMAX, MAX(dmax, 1000));
		tmc4361_writeInt(tmc4361, TMC4361_VMAX, 0);

		state = 2;
		break;
	case 2:
		// Clear encoder calibration bit
		TMC4361_FIELD_UPDATE(tmc4361, TMC4361_ENC_IN_CONF, TMC4361_CL_CALIBRATION_EN_MASK, TMC4361_CL_CALIBRATION_EN_SHIFT, 0);

		// Disable internal data regulation for closed loop operation in encoder config
		TMC4361_FIELD_UPDATE(tmc4361, TMC4361_ENC_IN_CONF, TMC4361_REGULATION_MODUS_MASK, TMC4361_REGULATION_MODUS_SHIFT, 1);

		if(tmc4361_moveToNextFullstep(tmc4361)) // move to next fullstep, motor must be stopped, poll until finished
			state = 3;
		break;
	case 3:
		// Start encoder calibration
		TMC4361_FIELD_UPDATE(tmc4361, TMC4361_ENC_IN_CONF, TMC4361_CL_CALIBRATION_EN_MASK, TMC4361_CL_CALIBRATION_EN_SHIFT, 1);

		state = 4;
		break;
	case 4:
		if(worker0master1)
			break;

		// Stop encoder calibration
		TMC4361_FIELD_UPDATE(tmc4361, TMC4361_ENC_IN_CONF, TMC4361_CL_CALIBRATION_EN_MASK, TMC4361_CL_CALIBRATION_EN_SHIFT, 0);
		// Enable closed loop in encoder config
		TMC4361_FIELD_UPDATE(tmc4361, TMC4361_ENC_IN_CONF, TMC4361_REGULATION_MODUS_MASK, TMC4361_REGULATION_MODUS_SHIFT, 1);
		// Restore old ramp mode, enable position mode
		tmc4361_writeInt(tmc4361, TMC4361_RAMPMODE, TMC4361_RAMP_POSITION | oldRamp);

		state = 5;
		break;
	case 5:
		state = 0;
		return 1;
		break;
	default:
		break;
	}
	return 0;
}
