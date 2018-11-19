/*
 * TMC4671_Constants.h
 *
 *  Created on: 23.07.2018
 *      Author: LK
 */

#ifndef TMC_IC_TMC4671_TMC4671_CONSTANTS_H_
#define TMC_IC_TMC4671_TMC4671_CONSTANTS_H_

// Copied from Constants.h SDE 10-23-18
#define TMC_WRITE_BIT 0x80
#define TMC_ADDRESS_MASK 0x7F

#define TMC_DEFAULT_MOTOR 0

#define TMC_REGISTER_COUNT 128 // Default register count
// end copy

#define TMC4671_MOTORS 1
#define TMC4671_STATUS_FLAGS_REF_MASK   0x00700000
#define TMC4671_STATUS_FLAGS_REF_SHIFT  20

#endif /* TMC_IC_TMC4671_TMC4671_CONSTANTS_H_ */
