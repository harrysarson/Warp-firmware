#include "warp.h"
#include "fsl_i2c_hal.h"
#include "fsl_i2c_master_driver.h"
#include "gpio_pins.h"
#include "devINA219.h"

#include <stdint.h>

extern volatile WarpI2CDeviceState deviceAS7262State;
extern volatile uint32_t gWarpI2cBaudRateKbps;

#define READ_BYTES 2
#define CMD_BYTES 1
#define CALIBRATE_BYTES 3

WarpStatus read_current(unsigned *milli_amps)
{
	uint8_t cmdBuf[CMD_BYTES] = {INA219_REG_CURRENT};
	uint8_t calibrateBuf[CALIBRATE_BYTES] = {INA219_REG_CALIBRATION, 0x10, 0x00};
	uint8_t read_result[READ_BYTES];
	i2c_status_t returnValue;

	i2c_device_t slave = {
	    .address = INA219_ADDRESS,
	    .baudRate_kbps = gWarpI2cBaudRateKbps,
	};

	// TODO: calibrate?

	// returnValue = I2C_DRV_MasterSendDataBlocking(
	//     0,
	//     &slave /* The pointer to the I2C device information structure */,
	//     calibrateBuf /* The pointer to the commands to be transferred */,
	//     CALIBRATE_BYTES /* The length in bytes of the commands to be transferred */,
	//     NULL /* The pointer to the data to be transferred */,
	//     0 /* The length in bytes of the data to be transferred */,
	//     500 /* timeout in milliseconds */);

	// if (returnValue != kStatus_I2C_Success)
	// {
	// 	return kWarpStatusDeviceCommunicationFailed;
	// }

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
	    0 /* I2C peripheral instance */,
	    &slave,
	    cmdBuf,
	    CMD_BYTES,
	    read_result,
	    READ_BYTES,
	    500 /* timeout in milliseconds */);

	if (milli_amps != NULL)
	{
		// 12 bit resolution, 0xFFF = 320mV
		// Resistor is 0.1 ohm
		unsigned shunt_voltage_drop = (read_result[0] << 8) | read_result[1];
		assert(shunt_voltage_drop & ~0xFFFu == 0);
		*milli_amps = (shunt_voltage_drop * 100 * 320) / 0xFFF;
	}

	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}
