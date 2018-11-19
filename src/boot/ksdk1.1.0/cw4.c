/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devSSD1331.h"
#include "devINA219.h"

#define kWarpConstantStringI2cFailure "\rI2C failed, reg 0x%02x, code %d\n"
#define kWarpConstantStringErrorInvalidVoltage "\rInvalid supply voltage [%d] mV!"
#define kWarpConstantStringErrorSanity "\rSanity Check Failed!"

#define NUMBER_OF_COLOURS 4

volatile WarpSPIDeviceState deviceINA219tate;

/*
 *	TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t i2cMasterState;
volatile spi_master_state_t spiMasterState;
volatile spi_master_user_config_t spiUserConfig;

/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t gWarpI2cBaudRateKbps = 1;
volatile uint32_t gWarpUartBaudRateKbps = 1;
volatile uint32_t gWarpSpiBaudRateKbps = 1;
volatile uint32_t gWarpSleeptimeSeconds = 0;
volatile WarpModeMask gWarpMode = kWarpModeDisableAdcOnSleep;

void sleepUntilReset(void);
void lowPowerPinStates(void);
void disableTPS82740A(void);
void disableTPS82740B(void);
void enableTPS82740A(uint16_t voltageMillivolts);
void enableTPS82740B(uint16_t voltageMillivolts);
void setTPS82740CommonControlLines(uint16_t voltageMillivolts);
void printPinDirections(void);
void dumpProcessorState(void);
void repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
					   uint16_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty,
					   int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
					   uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);

void warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);

/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *notify, void *callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t clockManagerCallbackUserlevelStructure =
    {
	.callback = clockManagerCallbackRoutine,
	.callbackType = kClockManagerCallbackBeforeAfter,
	.callbackData = NULL};

static clock_manager_callback_user_config_t *clockCallbackTable[] =
    {
	&clockManagerCallbackUserlevelStructure};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *notify, void *callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
	case kClockManagerNotifyBefore:
		break;
	case kClockManagerNotifyRecover:
	case kClockManagerNotifyAfter:
		break;
	default:
		result = kClockManagerError;
		break;
	}

	return result;
}

/*
 *	Override the RTC IRQ handler
 */
void RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *notify,
				     power_manager_callback_data_t *dataPtr)
{
	WarpPowerManagerCallbackStructure *callbackUserData = (WarpPowerManagerCallbackStructure *)dataPtr;
	power_manager_error_code_t status = kPowerManagerError;

	switch (notify->notifyType)
	{
	case kPowerManagerNotifyBefore:
		status = kPowerManagerSuccess;
		break;
	case kPowerManagerNotifyAfter:
		status = kPowerManagerSuccess;
		break;
	default:
		callbackUserData->errorCount++;
		break;
	}

	return status;
}

/*
 *	From KSDK power_manager_demo.c <<END>>>
 */

void sleepUntilReset(void)
{
	while (1)
	{
		GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);
		GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}

void enableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	/*	Warp KL03_SPI_SCK	--> PTB0	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 *
	 */
	uint32_t calculatedBaudRate;
	spiUserConfig.polarity = kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase = kSpiClockPhase_FirstEdge;
	spiUserConfig.direction = kSpiMsbFirst;
	spiUserConfig.bitsPerSec = gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void disableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	/*	Warp KL03_SPI_SCK	--> PTB0	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}

void enableI2Cpins(uint16_t pullupValue)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);

	/*
	 *	TODO: need to implement config of the DCP
	 */
	//...
}

void disableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);

	/*
	 *	TODO: need to implement clearing of the DCP
	 */
	//...

	/*
	 *	Drive the I2C pins low
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);

	CLOCK_SYS_DisableI2cClock(0);
}

void lowPowerPinStates(void)
{
	/*
	 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
	 *	we configure all pins as output and set them to a known state. We choose
	 *	to set them all to '0' since it happens that the devices we want to keep
	 *	deactivated (SI4705, PAN1326) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	/*
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
	*/

	/*
	 *	PTA3 and PTA4 are the EXTAL/XTAL
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);

	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	/*
	 *	PTB1 is connected to KL03_VDD. We have a choice of:
	 *		(1) Keep 'disabled as analog'.
	 *		(2) Set as output and drive high.
	 *
	 *	Pin state "disabled" means default functionality (ADC) is _active_
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	}
	else
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	}

	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

	/*
	 *	PTB3 and PTB3 (I2C pins) are true open-drain
	 *	and we purposefully leave them disabled.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB8 or PTB9
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB12
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);

	/*
	 *	Now, set all the pins (except kWarpPinKL03_VDD_ADC, the SWD pins, and the XTAL/EXTAL) to 0
	 */

	/*
	 *	If we are in mode where we disable the ADC, then drive the pin high since it is tied to KL03_VDD
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		GPIO_DRV_SetPinOutput(kWarpPinKL03_VDD_ADC);
	}

#ifdef WARP_FRDMKL03
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1323_nSHUTD);
#else
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);
	GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

	/*
	 *	Drive these chip selects high since they are active low:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);
	GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);

	/*
	 *	When the PAN1326 is installed, note that it has the
	 *	following pull-up/down by default:
	 *
	 *		HCI_RX / kWarpPinI2C0_SCL	: pull up
	 *		HCI_TX / kWarpPinI2C0_SDA	: pull up
	 *		HCI_RTS / kWarpPinSPI_MISO	: pull up
	 *		HCI_CTS / kWarpPinSPI_MOSI	: pull up
	 *
	 *	These I/Os are 8mA (see panasonic_PAN13xx.pdf, page 10),
	 *	so we really don't want to be driving them low. We
	 *	however also have to be careful of the I2C pullup and
	 *	pull-up gating. However, driving them high leads to
	 *	higher board power dissipation even when SSSUPPLY is off
	 *	by ~80mW on board #003 (PAN1326 populated).
	 *
	 *	In revB board, with the ISL23415 DCP pullups, we also
	 *	want I2C_SCL and I2C_SDA driven high since when we
	 *	send a shutdown command to the DCP it will connect
	 *	those lines to 25570_VOUT.
	 *
	 *	For now, we therefore leave the SPI pins low and the
	 *	I2C pins (PTB3, PTB4, which are true open-drain) disabled.
	 */

	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	/*
	 *	HCI_RX / kWarpPinI2C0_SCL is an input. Set it low.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);

	/*
	 *	HCI_TX / kWarpPinI2C0_SDA is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);

	/*
	 *	HCI_RTS / kWarpPinSPI_MISO is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO);

	/*
	 *	From PAN1326 manual, page 10:
	 *
	 *		"When HCI_CTS is high, then CC256X is not allowed to send data to Host device"
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI);
}

void disableTPS82740A(void)
{
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
}

void disableTPS82740B(void)
{
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
}

void enableTPS82740A(uint16_t voltageMillivolts)
{
	setTPS82740CommonControlLines(voltageMillivolts);
	GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82740
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
}

void enableTPS82740B(uint16_t voltageMillivolts)
{
	setTPS82740CommonControlLines(voltageMillivolts);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_SetPinOutput(kWarpPinTPS82740B_CTLEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82740
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);
}

void setTPS82740CommonControlLines(uint16_t voltageMillivolts)
{
	/*
	 *	 From Manual:
	 *
	 *		TPS82740A:	VSEL1 VSEL2 VSEL3:	000-->1.8V, 111-->2.5V
	 *		TPS82740B:	VSEL1 VSEL2 VSEL3:	000-->2.6V, 111-->3.3V
	 */

	switch (voltageMillivolts)
	{
	case 2600:
	case 1800:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 2700:
	case 1900:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 2800:
	case 2000:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 2900:
	case 2100:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3000:
	case 2200:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3100:
	case 2300:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3200:
	case 2400:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3300:
	case 2500:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	/*
		 *	Should never happen, due to previous check in enableSssupply()
		 */
	default:
	{
		SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
	}
	}

	/*
	 *	Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
	 */
	OSA_TimeDelay(1);
}

int main(void)
{
	rtc_datetime_t warpBootDate;

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(g_defaultClockConfigurations,
		       CLOCK_CONFIG_NUM,
		       &clockCallbackTable,
		       ARRAY_SIZE(clockCallbackTable));
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Initialize RTC Driver
	 */
	RTC_DRV_Init(0);

	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year = 2016U;
	warpBootDate.month = 1U;
	warpBootDate.day = 1U;
	warpBootDate.hour = 0U;
	warpBootDate.minute = 0U;
	warpBootDate.second = 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins /* input pins */, outputPins /* output pins */);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	lowPowerPinStates();

	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST)
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

	/*
	 * Initialise oled
	 */
	SEGGER_RTT_WriteString(0, "\r\n\tstart oled init...\n");
	OSA_TimeDelay(1000);
	devSSD1331init();
	SEGGER_RTT_WriteString(0, "\r\n\tdone oled init...\n");

	/*
	 *	TODO: initialize the kWarpPinKL03_VDD_ADC, write routines to read the VDD and temperature
	 */

	/*
	 *	Wait for supply and pull-ups to settle.
	 */

	int colour_index = 0;

	const uint8_t colours[NUMBER_OF_COLOURS][3] = {
	    {0, 0, 0},
	    {0x1F, 0, 0},
	    {0, 0x3F, 0},
	    {0, 0, 0x1F},
	};

	const char const *labels[NUMBER_OF_COLOURS] = {
	    "black",
	    "red",
	    "green",
	    "blue",
	};

	while (1)
	{
		unsigned current = 19;
		devSSD1331fill(colours[colour_index][0], colours[colour_index][1], colours[colour_index][2]);
		SEGGER_RTT_printf(0, "\ncolor is %s\t", labels[colour_index]);
		OSA_TimeDelay(500);
		enableI2Cpins(65535);
		read_current(&current);
		SEGGER_RTT_printf(0, "current = %umA", current);
		disableI2Cpins();
		OSA_TimeDelay(500);
		colour_index = (colour_index + 1) % NUMBER_OF_COLOURS;
	}
}
