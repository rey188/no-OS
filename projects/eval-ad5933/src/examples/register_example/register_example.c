/*******************************************************************************
 *   @file   register_example.c
 *   @brief  Simple I2C register readback example for eval-ad5933.
 *   @author Reymond Olmedo (reymond.olmedo@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <errno.h>
#include "register_example.h"
#include "common_data.h"
#include "ad5933.h"
#include "no_os_uart.h"
#include "no_os_print_log.h"
#include "no_os_util.h"

/*
 * One register-readback check: write a known value to a register, read it
 * back over I2C and report whether the device echoed the same value.
 */
struct reg_check {
	const char *name;	/* Human-readable register name. */
	uint8_t     addr;	/* Register start address. */
	uint8_t     bytes;	/* Number of bytes in the register. */
	uint32_t    value;	/* Value to write and expect back. */
};

/*
 * Registers safe to write/read as a connectivity check. The frequency, gain
 * and settling-time registers are plain R/W storage on the AD5933, so writing
 * a known pattern and reading it back proves the I2C link works end-to-end.
 *
 * The control register high byte is included with a NOP function and a benign
 * range/gain so it does not start a measurement.
 */
static const struct reg_check reg_checks[] = {
	{
		.name  = "CONTROL_HB",
		.addr  = AD5933_REG_CONTROL_HB,
		.bytes = 1,
		.value = AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_NOP)  |
		AD5933_CONTROL_RANGE(AD5933_RANGE_2000mVpp) |
		AD5933_CONTROL_PGA_GAIN(AD5933_GAIN_X1),
	},
	{
		.name  = "FREQ_START (24-bit)",
		.addr  = AD5933_REG_FREQ_START,
		.bytes = 3,
		.value = 0x123456,
	},
	{
		.name  = "FREQ_INC (24-bit)",
		.addr  = AD5933_REG_FREQ_INC,
		.bytes = 3,
		.value = 0x000ABC,
	},
	{
		.name  = "INC_NUM (16-bit)",
		.addr  = AD5933_REG_INC_NUM,
		.bytes = 2,
		.value = 0x01FF,
	},
	{
		.name  = "SETTLING_CYCLES (16-bit)",
		.addr  = AD5933_REG_SETTLING_CYCLES,
		.bytes = 2,
		.value = 0x000F,
	},
};

/***************************************************************************//**
 * @brief Register readback example main execution.
 *
 *        Initializes UART (for printf/pr_info output) and the AD5933 over I2C,
 *        then for each test register: writes a known value, reads it back via
 *        ad5933_get_register_value() and prints PASS/FAIL. This is a quick way
 *        to confirm the SDP-K1 <-> AD5933 eval board I2C connection is good.
 *
 * @return ret - 0 on success (all registers matched), negative error code or
 *               the number of mismatches otherwise.
*******************************************************************************/
int register_example_main(void)
{
	struct ad5933_dev *ad5933_desc;
	struct no_os_uart_desc *uart_desc;
	uint32_t readback;
	uint32_t mask;
	float temperature;
	int temp_centi;
	unsigned int i;
	int failures = 0;
	int ret;

	ret = no_os_uart_init(&uart_desc, &uip);
	if (ret)
		return ret;

	no_os_uart_stdio(uart_desc);

	pr_info("\nAD5933 I2C register readback example\n");
	pr_info("Slave address: 0x%02X\n\n", AD5933_ADDRESS);

	ret = ad5933_init(&ad5933_desc, ad5933_init_ip);
	if (ret) {
		pr_info("ad5933_init failed (%d) - check I2C wiring/power.\n", ret);
		goto err_uart;
	}

	for (i = 0; i < NO_OS_ARRAY_SIZE(reg_checks); i++) {
		/* Only the low (8 * bytes) bits are meaningful for comparison. */
		mask = (reg_checks[i].bytes >= 4) ?
		       0xFFFFFFFFu : ((1u << (8 * reg_checks[i].bytes)) - 1);

		ad5933_set_register_value(ad5933_desc, reg_checks[i].addr,
					  reg_checks[i].value, reg_checks[i].bytes);

		readback = ad5933_get_register_value(ad5933_desc, reg_checks[i].addr,
						     reg_checks[i].bytes);

		if ((readback & mask) == (reg_checks[i].value & mask)) {
			pr_info("[ PASS ] %-24s addr 0x%02X wrote 0x%06lX read 0x%06lX\n",
				reg_checks[i].name, reg_checks[i].addr,
				(unsigned long)(reg_checks[i].value & mask),
				(unsigned long)(readback & mask));
		} else {
			failures++;
			pr_info("[ FAIL ] %-24s addr 0x%02X wrote 0x%06lX read 0x%06lX\n",
				reg_checks[i].name, reg_checks[i].addr,
				(unsigned long)(reg_checks[i].value & mask),
				(unsigned long)(readback & mask));
		}
	}

	/* Read-only registers: just dump the current value (no compare). */
	readback = ad5933_get_register_value(ad5933_desc, AD5933_REG_STATUS, 1);
	pr_info("\nSTATUS register (read-only)      addr 0x%02X read 0x%02lX\n",
		AD5933_REG_STATUS, (unsigned long)readback);

	/*
	 * On-chip temperature sensor: triggers a MEASURE_TEMP conversion and
	 * returns degrees Celsius. Printed as integer.fraction (in 1/100 C)
	 * to avoid depending on floating-point printf support.
	 */
	temperature = ad5933_get_temperature(ad5933_desc);
	temp_centi = (int)(temperature * 100.0f + (temperature < 0 ? -0.5f : 0.5f));
	pr_info("On-chip temperature              addr 0x%02X read %d.%02d C\n",
		AD5933_REG_TEMP_DATA, temp_centi / 100,
		(temp_centi < 0 ? -temp_centi : temp_centi) % 100);

	if (failures == 0)
		pr_info("\nResult: ALL REGISTERS MATCHED - I2C readback OK.\n");
	else
		pr_info("\nResult: %d register(s) MISMATCHED - check connection.\n",
			failures);

	ad5933_remove(ad5933_desc);
	no_os_uart_remove(uart_desc);

	return failures;

err_uart:
	no_os_uart_remove(uart_desc);
	return ret;
}
