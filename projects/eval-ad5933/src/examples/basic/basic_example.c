/*******************************************************************************
 *   @file   basic_example.c
 *   @brief  Impedance frequency-sweep example for eval-ad5933.
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
#include "basic_example.h"
#include "common_data.h"
#include "ad5933.h"
#include "no_os_uart.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"

/* Sweep definition: start frequency, increment and number of increments. */
#define BASIC_START_FREQ_HZ	30000U	/* 30 kHz start frequency. */
#define BASIC_FREQ_INC_HZ	1000U	/* 1 kHz per step.         */
#define BASIC_INC_NUM		10U	/* 10 steps (11 points).   */

/* Settling time before each measurement, expressed in output cycles. */
#define BASIC_SETTLING_CYCLES	15U

/***************************************************************************//**
 * @brief Basic impedance sweep example main execution.
 *
 *        Initializes UART (for printf/pr_info output) and the AD5933 over I2C,
 *        configures a frequency sweep, calibrates the gain factor against the
 *        known calibration resistor and then measures and prints the impedance
 *        at each frequency point of the sweep.
 *
 * @return ret - 0 on success, negative error code otherwise.
*******************************************************************************/
int basic_example_main(void)
{
	struct ad5933_dev *ad5933_desc;
	struct no_os_uart_desc *uart_desc;
	double gain_factor;
	double impedance;
	uint32_t freq;
	unsigned int i;
	int imp_milli;
	int ret;

	ret = no_os_uart_init(&uart_desc, &uip);
	if (ret)
		return ret;

	no_os_uart_stdio(uart_desc);

	pr_info("\nAD5933 impedance sweep example\n");
	pr_info("Calibration resistor: %lu ohm\n\n",
		(unsigned long)AD5933_CALIBRATION_RESISTOR_OHM);

	ret = ad5933_init(&ad5933_desc, ad5933_init_ip);
	if (ret) {
		pr_info("ad5933_init failed (%d) - check I2C wiring/power.\n", ret);
		goto err_uart;
	}

	/* Use the internal system clock and default range/gain from init. */
	ad5933_set_system_clk(ad5933_desc, AD5933_CONTROL_INT_SYSCLK,
			      AD5933_INTERNAL_SYS_CLK);
	ad5933_set_range_and_gain(ad5933_desc, ad5933_init_ip.current_range,
				  ad5933_init_ip.current_gain);

	/* Program the sweep and the settling time. */
	ad5933_config_sweep(ad5933_desc, BASIC_START_FREQ_HZ,
			    BASIC_FREQ_INC_HZ, BASIC_INC_NUM);
	ad5933_set_settling_time(ad5933_desc, AD5933_SETTLING_X1,
				 BASIC_SETTLING_CYCLES);

	/* Start the sweep: standby -> init start freq -> start sweep. */
	ad5933_start_sweep(ad5933_desc);

	/*
	 * Calibrate on the first sweep point. The gain factor maps the DFT
	 * magnitude to admittance using the known calibration resistor, and is
	 * reused for the impedance calculation at every subsequent point.
	 */
	gain_factor = ad5933_calculate_gain_factor(ad5933_desc,
			AD5933_CALIBRATION_RESISTOR_OHM,
			AD5933_FUNCTION_REPEAT_FREQ);
	pr_info("Gain factor calibrated at start frequency.\n\n");

	pr_info("%-12s %-16s\n", "Freq [Hz]", "Impedance [ohm]");

	freq = BASIC_START_FREQ_HZ;
	for (i = 0; i <= BASIC_INC_NUM; i++) {
		impedance = ad5933_calculate_impedance(ad5933_desc, gain_factor,
						       AD5933_FUNCTION_REPEAT_FREQ);

		/*
		 * Print as integer.milli to avoid depending on floating-point
		 * printf support in the platform C library.
		 */
		imp_milli = (int)(impedance * 1000.0 + 0.5);
		pr_info("%-12lu %d.%03d\n", (unsigned long)freq,
			imp_milli / 1000, imp_milli % 1000);

		/* Step to the next frequency point (skip after the last one). */
		if (i < BASIC_INC_NUM) {
			ad5933_set_register_value(ad5933_desc,
						  AD5933_REG_CONTROL_HB,
						  AD5933_CONTROL_FUNCTION(
							  AD5933_FUNCTION_INC_FREQ) |
						  AD5933_CONTROL_RANGE(
							  ad5933_init_ip.current_range) |
						  AD5933_CONTROL_PGA_GAIN(
							  ad5933_init_ip.current_gain),
						  1);
			freq += BASIC_FREQ_INC_HZ;
		}
	}

	/* Place the device in power-down when the sweep is complete. */
	ad5933_set_register_value(ad5933_desc, AD5933_REG_CONTROL_HB,
				  AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_POWER_DOWN),
				  1);

	pr_info("\nSweep complete.\n");

	ad5933_remove(ad5933_desc);
	no_os_uart_remove(uart_desc);

	return 0;

err_uart:
	no_os_uart_remove(uart_desc);
	return ret;
}
