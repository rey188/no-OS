/*******************************************************************************
 *   @file   iio_example.c
 *   @brief  Implementation of IIO example for eval-ad5933 project.
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
#include "iio_example.h"
#include "iio_ad5933.h"
#include "common_data.h"

/***************************************************************************//**
 * @brief IIO example main execution.
 *
 *        Initializes the AD5933 IIO device and starts the IIO application loop.
 *        The IIO application exposes all AD5933 attributes over UART so a host
 *        tool (e.g. IIO Oscilloscope) can:
 *          - Configure measurement_mode (2wire/4wire)
 *          - Configure excitation_range, pga_gain, clock_frequency_hz
 *          - Configure sweep parameters (start/increment/count)
 *          - Configure calibration_impedance_ohm and phase_offset_deg
 *          - Trigger calibrate / measure / sweep via the "action" attribute
 *          - Read back impedance, admittance, phase, magnitude, raw I/Q
 *          - Monitor the calibrated, saturated, freq_warning status flags
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously inside iio_app_run and will not return.
*******************************************************************************/
int iio_example_main(void)
{
	int ret;
	struct ad5933_iio_dev *ad5933_iio_desc;
	struct ad5933_iio_init_param ad5933_iio_ip = {
		.ad5933_init_param      = &ad5933_init_ip,
		.mode                   = AD5933_MODE_2WIRE,
		.calibration_impedance  = AD5933_CALIBRATION_RESISTOR_OHM,
	};
	struct iio_app_desc *app;
	struct iio_app_init_param app_init_param = { 0 };

	ret = ad5933_iio_init(&ad5933_iio_desc, &ad5933_iio_ip);
	if (ret)
		return ret;

	struct iio_app_device iio_devices[] = {
		{
			.name           = "ad5933",
			.dev            = ad5933_iio_desc,
			.dev_descriptor = ad5933_iio_desc->iio_dev,
		}
	};

	app_init_param.devices        = iio_devices;
	app_init_param.nb_devices     = NO_OS_ARRAY_SIZE(iio_devices);
	app_init_param.uart_init_params = uip;

	ret = iio_app_init(&app, app_init_param);
	if (ret)
		goto err_remove;

	return iio_app_run(app);

err_remove:
	ad5933_iio_remove(ad5933_iio_desc);
	return ret;
}
