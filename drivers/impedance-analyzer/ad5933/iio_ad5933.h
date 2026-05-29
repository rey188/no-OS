/***************************************************************************//**
 *   @file   iio_ad5933.h
 *   @brief  Header file for the AD5933 IIO driver.
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
#ifndef IIO_AD5933_H
#define IIO_AD5933_H

#include <stdint.h>
#include <stdbool.h>
#include "iio.h"
#include "ad5933.h"

#define AD5933_IIO_MAX_SWEEP_POINTS	511

enum ad5933_measurement_mode {
	AD5933_MODE_2WIRE,
	AD5933_MODE_4WIRE,
};

struct ad5933_iio_dev {
	struct ad5933_dev *ad5933_dev;
	struct iio_device *iio_dev;

	enum ad5933_measurement_mode mode;

	uint32_t start_freq;
	uint32_t inc_freq;
	uint16_t num_increments;
	uint8_t settling_cycles_mult;
	uint16_t settling_cycles_num;

	uint32_t calibration_impedance;
	double gain_factor;
	double phase_offset;
	bool calibrated;

	int16_t last_real;
	int16_t last_imag;
	double last_impedance;
	double last_admittance;
	double last_phase;
	double last_magnitude;

	bool saturated;
	bool freq_warning;
};

struct ad5933_iio_init_param {
	struct ad5933_init_param *ad5933_init_param;
	enum ad5933_measurement_mode mode;
	uint32_t calibration_impedance;
};

int ad5933_iio_init(struct ad5933_iio_dev **iio_dev,
		    struct ad5933_iio_init_param *init_param);

int ad5933_iio_remove(struct ad5933_iio_dev *iio_dev);

#endif /* IIO_AD5933_H */
