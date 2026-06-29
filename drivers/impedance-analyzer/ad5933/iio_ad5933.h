/***************************************************************************//**
 *   @file   iio_ad5933.h
 *   @brief  Header file of IIO AD5933/AD5934 driver.
 *   @author Reymond Olmedo (reymond.olmedo@analog.com)
 *******************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
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
 ******************************************************************************/

#ifndef IIO_AD5933_H
#define IIO_AD5933_H

#include <stdbool.h>
#include <stdint.h>
#include "iio.h"
#include "ad5933.h"

/** Maximum number of frequency sweep points (hardware limit) */
#define AD5933_IIO_MAX_SWEEP_POINTS	511

/** ADC saturation threshold — 16-bit signed magnitude near full-scale */
#define AD5933_SAT_THRESHOLD		32000

/**
 * Frequency bounds derived from MCLK:
 *   fmin = MCLK / (2^27 * 4) ≈ 1000 Hz @ 16 MHz
 *   fmax soft cap per datasheet accuracy spec = 100 kHz
 */
#define AD5933_FREQ_MIN_HZ		1000UL
#define AD5933_FREQ_MAX_HZ		100000UL

/** @enum ad5933_meas_mode
 *  @brief Measurement path topology
 */
enum ad5933_meas_mode {
	AD5933_MODE_2WIRE = 0,	/* Voltage forced, current measured → admittance primary */
	AD5933_MODE_4WIRE = 1,	/* Current forced, voltage measured → impedance primary  */
};

/** @enum ad5933_iio_chan_idx
 *  @brief IIO channel scan indices
 */
enum ad5933_iio_chan_idx {
	AD5933_CH_IMPEDANCE = 0,
	AD5933_NUM_CH,
};

/**
 * @struct ad5933_sweep_result
 * @brief Per-point result from a sweep or single measurement
 */
struct ad5933_sweep_result {
	/** Raw DFT real component from ADC */
	int16_t real;
	/** Raw DFT imaginary component from ADC */
	int16_t imag;
	/** Computed DFT vector magnitude sqrt(R^2 + I^2) */
	double magnitude;
	/** Phase angle in degrees, phase-offset corrected */
	double phase_deg;
	/** Impedance in ohms (derived from gain factor) */
	double impedance;
	/** Admittance in siemens (reciprocal of impedance) */
	double admittance;
};

/**
 * @struct ad5933_iio_dev
 * @brief AD5933/AD5934 IIO device descriptor
 */
struct ad5933_iio_dev {
	/** Pointer to the underlying AD5933 hardware driver descriptor */
	struct ad5933_dev *ad5933_desc;
	/** IIO device descriptor for framework registration */
	struct iio_device *iio_dev;

	/** Active measurement path topology */
	enum ad5933_meas_mode mode;

	/* --- Sweep configuration ----------------------------------------- */
	/** Sweep start frequency in Hz */
	uint32_t start_freq;
	/** Frequency step between sweep points in Hz */
	uint32_t inc_freq;
	/** Number of frequency increments (0–511) */
	uint16_t num_increments;
	/** Settling cycles count (1–511) */
	uint16_t settling_cycles_num;
	/** Settling cycles multiplier (AD5933_SETTLING_X1/X2/X4) */
	uint8_t settling_cycles_mult;

	/* --- Calibration -------------------------------------------------- */
	/** Known precision calibration resistor in ohms */
	uint32_t calibration_impedance;
	/**
	 * Per-point gain factors; one entry per sweep point (index 0 = start_freq).
	 * Sized for maximum sweep depth to avoid dynamic allocation.
	 */
	double gain_factor[AD5933_IIO_MAX_SWEEP_POINTS + 1];
	/** Phase offset in degrees applied to every measurement */
	double phase_offset;
	/** True once a successful calibration sweep has been completed */
	bool calibrated;

	/* --- Last measurement result -------------------------------------- */
	struct ad5933_sweep_result last;

	/* --- Status flags ------------------------------------------------- */
	/** True when ADC output magnitude exceeds AD5933_SAT_THRESHOLD */
	bool saturated;
	/** True when any configured frequency falls outside [fmin, fmax] */
	bool freq_warning;
};

/**
 * @struct ad5933_iio_init_param
 * @brief Initialization parameters for the AD5933/AD5934 IIO driver
 */
struct ad5933_iio_init_param {
	/** Hardware initialization parameters passed to ad5933_init() */
	struct ad5933_init_param *ad5933_init_param;
	/** Measurement topology: 2-wire or 4-wire */
	enum ad5933_meas_mode mode;
	/** Known calibration resistor in ohms (must be non-zero) */
	uint32_t calibration_impedance;
	/** Initial sweep start frequency in Hz */
	uint32_t start_freq;
	/** Initial sweep frequency increment in Hz */
	uint32_t inc_freq;
	/** Initial number of sweep increments (0–511) */
	uint16_t num_increments;
	/** Initial settling cycles count */
	uint16_t settling_cycles_num;
	/** Initial settling cycles multiplier */
	uint8_t settling_cycles_mult;
	/** Initial phase offset in degrees */
	double phase_offset;
};

int ad5933_iio_init(struct ad5933_iio_dev **iio_dev,
		    struct ad5933_iio_init_param *init_param);

int ad5933_iio_remove(struct ad5933_iio_dev *desc);

#endif /* IIO_AD5933_H */
