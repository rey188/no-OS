/***************************************************************************//**
 *   @file   iio_ad5933.c
 *   @brief  Implementation of AD5933 IIO driver.
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "iio.h"
#include "iio_ad5933.h"
#include "ad5933.h"
#include "no_os_alloc.h"
#include "no_os_error.h"
#include "no_os_util.h"
#include "no_os_print_log.h"

/* Saturation threshold: 16-bit signed magnitude approaching ±32767 */
#define AD5933_SAT_THRESHOLD		32000

/*
 * fmin = MCLK / (2^27 * 4) ≈ 1000 Hz for 16 MHz MCLK.
 * fmax = MCLK / 4 = 4 MHz for 16 MHz MCLK. The datasheet upper limit is
 * 100 kHz for accurate measurements, so we use that as a soft cap.
 */
#define AD5933_FREQ_MIN_HZ		1000UL
#define AD5933_FREQ_MAX_HZ		100000UL

/******************************************************************************/
/*                         Internal helpers                                   */
/******************************************************************************/

/**
 * @brief Validate start/increment/count against clock constraints and warn.
 */
static int ad5933_iio_validate_freq(struct ad5933_iio_dev *dev,
				    uint32_t start_freq,
				    uint32_t inc_freq,
				    uint16_t num_inc)
{
	uint32_t fmin = AD5933_FREQ_MIN_HZ;
	uint32_t fmax = AD5933_FREQ_MAX_HZ;
	uint32_t end_freq;

	dev->freq_warning = false;

	if (start_freq < fmin || start_freq > fmax) {
		pr_warning("ad5933: start_freq %u Hz out of range [%u, %u]\n",
			   start_freq, fmin, fmax);
		dev->freq_warning = true;
	}

	end_freq = start_freq + (uint32_t)inc_freq * num_inc;
	if (end_freq > fmax) {
		pr_warning("ad5933: end_freq %u Hz exceeds fmax %u Hz\n",
			   end_freq, fmax);
		dev->freq_warning = true;
	}

	return 0;
}

/**
 * @brief Perform a single measurement and update last_* fields in iio dev.
 *        Detects saturation. Applies gain factor and phase correction.
 */
static int ad5933_iio_do_measurement(struct ad5933_iio_dev *dev,
				     uint8_t freq_function)
{
	int16_t real_data, imag_data;
	double magnitude, impedance, admittance, phase;

	ad5933_get_data(dev->ad5933_dev, freq_function, &imag_data, &real_data);

	dev->last_real = real_data;
	dev->last_imag = imag_data;

	/* Saturation check */
	dev->saturated = (abs(real_data) > AD5933_SAT_THRESHOLD ||
			  abs(imag_data) > AD5933_SAT_THRESHOLD);
	if (dev->saturated)
		pr_warning("ad5933: ADC saturation detected (R=%d, I=%d)\n",
			   real_data, imag_data);

	magnitude = sqrt((double)real_data * real_data +
			 (double)imag_data * imag_data);

	if (!dev->calibrated || dev->gain_factor == 0.0) {
		dev->last_impedance  = 0.0;
		dev->last_admittance = 0.0;
		dev->last_magnitude  = magnitude;
		dev->last_phase      = atan2((double)imag_data,
					     (double)real_data) * 180.0 / M_PI
				       - dev->phase_offset;
		return 0;
	}

	if (dev->mode == AD5933_MODE_2WIRE) {
		/*
		 * 2-wire: voltage forced, current measured.
		 * admittance = gain_factor * magnitude
		 * impedance  = 1 / admittance
		 */
		admittance = dev->gain_factor * magnitude;
		impedance  = (admittance != 0.0) ? 1.0 / admittance : 0.0;
	} else {
		/*
		 * 4-wire: current forced, voltage measured.
		 * impedance  = 1 / (gain_factor * magnitude)
		 * admittance = 1 / impedance
		 */
		impedance  = (magnitude != 0.0) ?
			     1.0 / (dev->gain_factor * magnitude) : 0.0;
		admittance = (impedance != 0.0) ? 1.0 / impedance : 0.0;
	}

	phase = atan2((double)imag_data, (double)real_data) * 180.0 / M_PI
		- dev->phase_offset;

	dev->last_impedance  = impedance;
	dev->last_admittance = admittance;
	dev->last_magnitude  = magnitude;
	dev->last_phase      = phase;

	return 0;
}

/******************************************************************************/
/*                         Channel attribute callbacks                        */
/******************************************************************************/

/**
 * @brief Read raw real component of the impedance measurement.
 */
static int ad5933_iio_read_real(void *device, char *buf, uint32_t len,
				const struct iio_ch_info *channel,
				intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%d", dev->last_real);
}

/**
 * @brief Read raw imaginary component of the impedance measurement.
 */
static int ad5933_iio_read_imag(void *device, char *buf, uint32_t len,
				const struct iio_ch_info *channel,
				intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%d", dev->last_imag);
}

/**
 * @brief Read calculated impedance magnitude in ohms.
 */
static int ad5933_iio_read_impedance(void *device, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%.6f", dev->last_impedance);
}

/**
 * @brief Read calculated admittance in siemens.
 */
static int ad5933_iio_read_admittance(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%.9f", dev->last_admittance);
}

/**
 * @brief Read DFT magnitude (sqrt(R^2+I^2)).
 */
static int ad5933_iio_read_magnitude(void *device, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%.3f", dev->last_magnitude);
}

/**
 * @brief Read phase angle in degrees (phase-corrected).
 */
static int ad5933_iio_read_phase(void *device, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%.4f", dev->last_phase);
}

/** Impedance channel attributes */
static struct iio_attribute ad5933_iio_impedance_attrs[] = {
	{ .name = "raw_real",    .show = ad5933_iio_read_real,       .store = NULL },
	{ .name = "raw_imag",    .show = ad5933_iio_read_imag,       .store = NULL },
	{ .name = "impedance",   .show = ad5933_iio_read_impedance,  .store = NULL },
	{ .name = "admittance",  .show = ad5933_iio_read_admittance, .store = NULL },
	{ .name = "magnitude",   .show = ad5933_iio_read_magnitude,  .store = NULL },
	{ .name = "phase",       .show = ad5933_iio_read_phase,      .store = NULL },
	END_ATTRIBUTES_ARRAY,
};

/** IIO channel definition */
static struct iio_channel ad5933_iio_channels[] = {
	{
		.name      = "impedance",
		.ch_type   = IIO_RESISTANCE,
		.channel   = 0,
		.attributes = ad5933_iio_impedance_attrs,
		.ch_out    = false,
		.indexed   = 1,
	},
	END_ATTRIBUTES_ARRAY,
};

/******************************************************************************/
/*                         Device attribute callbacks                         */
/******************************************************************************/

/**
 * @brief Read measurement mode (2wire / 4wire).
 */
static int ad5933_iio_read_mode(void *device, char *buf, uint32_t len,
				const struct iio_ch_info *channel,
				intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	const char *mode_str = (dev->mode == AD5933_MODE_2WIRE) ? "2wire" :
			       "4wire";

	return snprintf(buf, len, "%s", mode_str);
}

/**
 * @brief Write measurement mode ("2wire" or "4wire").
 */
static int ad5933_iio_write_mode(void *device, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	if (!strncmp(buf, "2wire", 5))
		dev->mode = AD5933_MODE_2WIRE;
	else if (!strncmp(buf, "4wire", 5))
		dev->mode = AD5933_MODE_4WIRE;
	else
		return -EINVAL;

	return len;
}

/**
 * @brief Read available measurement modes.
 */
static int ad5933_iio_read_mode_avail(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	return snprintf(buf, len, "2wire 4wire");
}

/**
 * @brief Read Tx excitation voltage range.
 */
static int ad5933_iio_read_range(void *device, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%d", dev->ad5933_dev->current_range);
}

/**
 * @brief Write Tx excitation voltage range (0=2000mVpp,1=200mVpp,
 *        2=400mVpp,3=1000mVpp).
 */
static int ad5933_iio_write_range(void *device, char *buf, uint32_t len,
				  const struct iio_ch_info *channel,
				  intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	uint32_t range = no_os_str_to_uint32(buf);

	if (range > AD5933_RANGE_1000mVpp)
		return -EINVAL;

	ad5933_set_range_and_gain(dev->ad5933_dev, (int8_t)range,
				  dev->ad5933_dev->current_gain);
	return len;
}

/**
 * @brief Read available range options.
 */
static int ad5933_iio_read_range_avail(void *device, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv)
{
	return snprintf(buf, len, "0 1 2 3");
}

/**
 * @brief Read PGA gain setting.
 */
static int ad5933_iio_read_pga_gain(void *device, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%d", dev->ad5933_dev->current_gain);
}

/**
 * @brief Write PGA gain (0=x5, 1=x1).
 */
static int ad5933_iio_write_pga_gain(void *device, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	uint32_t gain = no_os_str_to_uint32(buf);

	if (gain > AD5933_GAIN_X1)
		return -EINVAL;

	ad5933_set_range_and_gain(dev->ad5933_dev,
				  dev->ad5933_dev->current_range,
				  (int8_t)gain);
	return len;
}

/**
 * @brief Read available PGA gain options.
 */
static int ad5933_iio_read_pga_gain_avail(void *device, char *buf,
					  uint32_t len,
					  const struct iio_ch_info *channel,
					  intptr_t priv)
{
	return snprintf(buf, len, "0 1");
}

/**
 * @brief Read clock frequency in Hz.
 */
static int ad5933_iio_read_clock_freq(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%u", dev->ad5933_dev->current_sys_clk);
}

/**
 * @brief Write clock source and frequency.
 *        Write 0 to select the 16 MHz internal clock; any other value
 *        selects external clock at that frequency in Hz.
 */
static int ad5933_iio_write_clock_freq(void *device, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	uint32_t freq = no_os_str_to_uint32(buf);

	if (freq == 0) {
		ad5933_set_system_clk(dev->ad5933_dev,
				      AD5933_CONTROL_INT_SYSCLK, 0);
	} else {
		ad5933_set_system_clk(dev->ad5933_dev,
				      AD5933_CONTROL_EXT_SYSCLK, freq);
	}

	return len;
}

/**
 * @brief Read sweep start frequency in Hz.
 */
static int ad5933_iio_read_start_freq(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%u", dev->start_freq);
}

/**
 * @brief Write sweep start frequency in Hz.
 */
static int ad5933_iio_write_start_freq(void *device, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	dev->start_freq = no_os_str_to_uint32(buf);
	ad5933_iio_validate_freq(dev, dev->start_freq, dev->inc_freq,
				 dev->num_increments);
	return len;
}

/**
 * @brief Read sweep frequency increment in Hz.
 */
static int ad5933_iio_read_inc_freq(void *device, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%u", dev->inc_freq);
}

/**
 * @brief Write sweep frequency increment in Hz.
 */
static int ad5933_iio_write_inc_freq(void *device, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	dev->inc_freq = no_os_str_to_uint32(buf);
	ad5933_iio_validate_freq(dev, dev->start_freq, dev->inc_freq,
				 dev->num_increments);
	return len;
}

/**
 * @brief Read number of frequency increments (0–511).
 */
static int ad5933_iio_read_num_increments(void *device, char *buf,
					  uint32_t len,
					  const struct iio_ch_info *channel,
					  intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%u", dev->num_increments);
}

/**
 * @brief Write number of frequency increments (0–511).
 */
static int ad5933_iio_write_num_increments(void *device, char *buf,
					   uint32_t len,
					   const struct iio_ch_info *channel,
					   intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	uint32_t val = no_os_str_to_uint32(buf);

	if (val > AD5933_IIO_MAX_SWEEP_POINTS)
		return -EINVAL;

	dev->num_increments = (uint16_t)val;
	ad5933_iio_validate_freq(dev, dev->start_freq, dev->inc_freq,
				 dev->num_increments);
	return len;
}

/**
 * @brief Read settling cycles multiplier.
 */
static int ad5933_iio_read_settling_mult(void *device, char *buf, uint32_t len,
					 const struct iio_ch_info *channel,
					 intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%u", dev->settling_cycles_mult);
}

/**
 * @brief Write settling cycles multiplier (0=x1, 1=x2, 3=x4).
 */
static int ad5933_iio_write_settling_mult(void *device, char *buf,
					  uint32_t len,
					  const struct iio_ch_info *channel,
					  intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	uint32_t mult = no_os_str_to_uint32(buf);

	if (mult != AD5933_SETTLING_X1 && mult != AD5933_SETTLING_X2 &&
	    mult != AD5933_SETTLING_X4)
		return -EINVAL;

	dev->settling_cycles_mult = (uint8_t)mult;
	ad5933_set_settling_time(dev->ad5933_dev,
				 dev->settling_cycles_mult,
				 dev->settling_cycles_num);
	return len;
}

/**
 * @brief Read settling cycles count.
 */
static int ad5933_iio_read_settling_num(void *device, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%u", dev->settling_cycles_num);
}

/**
 * @brief Write settling cycles count (1–511).
 */
static int ad5933_iio_write_settling_num(void *device, char *buf, uint32_t len,
					 const struct iio_ch_info *channel,
					 intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	uint32_t num = no_os_str_to_uint32(buf);

	if (num == 0 || num > 511)
		return -EINVAL;

	dev->settling_cycles_num = (uint16_t)num;
	ad5933_set_settling_time(dev->ad5933_dev,
				 dev->settling_cycles_mult,
				 dev->settling_cycles_num);
	return len;
}

/**
 * @brief Read calibration impedance value in ohms.
 */
static int ad5933_iio_read_cal_impedance(void *device, char *buf, uint32_t len,
					 const struct iio_ch_info *channel,
					 intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%u", dev->calibration_impedance);
}

/**
 * @brief Write calibration impedance value in ohms.
 */
static int ad5933_iio_write_cal_impedance(void *device, char *buf,
					  uint32_t len,
					  const struct iio_ch_info *channel,
					  intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	uint32_t val = no_os_str_to_uint32(buf);

	if (val == 0)
		return -EINVAL;

	dev->calibration_impedance = val;
	dev->calibrated = false;

	return len;
}

/**
 * @brief Read phase offset (degrees).
 */
static int ad5933_iio_read_phase_offset(void *device, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%.4f", dev->phase_offset);
}

/**
 * @brief Write phase offset in degrees.
 */
static int ad5933_iio_write_phase_offset(void *device, char *buf, uint32_t len,
					 const struct iio_ch_info *channel,
					 intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	dev->phase_offset = strtod(buf, NULL);

	return len;
}

/**
 * @brief Read current gain factor (computed during calibration).
 */
static int ad5933_iio_read_gain_factor(void *device, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%.10e", dev->gain_factor);
}

/**
 * @brief Read saturation flag.
 */
static int ad5933_iio_read_saturated(void *device, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%d", (int)dev->saturated);
}

/**
 * @brief Read frequency-out-of-range warning flag.
 */
static int ad5933_iio_read_freq_warning(void *device, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%d", (int)dev->freq_warning);
}

/**
 * @brief Read calibrated flag.
 */
static int ad5933_iio_read_calibrated(void *device, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;

	return snprintf(buf, len, "%d", (int)dev->calibrated);
}

/**
 * @brief Trigger single-frequency calibration or measurement.
 *
 *        Writing "calibrate" runs calibration at start_freq with the known
 *        calibration_impedance resistor and stores the gain factor.
 *        Writing "measure" performs a single measurement at start_freq.
 *        Writing "sweep" runs a full frequency sweep collecting impedance at
 *        each step; results are reported for the last point (sweep data is
 *        printed via pr_info for each point).
 */
static int ad5933_iio_write_action(void *device, char *buf, uint32_t len,
				   const struct iio_ch_info *channel,
				   intptr_t priv)
{
	struct ad5933_iio_dev *dev = (struct ad5933_iio_dev *)device;
	int ret;

	ret = ad5933_iio_validate_freq(dev, dev->start_freq, dev->inc_freq,
				       dev->num_increments);
	if (ret)
		return ret;

	ad5933_config_sweep(dev->ad5933_dev, dev->start_freq, dev->inc_freq,
			    dev->num_increments);
	ad5933_set_settling_time(dev->ad5933_dev, dev->settling_cycles_mult,
				 dev->settling_cycles_num);

	if (!strncmp(buf, "calibrate", 9)) {
		if (!dev->calibration_impedance)
			return -EINVAL;

		ad5933_start_sweep(dev->ad5933_dev);
		dev->gain_factor = ad5933_calculate_gain_factor(
			dev->ad5933_dev, dev->calibration_impedance,
			AD5933_FUNCTION_REPEAT_FREQ);
		dev->calibrated = (dev->gain_factor != 0.0);
		pr_info("ad5933: calibration done, gain_factor=%.10e\n",
			dev->gain_factor);

	} else if (!strncmp(buf, "measure", 7)) {
		ad5933_start_sweep(dev->ad5933_dev);
		ret = ad5933_iio_do_measurement(dev,
						AD5933_FUNCTION_REPEAT_FREQ);
		if (ret)
			return ret;

	} else if (!strncmp(buf, "sweep", 5)) {
		uint16_t i;
		uint32_t freq = dev->start_freq;

		ad5933_start_sweep(dev->ad5933_dev);
		ret = ad5933_iio_do_measurement(dev,
						AD5933_FUNCTION_REPEAT_FREQ);
		if (ret)
			return ret;

		pr_info("ad5933: sweep freq=%u Z=%.4f Ohm Y=%.9f S ph=%.4f deg\n",
			freq, dev->last_impedance, dev->last_admittance,
			dev->last_phase);

		for (i = 0; i < dev->num_increments; i++) {
			freq += dev->inc_freq;
			ret = ad5933_iio_do_measurement(
				dev, AD5933_FUNCTION_INC_FREQ);
			if (ret)
				return ret;

			pr_info("ad5933: sweep freq=%u Z=%.4f Ohm Y=%.9f S ph=%.4f deg\n",
				freq, dev->last_impedance,
				dev->last_admittance, dev->last_phase);
		}

	} else {
		return -EINVAL;
	}

	return len;
}

/**
 * @brief Read available action strings.
 */
static int ad5933_iio_read_action_avail(void *device, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	return snprintf(buf, len, "calibrate measure sweep");
}

/** Device-level IIO attributes */
static struct iio_attribute ad5933_iio_attributes[] = {
	{ .name = "measurement_mode",
	  .show = ad5933_iio_read_mode,
	  .store = ad5933_iio_write_mode },
	{ .name = "measurement_mode_available",
	  .show = ad5933_iio_read_mode_avail,
	  .store = NULL },
	{ .name = "excitation_range",
	  .show = ad5933_iio_read_range,
	  .store = ad5933_iio_write_range },
	{ .name = "excitation_range_available",
	  .show = ad5933_iio_read_range_avail,
	  .store = NULL },
	{ .name = "pga_gain",
	  .show = ad5933_iio_read_pga_gain,
	  .store = ad5933_iio_write_pga_gain },
	{ .name = "pga_gain_available",
	  .show = ad5933_iio_read_pga_gain_avail,
	  .store = NULL },
	{ .name = "clock_frequency_hz",
	  .show = ad5933_iio_read_clock_freq,
	  .store = ad5933_iio_write_clock_freq },
	{ .name = "sweep_start_freq_hz",
	  .show = ad5933_iio_read_start_freq,
	  .store = ad5933_iio_write_start_freq },
	{ .name = "sweep_inc_freq_hz",
	  .show = ad5933_iio_read_inc_freq,
	  .store = ad5933_iio_write_inc_freq },
	{ .name = "sweep_num_increments",
	  .show = ad5933_iio_read_num_increments,
	  .store = ad5933_iio_write_num_increments },
	{ .name = "settling_cycles_multiplier",
	  .show = ad5933_iio_read_settling_mult,
	  .store = ad5933_iio_write_settling_mult },
	{ .name = "settling_cycles_count",
	  .show = ad5933_iio_read_settling_num,
	  .store = ad5933_iio_write_settling_num },
	{ .name = "calibration_impedance_ohm",
	  .show = ad5933_iio_read_cal_impedance,
	  .store = ad5933_iio_write_cal_impedance },
	{ .name = "phase_offset_deg",
	  .show = ad5933_iio_read_phase_offset,
	  .store = ad5933_iio_write_phase_offset },
	{ .name = "gain_factor",
	  .show = ad5933_iio_read_gain_factor,
	  .store = NULL },
	{ .name = "calibrated",
	  .show = ad5933_iio_read_calibrated,
	  .store = NULL },
	{ .name = "saturated",
	  .show = ad5933_iio_read_saturated,
	  .store = NULL },
	{ .name = "freq_warning",
	  .show = ad5933_iio_read_freq_warning,
	  .store = NULL },
	{ .name = "action",
	  .show = ad5933_iio_read_action_avail,
	  .store = ad5933_iio_write_action },
	END_ATTRIBUTES_ARRAY,
};

/** IIO device descriptor */
static struct iio_device ad5933_iio_device = {
	.num_ch     = 1,
	.channels   = ad5933_iio_channels,
	.attributes = ad5933_iio_attributes,
	.debug_attributes = NULL,
	.buffer_attributes = NULL,
};

/******************************************************************************/
/*                         Init / Remove                                      */
/******************************************************************************/

/***************************************************************************//**
 * @brief Initialize the AD5933 IIO driver.
 *
 * @param iio_dev    - Address of the pointer to the IIO device descriptor.
 * @param init_param - Initialization parameters.
 *
 * @return 0 on success, negative error code otherwise.
*******************************************************************************/
int ad5933_iio_init(struct ad5933_iio_dev **iio_dev,
		    struct ad5933_iio_init_param *init_param)
{
	struct ad5933_iio_dev *dev;
	int ret;

	if (!iio_dev || !init_param)
		return -EINVAL;

	dev = (struct ad5933_iio_dev *)no_os_calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	ret = ad5933_init(&dev->ad5933_dev, *init_param->ad5933_init_param);
	if (ret)
		goto error_free;

	dev->iio_dev                = &ad5933_iio_device;
	dev->mode                   = init_param->mode;
	dev->calibration_impedance  = init_param->calibration_impedance;

	/* Defaults */
	dev->start_freq             = 10000;
	dev->inc_freq               = 100;
	dev->num_increments         = 100;
	dev->settling_cycles_mult   = AD5933_SETTLING_X1;
	dev->settling_cycles_num    = AD5933_15_CYCLES;
	dev->gain_factor            = 0.0;
	dev->phase_offset           = 0.0;
	dev->calibrated             = false;
	dev->saturated              = false;
	dev->freq_warning           = false;

	*iio_dev = dev;

	return 0;

error_free:
	no_os_free(dev);
	return ret;
}

/***************************************************************************//**
 * @brief Free resources allocated by ad5933_iio_init().
 *
 * @param iio_dev - IIO device descriptor.
 *
 * @return 0 on success, negative error code otherwise.
*******************************************************************************/
int ad5933_iio_remove(struct ad5933_iio_dev *iio_dev)
{
	int ret;

	if (!iio_dev)
		return -EINVAL;

	ret = ad5933_remove(iio_dev->ad5933_dev);

	no_os_free(iio_dev);

	return ret;
}
