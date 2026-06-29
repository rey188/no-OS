/***************************************************************************//**
 *   @file   iio_ad5933.c
 *   @brief  Implementation of IIO AD5933/AD5934 driver.
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

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "iio_ad5933.h"
#include "ad5933.h"
#include "iio.h"
#include "no_os_alloc.h"
#include "no_os_print_log.h"
#include "no_os_util.h"

/* Forward declarations */
static int ad5933_iio_read_raw(void *dev, char *buf, uint32_t len,
			       const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_read_scale(void *dev, char *buf, uint32_t len,
				 const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_reg_read(struct ad5933_iio_dev *dev, uint32_t reg,
			       uint32_t *readval);

static int ad5933_iio_reg_write(struct ad5933_iio_dev *dev, uint32_t reg,
				uint32_t writeval);

static int ad5933_iio_show_meas_mode(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv);

static int ad5933_iio_store_meas_mode(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv);

static int ad5933_iio_show_meas_mode_avail(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_excitation_range(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_store_excitation_range(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_excitation_range_avail(void *dev, char *buf,
		uint32_t len, const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_pga_gain(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv);

static int ad5933_iio_store_pga_gain(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv);

static int ad5933_iio_show_pga_gain_avail(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_clock_freq(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv);

static int ad5933_iio_store_clock_freq(void *dev, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv);

static int ad5933_iio_show_start_freq(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv);

static int ad5933_iio_store_start_freq(void *dev, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv);

static int ad5933_iio_show_inc_freq(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv);

static int ad5933_iio_store_inc_freq(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv);

static int ad5933_iio_show_num_increments(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_store_num_increments(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_settling_mult(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_store_settling_mult(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_settling_num(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv);

static int ad5933_iio_store_settling_num(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_cal_impedance(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_store_cal_impedance(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_phase_offset(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv);

static int ad5933_iio_store_phase_offset(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv);

static int ad5933_iio_show_gain_factor(void *dev, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv);

static int ad5933_iio_show_calibrated(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv);

static int ad5933_iio_show_saturated(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv);

static int ad5933_iio_show_freq_warning(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv);

static int ad5933_iio_show_action_avail(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv);

static int ad5933_iio_store_action(void *dev, char *buf, uint32_t len,
				   const struct iio_ch_info *channel,
				   intptr_t priv);

/* Available string tables */
static const char *const ad5933_meas_mode_avail[] = {
	[AD5933_MODE_2WIRE] = "2wire",
	[AD5933_MODE_4WIRE] = "4wire",
};

static const char *const ad5933_excitation_range_avail[] = {
	[AD5933_RANGE_2000mVpp] = "2000mVpp",
	[AD5933_RANGE_200mVpp]  = "200mVpp",
	[AD5933_RANGE_400mVpp]  = "400mVpp",
	[AD5933_RANGE_1000mVpp] = "1000mVpp",
};

static const char *const ad5933_pga_gain_avail[] = {
	[AD5933_GAIN_X5] = "x5",
	[AD5933_GAIN_X1] = "x1",
};

/* Impedance channel attributes */
static struct iio_attribute ad5933_impedance_attrs[] = {
	{
		.name = "raw",
		.show = ad5933_iio_read_raw,
	},
	{
		.name = "scale",
		.show = ad5933_iio_read_scale,
	},
	END_ATTRIBUTES_ARRAY
};

/* IIO channel table */
static struct iio_channel ad5933_channels[] = {
	{
		.name       = "impedance",
		.ch_type    = IIO_RESISTANCE,
		.channel    = 0,
		.address    = AD5933_CH_IMPEDANCE,
		.scan_index = AD5933_CH_IMPEDANCE,
		.attributes = ad5933_impedance_attrs,
		.ch_out     = false,
		.indexed    = 1,
	},
};

/* Device-level debug attributes */
static struct iio_attribute ad5933_debug_attrs[] = {
	END_ATTRIBUTES_ARRAY
};

/* Device-level attributes */
static struct iio_attribute ad5933_iio_attrs[] = {
	{
		.name  = "measurement_mode",
		.show  = ad5933_iio_show_meas_mode,
		.store = ad5933_iio_store_meas_mode,
	},
	{
		.name  = "measurement_mode_available",
		.show  = ad5933_iio_show_meas_mode_avail,
		.store = NULL,
	},
	{
		.name  = "excitation_range",
		.show  = ad5933_iio_show_excitation_range,
		.store = ad5933_iio_store_excitation_range,
	},
	{
		.name  = "excitation_range_available",
		.show  = ad5933_iio_show_excitation_range_avail,
		.store = NULL,
	},
	{
		.name  = "pga_gain",
		.show  = ad5933_iio_show_pga_gain,
		.store = ad5933_iio_store_pga_gain,
	},
	{
		.name  = "pga_gain_available",
		.show  = ad5933_iio_show_pga_gain_avail,
		.store = NULL,
	},
	{
		.name  = "clock_frequency_hz",
		.show  = ad5933_iio_show_clock_freq,
		.store = ad5933_iio_store_clock_freq,
	},
	{
		.name  = "sweep_start_freq_hz",
		.show  = ad5933_iio_show_start_freq,
		.store = ad5933_iio_store_start_freq,
	},
	{
		.name  = "sweep_inc_freq_hz",
		.show  = ad5933_iio_show_inc_freq,
		.store = ad5933_iio_store_inc_freq,
	},
	{
		.name  = "sweep_num_increments",
		.show  = ad5933_iio_show_num_increments,
		.store = ad5933_iio_store_num_increments,
	},
	{
		.name  = "settling_cycles_multiplier",
		.show  = ad5933_iio_show_settling_mult,
		.store = ad5933_iio_store_settling_mult,
	},
	{
		.name  = "settling_cycles_count",
		.show  = ad5933_iio_show_settling_num,
		.store = ad5933_iio_store_settling_num,
	},
	{
		.name  = "calibration_impedance_ohm",
		.show  = ad5933_iio_show_cal_impedance,
		.store = ad5933_iio_store_cal_impedance,
	},
	{
		.name  = "phase_offset_deg",
		.show  = ad5933_iio_show_phase_offset,
		.store = ad5933_iio_store_phase_offset,
	},
	{
		.name  = "gain_factor",
		.show  = ad5933_iio_show_gain_factor,
		.store = NULL,
	},
	{
		.name  = "calibrated",
		.show  = ad5933_iio_show_calibrated,
		.store = NULL,
	},
	{
		.name  = "saturated",
		.show  = ad5933_iio_show_saturated,
		.store = NULL,
	},
	{
		.name  = "freq_warning",
		.show  = ad5933_iio_show_freq_warning,
		.store = NULL,
	},
	{
		.name  = "action",
		.show  = ad5933_iio_show_action_avail,
		.store = ad5933_iio_store_action,
	},
	END_ATTRIBUTES_ARRAY
};

static struct iio_device ad5933_iio_dev = {
	.num_ch           = NO_OS_ARRAY_SIZE(ad5933_channels),
	.channels         = ad5933_channels,
	.attributes       = ad5933_iio_attrs,
	.debug_attributes = ad5933_debug_attrs,
	.debug_reg_read   = (int32_t (*)())ad5933_iio_reg_read,
	.debug_reg_write  = (int32_t (*)())ad5933_iio_reg_write,
};

/******************************************************************************/
/*                           Internal helpers                                 */
/******************************************************************************/

/**
 * @brief Validate sweep frequencies against MCLK-derived bounds and warn.
 */
static int ad5933_iio_validate_freq(struct ad5933_iio_dev *iio_dev,
				    uint32_t start_freq,
				    uint32_t inc_freq,
				    uint16_t num_inc)
{
	uint32_t end_freq;

	iio_dev->freq_warning = false;

	if (start_freq < AD5933_FREQ_MIN_HZ || start_freq > AD5933_FREQ_MAX_HZ) {
		pr_warning("ad5933: start_freq %u Hz out of [%lu, %lu]\n",
			   start_freq, AD5933_FREQ_MIN_HZ, AD5933_FREQ_MAX_HZ);
		iio_dev->freq_warning = true;
	}

	end_freq = start_freq + (uint32_t)inc_freq * num_inc;
	if (end_freq > AD5933_FREQ_MAX_HZ) {
		pr_warning("ad5933: end_freq %u Hz exceeds fmax %lu Hz\n",
			   end_freq, AD5933_FREQ_MAX_HZ);
		iio_dev->freq_warning = true;
	}

	return 0;
}

/**
 * @brief Acquire one DFT sample and compute impedance / admittance / phase.
 *        Detects ADC saturation. Applies per-point gain factor and phase offset.
 *        Point index selects which gain_factor[] entry to use.
 */
static int ad5933_iio_measure(struct ad5933_iio_dev *iio_dev,
			      uint8_t freq_function,
			      uint16_t point_idx)
{
	struct ad5933_dev *dev = iio_dev->ad5933_desc;
	struct ad5933_sweep_result *r = &iio_dev->last;
	int16_t real_data, imag_data;
	double magnitude, gf;

	ad5933_get_data(dev, freq_function,
			(short *)&imag_data, (short *)&real_data);

	r->real = real_data;
	r->imag = imag_data;

	iio_dev->saturated = (abs((int)real_data) > AD5933_SAT_THRESHOLD ||
			      abs((int)imag_data) > AD5933_SAT_THRESHOLD);
	if (iio_dev->saturated)
		pr_warning("ad5933: ADC saturation (R=%d I=%d)\n",
			   real_data, imag_data);

	magnitude   = sqrt((double)real_data * real_data +
			   (double)imag_data * imag_data);
	r->magnitude = magnitude;
	r->phase_deg = atan2((double)imag_data, (double)real_data)
		       * (180.0 / M_PI) - iio_dev->phase_offset;

	if (!iio_dev->calibrated || magnitude == 0.0) {
		r->impedance  = 0.0;
		r->admittance = 0.0;
		return 0;
	}

	gf = iio_dev->gain_factor[point_idx];

	if (iio_dev->mode == AD5933_MODE_2WIRE) {
		/*
		 * 2-wire: Vexcite forced, I measured.
		 * admittance = gf * magnitude,  impedance = 1 / admittance
		 */
		r->admittance = gf * magnitude;
		r->impedance  = (r->admittance != 0.0) ?
				1.0 / r->admittance : 0.0;
	} else {
		/*
		 * 4-wire: Iexcite forced, V measured.
		 * impedance  = 1 / (gf * magnitude),  admittance = 1 / impedance
		 */
		r->impedance  = 1.0 / (gf * magnitude);
		r->admittance = (r->impedance != 0.0) ?
				1.0 / r->impedance : 0.0;
	}

	return 0;
}

/**
 * @brief Program the hardware with the current sweep config and run standby +
 *        reset + init sequence so the first REPEAT/INC command yields valid data.
 */
static void ad5933_iio_prepare_sweep(struct ad5933_iio_dev *iio_dev)
{
	struct ad5933_dev *dev = iio_dev->ad5933_desc;

	ad5933_config_sweep(dev, iio_dev->start_freq, iio_dev->inc_freq,
			    iio_dev->num_increments);
	ad5933_set_settling_time(dev, iio_dev->settling_cycles_mult,
				 iio_dev->settling_cycles_num);
	ad5933_start_sweep(dev);
}

/******************************************************************************/
/*                        Debug register access                               */
/******************************************************************************/

/**
 * @brief Debug register read wrapper.
 * @param dev     - IIO device descriptor.
 * @param reg     - Register address.
 * @param readval - Returned register value.
 * @return 0 in case of success, errno errors otherwise.
 */
static int ad5933_iio_reg_read(struct ad5933_iio_dev *dev, uint32_t reg,
			       uint32_t *readval)
{
	*readval = ad5933_get_register_value(dev->ad5933_desc, (uint8_t)reg, 1);

	return 0;
}

/**
 * @brief Debug register write wrapper.
 * @param dev      - IIO device descriptor.
 * @param reg      - Register address.
 * @param writeval - Value to write.
 * @return 0 in case of success, errno errors otherwise.
 */
static int ad5933_iio_reg_write(struct ad5933_iio_dev *dev, uint32_t reg,
				uint32_t writeval)
{
	ad5933_set_register_value(dev->ad5933_desc, (uint8_t)reg,
				  (uint32_t)writeval, 1);

	return 0;
}

/******************************************************************************/
/*                        Channel attribute handlers                          */
/******************************************************************************/

/**
 * @brief Read impedance channel raw value (last measured impedance in milli-ohms
 *        as integer, so IIO userspace can apply the scale attribute).
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Channel info.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_read_raw(void *dev, char *buf, uint32_t len,
			       const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val;

	/* Represent impedance as integer milli-ohms for IIO raw convention */
	val = (int32_t)(iio_dev->last.impedance * 1000.0);

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Read impedance channel scale (milli-ohms → ohms: 1/1000).
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Channel info.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_read_scale(void *dev, char *buf, uint32_t len,
				 const struct iio_ch_info *channel, intptr_t priv)
{
	int32_t vals[2] = {1, 1000};

	return iio_format_value(buf, len, IIO_VAL_FRACTIONAL, 2, vals);
}

/******************************************************************************/
/*                        Device attribute handlers                           */
/******************************************************************************/

/**
 * @brief Read current measurement mode.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_meas_mode(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;

	return snprintf(buf, len, "%s", ad5933_meas_mode_avail[iio_dev->mode]);
}

/**
 * @brief Write measurement mode ("2wire" or "4wire").
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_meas_mode(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int i;

	for (i = 0; i < (int)NO_OS_ARRAY_SIZE(ad5933_meas_mode_avail); i++)
		if (!strcmp(buf, ad5933_meas_mode_avail[i])) {
			iio_dev->mode = (enum ad5933_meas_mode)i;
			return len;
		}

	return -EINVAL;
}

/**
 * @brief Read available measurement modes.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_meas_mode_avail(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int length = 0;
	int i, ret;

	for (i = 0; i < (int)NO_OS_ARRAY_SIZE(ad5933_meas_mode_avail); i++) {
		ret = snprintf(buf + length, len - length, "%s ",
			       ad5933_meas_mode_avail[i]);
		if (ret < 0 || ret >= (int)(len - length))
			return -ENOMEM;
		length += ret;
	}

	return length;
}

/**
 * @brief Read Tx excitation voltage range.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_excitation_range(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	uint8_t range = iio_dev->ad5933_desc->current_range;

	if (range >= NO_OS_ARRAY_SIZE(ad5933_excitation_range_avail))
		return -EINVAL;

	return snprintf(buf, len, "%s", ad5933_excitation_range_avail[range]);
}

/**
 * @brief Write Tx excitation voltage range (e.g. "2000mVpp", "200mVpp").
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_excitation_range(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int i;

	for (i = 0; i < (int)NO_OS_ARRAY_SIZE(ad5933_excitation_range_avail); i++)
		if (!strcmp(buf, ad5933_excitation_range_avail[i])) {
			ad5933_set_range_and_gain(iio_dev->ad5933_desc, (int8_t)i,
						  iio_dev->ad5933_desc->current_gain);
			return len;
		}

	return -EINVAL;
}

/**
 * @brief Read available excitation voltage ranges.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_excitation_range_avail(void *dev, char *buf,
		uint32_t len, const struct iio_ch_info *channel, intptr_t priv)
{
	int length = 0;
	int i, ret;

	for (i = 0; i < (int)NO_OS_ARRAY_SIZE(ad5933_excitation_range_avail); i++) {
		ret = snprintf(buf + length, len - length, "%s ",
			       ad5933_excitation_range_avail[i]);
		if (ret < 0 || ret >= (int)(len - length))
			return -ENOMEM;
		length += ret;
	}

	return length;
}

/**
 * @brief Read Rx PGA gain setting.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_pga_gain(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	uint8_t gain = iio_dev->ad5933_desc->current_gain;

	if (gain >= NO_OS_ARRAY_SIZE(ad5933_pga_gain_avail))
		return -EINVAL;

	return snprintf(buf, len, "%s", ad5933_pga_gain_avail[gain]);
}

/**
 * @brief Write Rx PGA gain ("x5" or "x1").
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_pga_gain(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int i;

	for (i = 0; i < (int)NO_OS_ARRAY_SIZE(ad5933_pga_gain_avail); i++)
		if (!strcmp(buf, ad5933_pga_gain_avail[i])) {
			ad5933_set_range_and_gain(iio_dev->ad5933_desc,
						  iio_dev->ad5933_desc->current_range,
						  (int8_t)i);
			return len;
		}

	return -EINVAL;
}

/**
 * @brief Read available PGA gain options.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_pga_gain_avail(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	int length = 0;
	int i, ret;

	for (i = 0; i < (int)NO_OS_ARRAY_SIZE(ad5933_pga_gain_avail); i++) {
		ret = snprintf(buf + length, len - length, "%s ",
			       ad5933_pga_gain_avail[i]);
		if (ret < 0 || ret >= (int)(len - length))
			return -ENOMEM;
		length += ret;
	}

	return length;
}

/**
 * @brief Read MCLK frequency in Hz.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_clock_freq(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->ad5933_desc->current_sys_clk;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Write MCLK source/frequency.
 *        Write 0 → internal 16 MHz clock.
 *        Write non-zero → external clock at that frequency in Hz.
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_clock_freq(void *dev, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	uint32_t freq = no_os_str_to_uint32(buf);

	if (freq == 0)
		ad5933_set_system_clk(iio_dev->ad5933_desc,
				      AD5933_CONTROL_INT_SYSCLK, 0);
	else
		ad5933_set_system_clk(iio_dev->ad5933_desc,
				      AD5933_CONTROL_EXT_SYSCLK, freq);

	return len;
}

/**
 * @brief Read sweep start frequency in Hz.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_start_freq(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->start_freq;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Write sweep start frequency in Hz.
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_start_freq(void *dev, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;

	iio_dev->start_freq = no_os_str_to_uint32(buf);
	ad5933_iio_validate_freq(iio_dev, iio_dev->start_freq,
				 iio_dev->inc_freq, iio_dev->num_increments);

	return len;
}

/**
 * @brief Read sweep frequency increment in Hz.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_inc_freq(void *dev, char *buf, uint32_t len,
				    const struct iio_ch_info *channel,
				    intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->inc_freq;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Write sweep frequency increment in Hz.
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_inc_freq(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;

	iio_dev->inc_freq = no_os_str_to_uint32(buf);
	ad5933_iio_validate_freq(iio_dev, iio_dev->start_freq,
				 iio_dev->inc_freq, iio_dev->num_increments);

	return len;
}

/**
 * @brief Read number of frequency sweep increments.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_num_increments(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->num_increments;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Write number of frequency sweep increments (0–511).
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_num_increments(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	uint32_t val = no_os_str_to_uint32(buf);

	if (val > AD5933_IIO_MAX_SWEEP_POINTS)
		return -EINVAL;

	iio_dev->num_increments = (uint16_t)val;
	ad5933_iio_validate_freq(iio_dev, iio_dev->start_freq,
				 iio_dev->inc_freq, iio_dev->num_increments);

	return len;
}

/**
 * @brief Read settling cycles multiplier.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_settling_mult(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->settling_cycles_mult;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Write settling cycles multiplier (0=x1, 1=x2, 3=x4).
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_settling_mult(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	uint32_t mult = no_os_str_to_uint32(buf);

	if (mult != AD5933_SETTLING_X1 && mult != AD5933_SETTLING_X2 &&
	    mult != AD5933_SETTLING_X4)
		return -EINVAL;

	iio_dev->settling_cycles_mult = (uint8_t)mult;
	ad5933_set_settling_time(iio_dev->ad5933_desc,
				 iio_dev->settling_cycles_mult,
				 iio_dev->settling_cycles_num);

	return len;
}

/**
 * @brief Read settling cycles count.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_settling_num(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->settling_cycles_num;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Write settling cycles count (1–511).
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_settling_num(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	uint32_t num = no_os_str_to_uint32(buf);

	if (num == 0 || num > AD5933_IIO_MAX_SWEEP_POINTS)
		return -EINVAL;

	iio_dev->settling_cycles_num = (uint16_t)num;
	ad5933_set_settling_time(iio_dev->ad5933_desc,
				 iio_dev->settling_cycles_mult,
				 iio_dev->settling_cycles_num);

	return len;
}

/**
 * @brief Read calibration resistor value in ohms.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_cal_impedance(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->calibration_impedance;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Write calibration resistor value in ohms. Resets calibrated flag.
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_cal_impedance(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	uint32_t val = no_os_str_to_uint32(buf);

	if (val == 0)
		return -EINVAL;

	iio_dev->calibration_impedance = val;
	iio_dev->calibrated = false;

	return len;
}

/**
 * @brief Read phase offset in degrees.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_phase_offset(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;

	/* Phase offset is a float; use IIO_VAL_INT_PLUS_MICRO for portability */
	int32_t vals[2];
	double off = iio_dev->phase_offset;

	vals[0] = (int32_t)off;
	vals[1] = (int32_t)((off - vals[0]) * 1000000.0);

	return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);
}

/**
 * @brief Write phase offset in degrees (integer part only; use IIO convention).
 *        The value is parsed as a plain integer degrees value.
 * @param dev     - IIO device structure.
 * @param buf     - Input buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_phase_offset(void *dev, char *buf, uint32_t len,
		const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val;
	int ret;

	ret = iio_parse_value(buf, IIO_VAL_INT, &val, NULL);
	if (ret)
		return ret;

	iio_dev->phase_offset = (double)val;

	return len;
}

/**
 * @brief Read gain factor (start_freq point, index 0) as a string.
 *        Reports the gain factor computed at the start frequency after
 *        calibration. Returns "0.0" when not yet calibrated.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_gain_factor(void *dev, char *buf, uint32_t len,
				       const struct iio_ch_info *channel,
				       intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;

	/* Express as integer-plus-micro to avoid %f dependency on nano-libc */
	double gf = iio_dev->gain_factor[0];
	int32_t vals[2];

	vals[0] = (int32_t)gf;
	vals[1] = (int32_t)((gf - vals[0]) * 1000000.0);

	return iio_format_value(buf, len, IIO_VAL_INT_PLUS_MICRO, 2, vals);
}

/**
 * @brief Read calibrated status flag (0 or 1).
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_calibrated(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->calibrated;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Read ADC saturation flag (0 or 1).
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_saturated(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->saturated;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Read frequency-out-of-range warning flag (0 or 1).
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_freq_warning(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int32_t val = (int32_t)iio_dev->freq_warning;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Read available action strings.
 * @param dev     - IIO device structure.
 * @param buf     - Output buffer.
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return Number of bytes written, or negative error code.
 */
static int ad5933_iio_show_action_avail(void *dev, char *buf, uint32_t len,
					const struct iio_ch_info *channel,
					intptr_t priv)
{
	return snprintf(buf, len, "calibrate measure sweep");
}

/**
 * @brief Execute a measurement action.
 *
 *        "calibrate" — performs a full sweep over num_increments+1 points
 *                      using calibration_impedance, stores per-point gain
 *                      factors, and sets calibrated = true.
 *
 *        "measure"   — single-frequency measurement at start_freq using the
 *                      gain factor at index 0.
 *
 *        "sweep"     — sweeps all configured points; per-point results are
 *                      logged via pr_info; last point remains in last.
 *
 * @param dev     - IIO device structure.
 * @param buf     - Action string ("calibrate", "measure", or "sweep").
 * @param len     - Buffer length.
 * @param channel - Unused.
 * @param priv    - Unused.
 * @return len on success, negative error code otherwise.
 */
static int ad5933_iio_store_action(void *dev, char *buf, uint32_t len,
				   const struct iio_ch_info *channel,
				   intptr_t priv)
{
	struct ad5933_iio_dev *iio_dev = (struct ad5933_iio_dev *)dev;
	int ret;
	uint16_t i;
	uint32_t freq;
	double magnitude;
	int16_t real_data, imag_data;

	ret = ad5933_iio_validate_freq(iio_dev, iio_dev->start_freq,
				       iio_dev->inc_freq, iio_dev->num_increments);
	if (ret)
		return ret;

	ad5933_iio_prepare_sweep(iio_dev);

	if (!strncmp(buf, "calibrate", 9)) {
		if (!iio_dev->calibration_impedance)
			return -EINVAL;

		/* Point 0: start_freq, already armed by prepare_sweep */
		ad5933_get_data(iio_dev->ad5933_desc,
				AD5933_FUNCTION_REPEAT_FREQ,
				(short *)&imag_data, (short *)&real_data);

		magnitude = sqrt((double)real_data * real_data +
				 (double)imag_data * imag_data);

		if (magnitude == 0.0)
			return -EIO;

		iio_dev->gain_factor[0] = 1.0 / (magnitude *
					  (double)iio_dev->calibration_impedance);

		/* Remaining points */
		for (i = 1; i <= iio_dev->num_increments; i++) {
			ad5933_get_data(iio_dev->ad5933_desc,
					AD5933_FUNCTION_INC_FREQ,
					(short *)&imag_data, (short *)&real_data);

			magnitude = sqrt((double)real_data * real_data +
					 (double)imag_data * imag_data);

			if (magnitude == 0.0) {
				iio_dev->gain_factor[i] = iio_dev->gain_factor[i - 1];
			} else {
				iio_dev->gain_factor[i] = 1.0 / (magnitude *
						  (double)iio_dev->calibration_impedance);
			}
		}

		iio_dev->calibrated = true;
		pr_info("ad5933: calibration done, gf[0]=%.6e gf[%u]=%.6e\n",
			iio_dev->gain_factor[0],
			iio_dev->num_increments,
			iio_dev->gain_factor[iio_dev->num_increments]);

	} else if (!strncmp(buf, "measure", 7)) {
		ret = ad5933_iio_measure(iio_dev, AD5933_FUNCTION_REPEAT_FREQ, 0);
		if (ret)
			return ret;

	} else if (!strncmp(buf, "sweep", 5)) {
		freq = iio_dev->start_freq;

		ret = ad5933_iio_measure(iio_dev, AD5933_FUNCTION_REPEAT_FREQ, 0);
		if (ret)
			return ret;

		pr_info("ad5933: sweep f=%u Z=%.4f Y=%.9f ph=%.4f\n",
			freq, iio_dev->last.impedance,
			iio_dev->last.admittance, iio_dev->last.phase_deg);

		for (i = 1; i <= iio_dev->num_increments; i++) {
			freq += iio_dev->inc_freq;

			ret = ad5933_iio_measure(iio_dev,
						 AD5933_FUNCTION_INC_FREQ, i);
			if (ret)
				return ret;

			pr_info("ad5933: sweep f=%u Z=%.4f Y=%.9f ph=%.4f\n",
				freq, iio_dev->last.impedance,
				iio_dev->last.admittance, iio_dev->last.phase_deg);
		}

	} else {
		return -EINVAL;
	}

	return len;
}

/******************************************************************************/
/*                              Init / Remove                                 */
/******************************************************************************/

/**
 * @brief Initialize the AD5933/AD5934 IIO driver.
 * @param iio_dev    - Address of the pointer to the IIO device descriptor.
 * @param init_param - Initialization parameters.
 * @return 0 on success, negative errno otherwise.
 */
int ad5933_iio_init(struct ad5933_iio_dev **iio_dev,
		    struct ad5933_iio_init_param *init_param)
{
	struct ad5933_iio_dev *descriptor;
	int ret;

	if (!init_param)
		return -EINVAL;

	descriptor = (struct ad5933_iio_dev *)no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = ad5933_init(&descriptor->ad5933_desc,
			  *init_param->ad5933_init_param);
	if (ret)
		goto init_err;

	descriptor->iio_dev               = &ad5933_iio_dev;
	descriptor->mode                  = init_param->mode;
	descriptor->calibration_impedance = init_param->calibration_impedance;
	descriptor->start_freq            = init_param->start_freq ?
					    init_param->start_freq : 10000U;
	descriptor->inc_freq              = init_param->inc_freq ?
					    init_param->inc_freq  : 100U;
	descriptor->num_increments        = init_param->num_increments;
	descriptor->settling_cycles_mult  = init_param->settling_cycles_mult;
	descriptor->settling_cycles_num   = init_param->settling_cycles_num ?
					    init_param->settling_cycles_num :
					    AD5933_15_CYCLES;
	descriptor->phase_offset          = init_param->phase_offset;
	descriptor->calibrated            = false;
	descriptor->saturated             = false;
	descriptor->freq_warning          = false;

	ad5933_set_settling_time(descriptor->ad5933_desc,
				 descriptor->settling_cycles_mult,
				 descriptor->settling_cycles_num);

	ad5933_iio_validate_freq(descriptor, descriptor->start_freq,
				 descriptor->inc_freq,
				 descriptor->num_increments);

	*iio_dev = descriptor;

	return 0;

init_err:
	no_os_free(descriptor);

	return ret;
}

/**
 * @brief Free resources allocated by ad5933_iio_init().
 * @param desc - IIO device descriptor.
 * @return 0 on success, negative errno otherwise.
 */
int ad5933_iio_remove(struct ad5933_iio_dev *desc)
{
	if (!desc)
		return -EINVAL;

	ad5933_remove(desc->ad5933_desc);
	no_os_free(desc);

	return 0;
}
