/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MS5611.hpp"

MS5611::MS5611(device::Device *interface, ms5611::prom_u &prom_buf, enum MS56XX_DEVICE_TYPES device_type) :
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_px4_barometer(interface->get_device_id()),
	_interface(interface),
	_prom(prom_buf.s),
	_device_type(device_type),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
}

MS5611::~MS5611()
{
	/* make sure we are truly inactive */
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
MS5611::init()
{
	int ret;
	bool autodetect = false;

	ret = CDev::init();

	if (ret != OK) {
		PX4_DEBUG("CDev init failed");
		goto out;
	}

	if (_device_type == MS56XX_DEVICE) {
		autodetect = true;
		/* try first with MS5611 and fallback to MS5607 */
		_device_type = MS5611_DEVICE;
	}

	while (true) {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		_reports->get(&brp);

		if (autodetect) {
			if (_device_type == MS5611_DEVICE) {
				if (brp.pressure < 520.0f) {
					/* This is likely not this device, try again */
					_device_type = MS5607_DEVICE;
					_measure_phase = 0;

					continue;
				}

			} else if (_device_type == MS5607_DEVICE) {
				if (brp.pressure < 520.0f) {
					/* Both devices returned a very low pressure;
					 * have fun on Everest using MS5611 */
					_device_type = MS5611_DEVICE;
				}
			}
		}

		switch (_device_type) {
		default:

		/* fall through */
		case MS5611_DEVICE:
			_interface->set_device_type(DRV_BARO_DEVTYPE_MS5611);
			break;

		case MS5607_DEVICE:
			_interface->set_device_type(DRV_BARO_DEVTYPE_MS5607);
			break;
		}

		ret = OK;

		break;
	}

out:
	return ret;
}

void
MS5611::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
MS5611::stop()
{
	ScheduleClear();
}

void
MS5611::Run()
{
	int ret;
	unsigned dummy;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The ms5611 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			_interface->ioctl(IOCTL_RESET, dummy);
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5611 datasheet
			 */
			ScheduleDelayed(2800);
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_interval > MS5611_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - MS5611_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		/* issue a reset command to the sensor */
		_interface->ioctl(IOCTL_RESET, dummy);
		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(MS5611_CONVERSION_INTERVAL);
}

int
MS5611::measure()
{
	int ret;

	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	ret = _interface->ioctl(IOCTL_MEASURE, addr);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return ret;
}

int
MS5611::collect()
{
	perf_begin(_sample_perf);

	// this should be fairly close to the end of the conversion, so the best approximation of the time
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	uint32_t raw = 0;
	int ret = _interface->read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		_TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		if (_device_type == MS5611_DEVICE) {

			/* Perform MS5611 Caculation */

			_OFF  = ((int64_t)_prom.c2_pressure_offset << 16) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 7);
			_SENS = ((int64_t)_prom.c1_pressure_sens << 15) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 8);

			/* MS5611 temperature compensation */

			if (_TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)_TEMP - 2000);
				int64_t OFF2 = 5 * f >> 1;
				int64_t SENS2 = 5 * f >> 2;

				if (_TEMP < -1500) {

					int64_t f2 = POW2(_TEMP + 1500);
					OFF2 += 7 * f2;
					SENS2 += 11 * f2 >> 1;
				}

				_TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}

		} else if (_device_type == MS5607_DEVICE) {

			/* Perform MS5607 Caculation */

			_OFF  = ((int64_t)_prom.c2_pressure_offset << 17) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 6);
			_SENS = ((int64_t)_prom.c1_pressure_sens << 16) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 7);

			/* MS5607 temperature compensation */

			if (_TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)_TEMP - 2000);
				int64_t OFF2 = 61 * f >> 4;
				int64_t SENS2 = 2 * f;

				if (_TEMP < -1500) {
					int64_t f2 = POW2(_TEMP + 1500);
					OFF2 += 15 * f2;
					SENS2 += 8 * f2;
				}

				_TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}
		}

	} else {

		/* pressure calculation, result in Pa */
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 15;
		_P = P * 0.01f;
		_T = _TEMP * 0.01f;

		/* generate a new report */
		report.temperature = _TEMP / 100.0f;
		report.pressure = P / 100.0f;		/* convert to millibar */

		// report.error_count = perf_event_count(_comms_errors);

		_px4_barometer.update(timestamp_sample, pressure);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void
MS5611::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);

	printf("device:         %s\n", _device_type == MS5611_DEVICE ? "ms5611" : "ms5607");
}
