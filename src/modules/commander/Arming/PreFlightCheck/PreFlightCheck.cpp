/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file PreFlightCheck.cpp
 */

#include <PreFlightCheck.hpp>

#include <ArmAuthorization.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/vehicle_command_ack.h>

bool PreFlightCheck::preArmCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_flags_s &status_flags,
				 const safety_s &safety, const uint8_t arm_requirements)
{
	bool prearm_ok = true;
	char const *feedback_string = ""; // make sure it's never nullptr

	// USB not connected
	if (!status_flags.circuit_breaker_engaged_usb_check && status_flags.usb_connected) {
		feedback_string = "Disconnect USB";
		prearm_ok = false;
	}

	// Battery and system power status
	if (!status_flags.circuit_breaker_engaged_power_check) {

		// Fail transition if power is not good
		if (!status_flags.condition_power_input_valid) {
			feedback_string = "Connect power module";
			prearm_ok = false;
		}

		// main battery level
		if (!status_flags.condition_battery_healthy) {
			feedback_string = "Check battery";
			prearm_ok = false;
		}
	}

	// Arm Requirements: mission
	if (arm_requirements & ARM_REQ_MISSION_BIT) {

		if (!status_flags.condition_auto_mission_available) {
			feedback_string = "Upload valid mission";
			prearm_ok = false;
		}

		if (!status_flags.condition_global_position_valid) {
			feedback_string = "Mission requires global position";
			prearm_ok = false;
		}
	}

	// Arm Requirements: global position
	if (arm_requirements & ARM_REQ_GPS_BIT) {
		if (!status_flags.condition_global_position_valid) {
			feedback_string = "Global position required";
			prearm_ok = false;
		}
	}

	// Safety button
	if (safety.safety_switch_available && !safety.safety_off) {
		feedback_string = "Press safety button";
		prearm_ok = false;
	}

	if (status_flags.avoidance_system_required && !status_flags.avoidance_system_valid) {
		feedback_string = "Check avoidance system";
		prearm_ok = false;

	}

	if (status_flags.condition_escs_error && (arm_requirements & ARM_REQ_ESCS_CHECK_BIT)) {
		feedback_string = "Check motor controllers";
		prearm_ok = false;
	}

	// Arm Requirements: authorization
	// check last, and only if everything else has passed
	if (arm_requirements & ARM_REQ_ARM_AUTH_BIT) {
		if (arm_auth_check() != vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED) {
			feedback_string = "Request authorization";
			prearm_ok = false;
		}
	}

	if (!prearm_ok) {
		mavlink_log_critical(mavlink_log_pub, "Not arming: %s", feedback_string);
	}

	return prearm_ok;
}
