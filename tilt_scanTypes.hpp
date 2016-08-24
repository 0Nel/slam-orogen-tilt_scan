#ifndef __TILT_SCAN_TYPES_HPP__
#define __TILT_SCAN_TYPES_HPP__

#include <base/Pose.hpp>
#include <stdint.h>

namespace tilt_scan
{
	struct Configuration
	{
		Configuration()
		: mode(CONTINUOUS_SWEEPING),
		max_lines(200),
		sweep_angle_min(-1.0),
		sweep_angle_max(1.0),
		sweep_velocity_up(2.0),
		sweep_velocity_down(0.5)
		{}

		enum Mode {
			CONTINUOUS_SWEEPING,
			TRIGGERED_SWEEPING,
		};

		/** current mode */
		Mode mode;

		/** maximum number of lines until the scan is considered complete */
		int max_lines;

		/** minimum angle value to use when sweeping in rad */
		float sweep_angle_min;

		/** maximum angle to use when sweeping in rad */
		float sweep_angle_max;

		/** sweep velocity up in rad/s */
		float sweep_velocity_up;

		/** sweep velocity down in rad/s. Note the point cloud is only generated while sweeping down */
		float sweep_velocity_down;

		/** name of the servo to sweep */
		std::string sweep_servo_name;
	};

	struct SweepStatus
	{
		SweepStatus() : counter(0), curState(NOT_SWEEPING)
		{};
		
		enum State {
			NOT_SWEEPING,
			REACHED_UP_POSITION,
			SWEEPING_UP,
			SWEEPING_DOWN,
		};
		///counter of the sweeps, wraps at 255
		uint8_t counter;
		
		///current state of sweeping
		State curState;
		
		///name of the sweeping device
		std::string sourceName;
		
		/**
		 * Return true if one ore more sweeps
		 * in respect to the given state werde done.
		 * 
		 * Also returns true, if not sweeping at all.
		 * */
		bool isNextSweep(SweepStatus lastState)
		{
			return NOT_SWEEPING || lastState.counter != counter;
		}
	};

}

#endif
