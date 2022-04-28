#ifndef TILT_SCAN_TASK_TASK_HPP
#define TILT_SCAN_TASK_TASK_HPP

#include <tilt_scan/TaskBase.hpp>
#include <tilt_scanTypes.hpp>

#include <base/commands/Joints.hpp>
#include <base/samples/Pointcloud.hpp>

namespace tilt_scan
{
	class Task : public TaskBase
	{
	friend class TaskBase;
	protected:

		// Callbacks
		virtual void scan_samplesTransformerCallback(const base::Time &ts, const base::samples::LaserScan &scan);
		virtual void pointcloud_samplesTransformerCallback(const base::Time &ts, const base::samples::Pointcloud &pointcloud);
		virtual void trigger_sweep();

		// Internal methods
		void checkTiltStatus();
		void sendPointcloud();

		// Members
		base::Time mLastScanTime;
		bool mTrigger;

		base::commands::Joints mTiltUpCommand;
		base::commands::Joints mTiltDownCommand;
		base::samples::Pointcloud mPointcloud;

		Configuration mConfiguration;
		SweepStatus mSweepStatus;

	public:
		Task(std::string const& name = "tilt_scan::Task");
		Task(std::string const& name, RTT::ExecutionEngine* engine);
		~Task();

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

