// Tf tools: Filtered transform listener
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Hafez Farazi <farazi@ais.uni-bonn.de>
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef TF_FILTERED_LISTENER_H
#define TF_FILTERED_LISTENER_H

// Includes
#include <tf/tf.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <vector>
#include <string>

/**
* @namespace tf_tools
*
* @brief The namespace for all tf tools.
**/
namespace tf_tools
{
	/**
	* @class FilteredListener
	* 
	* @brief A class similar to `tf::TransformListener`, only with the ability to filter tf messages.
	**/
	class FilteredListener : public tf::Transformer
	{
	public:
		// Constructor/destructor
		explicit FilteredListener(ros::Duration cache_time = ros::Duration(DEFAULT_CACHE_TIME));
		explicit FilteredListener(const ros::NodeHandle& nh, ros::Duration cache_time = ros::Duration(DEFAULT_CACHE_TIME));
		virtual ~FilteredListener();

		// Start function
		void start(bool spin_thread = true);
		void start(const std::vector<std::string>& frames, bool spin_thread = true);

		// Set functions
		void setFutureIgnoreRatio(double ratio) { future_ignore_ratio_ = ratio; } // Note: This is a ratio of the cache time that is allowed into the future for incoming transforms. Set to a negative value to disable this check and accept all transforms.

		// Wait for transform functions
		bool waitForRelativeToNowTransform(const std::string& target_frame, const std::string& source_frame, double shift, double timeout);

	protected:
		// Transformer ok function
		virtual bool ok() const { return ros::ok(); }

	private:
		// Initialisation functions
		void init();
		void initWithThread();

		// Dedicated listener thread function
		void dedicatedListenerThread() { while(isUsingDedicatedThread()) tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01)); }

		// Tf topic handler
		void subscription_callback(const tf2_msgs::TFMessageConstPtr& msg);

		// Helper functions
		static std::string getPrefixParam(ros::NodeHandle& nh);

		// ROS node handle
		ros::NodeHandle node_;

		// Tf message subscription
		ros::Time last_update_ros_time_;
		ros::Subscriber message_subscriber_tf_;

		// Dedicated listener thread
		ros::CallbackQueue tf_message_callback_queue_;
		boost::thread* dedicated_listener_thread_;

		// Future ignore filter
		double future_ignore_ratio_;

		// Tf frame filter
		bool use_filter_;
		std::vector<std::string> filter_frames_;

		// Flag whether using simulation time
		bool use_sim_time;
	};
}

#endif
// EOF
