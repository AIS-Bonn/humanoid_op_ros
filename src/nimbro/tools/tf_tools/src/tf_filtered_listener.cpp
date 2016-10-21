// Tf tools: Filtered transform listener
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Hafez Farazi <farazi@ais.uni-bonn.de>

// Includes
#include <tf_tools/tf_filtered_listener.h>

// Namespaces
using namespace tf_tools;

// Macros
#define HAF_LOG_THROTTLE(rate, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ::ros::WallTime now = ::ros::WallTime::now(); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)
#define HAF_INFO_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define HAF_WARN_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define HAF_ERROR_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define HAF_DEBUG_THROTTLE(rate, ...) HAF_LOG_THROTTLE(rate, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

//
// FilteredListener class
//

// Constructor
FilteredListener::FilteredListener(ros::Duration cache_time)
 : Transformer(true, cache_time)
 , node_("~")
 , dedicated_listener_thread_(NULL)
 , future_ignore_ratio_(-1.0)
 , use_filter_(false)
 , use_sim_time(false)
{
}

// Constructor with explicit node handle
FilteredListener::FilteredListener(const ros::NodeHandle& nh, ros::Duration cache_time)
 : Transformer(true, cache_time)
 , node_(nh)
 , dedicated_listener_thread_(NULL)
 , future_ignore_ratio_(-1.0)
 , use_filter_(false)
 , use_sim_time(false)
{
}

// Destructor
FilteredListener::~FilteredListener()
{
	// Stop and delete the dedicated listener thread
	setUsingDedicatedThread(false);
	if(dedicated_listener_thread_)
	{
		dedicated_listener_thread_->join();
		delete dedicated_listener_thread_;
	}

	// Shut down the tf message subscriber (required)
	message_subscriber_tf_.shutdown();
}

// Start function without any filtering
void FilteredListener::start(bool spin_thread)
{
	// Turn filtering off
	use_filter_ = false;
	filter_frames_.clear();

	// Check whether simulation time is being used
	node_.param("/use_sim_time", use_sim_time, false);

	// Initialise and start the class
	if(spin_thread)
		initWithThread();
	else
		init();
}

// Start function with filtering
void FilteredListener::start(const std::vector<std::string>& frames, bool spin_thread)
{
	// Turn filtering on
	use_filter_ = true;
	filter_frames_ = frames;

	// Check whether simulation time is being used
	node_.param("/use_sim_time", use_sim_time, false);

	// Remove all empty strings from our list of frames
	filter_frames_.erase(std::remove(filter_frames_.begin(), filter_frames_.end(), std::string()), filter_frames_.end());

	// Initialise and start the class
	if(spin_thread)
		initWithThread();
	else
		init();
}

// Initialisation function without dedicated thread
void FilteredListener::init()
{
	// Set that we are not using a dedicated subscription thread
	setUsingDedicatedThread(false);

	// Subscribe to the tf messages
	message_subscriber_tf_ = node_.subscribe<tf2_msgs::TFMessage>("/tf", 100, boost::bind(&FilteredListener::subscription_callback, this, _1));

	// Update the tf prefix inside the transformer base class
	ros::NodeHandle local_nh("~");
	tf_prefix_ = getPrefixParam(local_nh);

	// Initialise the last update ROS time to now
	last_update_ros_time_ = ros::Time::now();
}

// Initialisation function with dedicated thread
void FilteredListener::initWithThread()
{
	// Set that we are using a dedicated subscription thread
	setUsingDedicatedThread(true);

	// Subscribe to the tf messages
	ros::SubscribeOptions ops_tf = ros::SubscribeOptions::create<tf2_msgs::TFMessage>("/tf", 100, boost::bind(&FilteredListener::subscription_callback, this, _1), ros::VoidPtr(), &tf_message_callback_queue_);
	message_subscriber_tf_ = node_.subscribe(ops_tf);

	// Create a dedicated listener thread
	dedicated_listener_thread_ = new boost::thread(boost::bind(&FilteredListener::dedicatedListenerThread, this));

	// Update the tf prefix inside the transformer base class
	ros::NodeHandle local_nh("~");
	tf_prefix_ = getPrefixParam(local_nh);

	// Initialise the last update ROS time to now
	last_update_ros_time_ = ros::Time::now();
}

// Subscription callback for tf messages
void FilteredListener::subscription_callback(const tf2_msgs::TFMessageConstPtr& msg)
{
	// Get the current ROS time
	ros::Time now = ros::Time::now();

	// Clear the tf buffer if a jump back in time is detected
	float ros_dt = (now - last_update_ros_time_).toSec();
	if(ros_dt < 0.0)
	{
		ROS_WARN("Saw a negative time change of %+.3fs => Clearing the tf buffer...", ros_dt);
		clear();
	}

	// Update the last seen ROS time
	last_update_ros_time_ = now;

	// Process the received tf transforms
	for(std::size_t i = 0; i < msg->transforms.size(); ++i)
	{
		// Ignore this transform if it is too far into the future
		const geometry_msgs::TransformStamped& tform = msg->transforms[i];
		ros::Duration inFuture = tform.header.stamp - now;
		if(future_ignore_ratio_ >= 0.0 && inFuture > tf2_buffer_.getCacheLength() * future_ignore_ratio_)
		{
			//ROS_WARN_THROTTLE(0.2, "Ignored transforms that were too far into the future (%+.3fs)", inFuture.toSec());
			continue;
		}

		// Convert the format of the transform
		tf::StampedTransform trans;
		tf::transformStampedMsgToTF(tform, trans);

		// Ignore this transform if it does not concern us
		if(use_filter_)
		{
			if(trans.frame_id_.empty() || trans.child_frame_id_.empty()) continue;
			if(std::find(filter_frames_.begin(), filter_frames_.end(), (trans.frame_id_[0] == '/' ? trans.frame_id_.substr(1) : trans.frame_id_)) == filter_frames_.end()) continue;
			if(std::find(filter_frames_.begin(), filter_frames_.end(), (trans.child_frame_id_[0] == '/' ? trans.child_frame_id_.substr(1) : trans.child_frame_id_)) == filter_frames_.end()) continue;
		}

		// Set the transform inside our transformer base class
		try
		{
			// Prevent instances that the transforms jump back in time sooner than ROS time (only in bags)
			if(use_sim_time)
			{
				float transform_dt = (now - trans.stamp_).toSec();
				if(transform_dt > tf2_buffer_.getCacheLength().toSec())
				{
					HAF_WARN_THROTTLE(1, "Saw a negative time change in trans.stamp %+.3fs", -transform_dt);
					return;
				}
			}

			// Set the required transform
			setTransform(trans);
		}
		catch(tf::TransformException& e) { ROS_ERROR("Failed to set transform from %s to %s: %s", trans.child_frame_id_.c_str(), trans.frame_id_.c_str(), e.what()); }
	}
}

// Retrieve the required tf prefix
std::string FilteredListener::getPrefixParam(ros::NodeHandle& nh)
{
	// Retrieve the path of the most relevant tf prefix ROS parameter on the ROS parameter server
	std::string param; 
	if(!nh.searchParam("tf_prefix", param)) 
		return ""; 

	// Return the value of the tf prefix
	std::string return_val;
	nh.getParam(param, return_val);
	return return_val;
}

// Waits until a transform is available from a source frame to a target frame at a time some constant duration from the current ROS time
bool FilteredListener::waitForRelativeToNowTransform(const std::string& target_frame, const std::string& source_frame, double shift, double timeout)
{
	// Wait for the required transform
	ros::WallTime startWallTime = ros::WallTime::now();
	while((ros::WallTime::now() - startWallTime).toSec() <= timeout)
	{
		try
		{
			if(canTransform(target_frame, source_frame, ros::Time::now() - ros::Duration(shift)))
			{
				return true;
			}
		}
		catch(...)
		{
		}
		usleep(10000);
	}

	// Return failure
	return false;
}

// EOF
