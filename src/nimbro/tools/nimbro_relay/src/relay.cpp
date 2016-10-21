//It is a modified version of https://github.com/ros/ros_comm/
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <cstdio>
#include <nimbro_relay/shape_shifter.h>
#include <nimbro_relay/parse.h>

using std::string;
using std::vector;
#include <ros/console.h>
using namespace nimbro_relay;
namespace nimbro_relay
{
// Strip any leading namespace qualification from a topic (or other kind
// of) ROS name
bool getBaseName(const std::string& full_name, std::string& base_name)
{
	std::string tmp = full_name;
	int i = tmp.rfind('/');
	// Strip off trailing slahes (are those legal anyway?)
	while ((tmp.size() > 0) && (i >= (int) (tmp.size() - 1)))
	{
		tmp = tmp.substr(0, tmp.size() - 1);
		i = tmp.rfind('/');
	}

	if (tmp.size() == 0)
	{
		ROS_ERROR("Base name extracted from \"%s\" is an empty string",
				full_name.c_str());
		return false;
	}

	if (i < 0)
		base_name = tmp;
	else
		base_name = tmp.substr(i + 1, tmp.size() - i - 1);

	return true;
}
}

bool ShapeShifter::uses_old_API_ = false;

ShapeShifter::ShapeShifter() :
		typed(false), msgBuf(NULL), msgBufUsed(0), msgBufAlloc(0)
{
}

ShapeShifter::~ShapeShifter()
{
	if (msgBuf)
		delete[] msgBuf;

	msgBuf = NULL;
	msgBufAlloc = 0;
}

std::string const& ShapeShifter::getDataType() const
{
	return datatype;
}

std::string const& ShapeShifter::getMD5Sum() const
{
	return md5;
}

std::string const& ShapeShifter::getMessageDefinition() const
{
	return msg_def;
}

void ShapeShifter::morph(const std::string& _md5sum,
		const std::string& _datatype, const std::string& _msg_def,
		const std::string& _latching)
{
	md5 = _md5sum;
	datatype = _datatype;
	msg_def = _msg_def;
	latching = _latching;
	typed = md5 != "*";
}

ros::Publisher ShapeShifter::advertise(ros::NodeHandle& nh,
		const std::string& topic, uint32_t queue_size_, bool latch,
		const ros::SubscriberStatusCallback &connect_cb) const
{
	ros::AdvertiseOptions opts(topic, queue_size_, getMD5Sum(), getDataType(),
			getMessageDefinition(), connect_cb);
	opts.latch = latch;

	return nh.advertise(opts);
}

uint32_t ShapeShifter::size() const
{
	return msgBufUsed;
}

ros::NodeHandle *g_node = NULL;
bool g_advertised = false;
string g_input_topic;
string g_output_topic;
ros::Publisher g_pub;
ros::Subscriber* g_sub;
bool g_lazy;
int rrate;
ros::TransportHints g_th;

void conn_cb(const ros::SingleSubscriberPublisher&);
void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event);

void subscribe()
{
	g_sub = new ros::Subscriber(
			g_node->subscribe(g_input_topic, 10, &in_cb, g_th));
}

void conn_cb(const ros::SingleSubscriberPublisher&)
{
	// If we're in lazy subscribe mode, and the first subscriber just
	// connected, then subscribe, #3389.
	if (g_lazy && !g_sub)
	{
		ROS_DEBUG("lazy mode; resubscribing");
		subscribe();
	}
}

ros::WallTimer m_timer;
bool fisrtMsgArrived;
ros::Time lastCallbackTime;
ros::MessageEvent<ShapeShifter> last_msg_event;
void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event)
{
	fisrtMsgArrived = true;
	last_msg_event = msg_event;
	boost::shared_ptr<ShapeShifter const> const &msg =
			msg_event.getConstMessage();
	boost::shared_ptr<const ros::M_string> const& connection_header =
			msg_event.getConnectionHeaderPtr();

	if (!g_advertised)
	{
		// If the input topic is latched, make the output topic latched, #3385.
		bool latch = false;
		if (connection_header)
		{
			ros::M_string::const_iterator it = connection_header->find(
					"latching");
			if ((it != connection_header->end()) && (it->second == "1"))
			{
				ROS_DEBUG(
						"input topic is latched; latching output topic to match");
				latch = true;
			}
		}
		g_pub = msg->advertise(*g_node, g_output_topic, 10, latch, conn_cb);
		g_advertised = true;
		ROS_INFO("advertised as %s\n", g_output_topic.c_str());
	}
	// If we're in lazy subscribe mode, and nobody's listening,
	// then unsubscribe, #3389.
	if (g_lazy && !g_pub.getNumSubscribers())
	{
		ROS_DEBUG("lazy mode; unsubscribing");
		delete g_sub;
		g_sub = NULL;
	}
	else
		g_pub.publish(msg);
}

void timerCallback(const ros::WallTimerEvent&)
{
	if (fisrtMsgArrived && ros::Time::now() <= lastCallbackTime)
	{
		in_cb(last_msg_event);
	}
	lastCallbackTime = ros::Time::now();
}


int main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf("\nusage: relay IN_TOPIC [OUT_TOPIC]\n\n");
		return 1;
	}
	std::string topic_name;
	if (!getBaseName(string(argv[1]), topic_name))
		return 1;
	ros::init(argc, argv, topic_name + string("_relay"),
			ros::init_options::AnonymousName);
	if (argc == 2)
		g_output_topic = string(argv[1]) + string("_relay");
	else
		// argc == 3
		g_output_topic = string(argv[2]);
	g_input_topic = string(argv[1]);
	ros::NodeHandle n;
	g_node = &n;

	ros::NodeHandle pnh("~");

	lastCallbackTime = ros::Time::now();
	fisrtMsgArrived = false;
	bool unreliable = false;
	pnh.getParam("unreliable", unreliable);
	pnh.getParam("lazy", g_lazy);
	pnh.param("rrate", rrate, 30);
	if (unreliable)
		g_th.unreliable().reliable(); // Prefers unreliable, but will accept reliable.

	subscribe();

	m_timer = pnh.createWallTimer(ros::WallDuration(1.0 / rrate),
			timerCallback);

	ros::spin();

	m_timer.stop();
	return 0;
}

