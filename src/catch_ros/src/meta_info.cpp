// Tell catch_ros about the package we are using it in
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

// This file is compiled in the package using catch_ros!

namespace catch_ros
{
	namespace meta
	{
		const char* packageName()
		{
			return ROS_PACKAGE_NAME;
		}
	}
}
