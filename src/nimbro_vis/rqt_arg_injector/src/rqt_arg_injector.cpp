// Injects ROS arguments into ros::init() using LD_PRELOAD hooking
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>

#include <ros/init.h>

namespace ros
{

typedef void (*orig_init_t)(int &argc, char **argv, const std::string& name, uint32_t options);

void init(int &argc, char **argv, const std::string& name, uint32_t options)
{
	// Lookup the original ROS ros::init() function
	orig_init_t orig_init;
	orig_init = (orig_init_t) dlsym(RTLD_NEXT, "_ZN3ros4initERiPPcRKSsj");
	if(!orig_init)
		orig_init = (orig_init_t) dlsym(RTLD_NEXT, "_ZN3ros4initERiPPcRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEj");
	if(!orig_init)
	{
		ROS_FATAL("rqt_arg_injector: Could not find the original ros::init() function!");
		std::abort();
	}

	// Path to get timewarp and plotter working
	const char* env = getenv("ROS_ARGUMENTS");
	std::vector<char> arg_str;
	std::vector<char*> arguments(argv, argv + argc);

	if(env && strlen(env) != 0)
	{
		ROS_INFO("Injecting rqt with ROS arguments: %s", env);

		arg_str.resize(strlen(env)+1, 0);
		memcpy(&arg_str[0], env, arg_str.size());

		arguments.push_back(&arg_str[0]);

		for(size_t i = 0; i < arg_str.size(); ++i)
		{
			if(arg_str[i] == ' ')
			{
				arg_str[i] = 0;

				if(i < arg_str.size()-1)
					arguments.push_back(&arg_str[i+1]);
			}
		}

		argv = &arguments[0];
		argc = arguments.size();
	}

// 	ROS_INFO("Calling original ros::init()");
	return orig_init(argc, argv, name, options);
}

}
