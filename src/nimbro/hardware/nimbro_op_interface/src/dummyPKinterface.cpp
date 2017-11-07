// NimbRo-OP robot hardware interface (dummy parallel kinematics)
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <nimbro_op_interface/dummyPKinterface.h>
#include <pluginlib/class_list_macros.h>

// Namespaces
using namespace nimbro_op_interface;

PLUGINLIB_EXPORT_CLASS(nimbro_op_interface::DummyPKInterface, robotcontrol::HardwareInterface);
// EOF
