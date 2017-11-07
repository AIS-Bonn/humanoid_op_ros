// Model of the robot
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
// Comment: The RobotModel class amounts to a central storage location for information pertaining to the robot,
//          and forms an information gateway between the hardware interface and the remaining code.

#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/model/singlesupportmodel.h>
#include <robotcontrol/model/joint.h>
#include <robotcontrol/robotcontrolvis.h>

#include <rbdl/rbdl_parser.h>
#include <rbdl/treestream.h>
#include <rbdl/Dynamics.h>

#include <boost/make_shared.hpp>

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <robotcontrol/State.h>
#include <robotcontrol/RobotState.h>
#include <robotcontrol/RobotHeading.h>

// Debugging defines
#define DUMP_TREES      0

// Defines
#define M_2PI           6.2831853071795864769
#define PROJ_MAG_SCALE  3.0
#define GYRO_VEC_SCALE  1.0
#define ACC_VEC_SCALE   -0.1
#define MAG_VEC_SCALE   1.0

namespace Math = RigidBodyDynamics::Math;

namespace robotcontrol
{

inline void rbdlToTF(const Math::SpatialTransform& X, tf::Transform* t)
{
	t->setOrigin(tf::Vector3(X.r.x(), X.r.y(), X.r.z()));
	t->setBasis(tf::Matrix3x3(
		X.E(0, 0), X.E(1, 0), X.E(2, 0),
		X.E(0, 1), X.E(1, 1), X.E(2, 1),
		X.E(0, 2), X.E(1, 2), X.E(2, 2)
	));
}

// Constants
const std::string RobotModel::RESOURCE_PATH = "robotmodel/";
const std::string RobotModel::CONFIG_PARAM_PATH = "/robotcontrol/robotModel/";

// RobotModel contructor
RobotModel::RobotModel(RobotControl* robotControl)
 : m_robotControl(robotControl)
 , m_warnZeroTotalSuppCoeff(CONFIG_PARAM_PATH + "warnZeroTotalSuppCoeff", true)
 , m_useSupportInformation(CONFIG_PARAM_PATH + "useSupportInfo", true)
 , m_useFeedbackPos(CONFIG_PARAM_PATH + "useFeedbackPos", false)
 , m_egoRotIncludesHeading(CONFIG_PARAM_PATH + "egoRotIncludesHeading", false)
 , m_headingIsFusedYaw(CONFIG_PARAM_PATH + "headingIsFusedYaw", true)
 , m_PM(PM_COUNT, RESOURCE_PATH) // Note: This is the first call to advertise for /plot for this node, and so the queue length passed here is the one that counts, so make it big enough for simultaneous plotting from all motion modules, robotcontrol and robotmodel!
 , m_plotRobotModelData(CONFIG_PARAM_PATH + "plotData", false)
 , m_currentState(0)
 , m_relaxed(false)
 , m_relaxedWasSet(false)
 , m_timerDuration(DEFAULT_TIMER_DURATION) // Note: This default value should be overwritten by RobotControl on init, so in theory this can be arbitrary...
{
	// Retrieve a node handle
	ros::NodeHandle nh("~");
	ros::NodeHandle nhs;

	// Retrieve whether to publish the ego_rot frame
	nh.param("publish_ego_rot", m_publishEgoRot, true);

	// Advertise topics to publish on
	m_pub_state = nh.advertise<robotcontrol::State>("state", 1, true);
	m_pub_robotState = nhs.advertise<robotcontrol::RobotState>(RESOURCE_PATH + "robot_state", 1);
	m_pub_robotHeading = nhs.advertise<robotcontrol::RobotHeading>(RESOURCE_PATH + "robot_heading", 1);

	// Initialise the robot state
	setRobotOrientation(Eigen::Quaterniond::Identity());     // Note: This internally also initialises the remaining orientation-based robot state variables (i.e. the internal Euler and fused angle representations)
	setRobotAngularVelocity(Eigen::Vector3d::Zero());        // Note: By default the robot is simply at rest
	setAccelerationVector(Eigen::Vector3d(0.0, 0.0, 9.81));  // Note: Gravity pointing downwards looks like an upwards acceleration of the robot
	setMagneticFieldVector(Eigen::Vector3d(0.5, 0.0, 0.0));  // Note: A 'reasonable' default value of 0.50 gauss in line with the positive x-axis
	setTemperature(35.0);                                    // Note: Robots usually heat up above standard room temperature
	setVoltage(16.0);                                        // Note: The approximate value of a slightly discharged 4-cell LiPo battery
	processReadData();

	// Configure the plot manager
	configurePlotManager();

	// Register an initial state and set it as the current state
	setState(registerState("unknown")); // Note: This should get overwritten in RobotControl::RobotControl()
}

// RobotModel destructor
RobotModel::~RobotModel()
{
}

void RobotModel::setModel(const boost::shared_ptr< urdf::Model >& model)
{
	m_model = model;
}

void RobotModel::addJoint(const boost::shared_ptr<Joint>& joint)
{
	m_joints.push_back(joint);
}

void RobotModel::initTrees()
{
	// Retrieve the root URDF link
	boost::shared_ptr<const urdf::Link> root = m_model->getRoot();

	// Create single support models for all tip links and the root link (Note: The root link will always be the first model in the list)
	m_models.clear();
	if(root->child_links.size() != 0)
		createSupportModelFromURDFLink(root); // Explicitly add a single support model for the root link if it isn't a tip link (usually the case)
	doInit(root);

	// Create a reference to the root link support model
	if(m_models.empty()) ROS_ERROR("No single support model was created for the root link, this shouldn't happen!");
	m_rootSupportModel = m_models.at(0); // Note: If m_models is empty (should never happen) then this should throw a std::out_of_range exception

	// Create plots for the tip link support coefficients
	m_PM.setSize(PM_COUNT + m_models.size());
	for(size_t i = 0; i < m_models.size(); i++)
		m_PM.setName(PM_SUPPORT + i, "SupportCoeff/" + m_models[i]->link()->name);
	if(!m_PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");

	// Calculate the total mass of the model
	m_mass = 0.0;
	std::vector<boost::shared_ptr<urdf::Link> > links;
	m_model->getLinks(links);
	for(size_t i = 0; i < links.size(); ++i)
	{
		if(links[i]->inertial)
			m_mass += links[i]->inertial->mass;
	}

	// Setup tf transform for /ego_rot
	if(m_publishEgoRot)
	{
		tf::StampedTransform t_imu;
		t_imu.frame_id_ = "/ego_rot";
		t_imu.child_frame_id_ = m_rootSupportModel->GetBodyName(0);
		t_imu.setIdentity();
		m_tf_buf.push_back(t_imu);
	}
	else // This keeps index-based access intact (loops starting at m_tf_buf[1])
	{
		tf::StampedTransform t_imu;
		t_imu.frame_id_ = "/dummy0";
		t_imu.child_frame_id_ = "dummy1";
		t_imu.setIdentity();
		m_tf_buf.push_back(t_imu);
	}

	// Setup tf transforms for each link
	for(size_t i = 1; i < m_rootSupportModel->mBodies.size(); ++i)
	{
		tf::StampedTransform t;
		t.frame_id_ = m_rootSupportModel->GetBodyName(m_rootSupportModel->lambda[i]);
		t.child_frame_id_ = m_rootSupportModel->GetBodyName(i);
		t.setIdentity();
		m_tf_buf.push_back(t);
	}

	// Setup fixed transforms
	for(size_t i = 1; i < m_rootSupportModel->mFixedBodies.size(); ++i)
	{
		tf::StampedTransform t;
		const RigidBodyDynamics::FixedBody& body = m_rootSupportModel->mFixedBodies[i];
		t.frame_id_ = m_rootSupportModel->GetBodyName(body.mMovableParent);
		t.child_frame_id_ = m_rootSupportModel->GetBodyName(m_rootSupportModel->fixed_body_discriminator + i);
		rbdlToTF(body.mParentTransform, &t);
		m_tf_fixed_buf.push_back(t);
	}
}

/**
 * This function calls itself recursively on every link in the kinematic tree.
 **/
void RobotModel::doInit(const boost::shared_ptr<const urdf::Link>& link)
{
	// If this is a tip link (no children), create a support model for it
	if(link->child_links.size() == 0)
		createSupportModelFromURDFLink(link);
	else
	{
		// Recursively call doInit() for all child links
		for(size_t i = 0; i < link->child_links.size(); ++i)
			doInit(link->child_links[i]);
	}
}

int RobotModel::jointIndex(const std::string& name)
{
	for(size_t i = 0; i < m_joints.size(); ++i)
	{
		if(m_joints[i]->name == name)
			return i;
	}

	std::stringstream ss;
	ss << "RobotModel::jointIndex(): Unknown joint '" << name << "'";
	throw std::logic_error(ss.str());
}

// Create and add a support model that uses the specified URDF link as its root
void RobotModel::createSupportModelFromURDFLink(const boost::shared_ptr<const urdf::Link>& supportLink)
{
	// Create and initialise a single support model that uses the given URDF link as root
	boost::shared_ptr<SingleSupportModel> model = boost::make_shared<SingleSupportModel>(this, supportLink);
	model->initFrom(*m_model, supportLink->name);
	
	// Be doubly sure that the initial coefficient is zero
	model->setCoeff(0.0);

	// Save the new single support model in our list
	m_models.push_back(model);

	// Print debug output if required
#if DUMP_TREES
	std::cout << std::endl << "SingleSupportModel RBDL Tree: " << supportLink->name << std::endl;
	TreeStream stream(&std::cout);
	stream << *model;
	std::cout << std::endl;
#endif
}

/**
 * @param name URDF joint name
 * @return Pointer to Joint, Null pointer if the joint does not exist
 **/
Joint::Ptr RobotModel::getJoint(const std::string& name)
{
	for(size_t i = 0; i < m_joints.size(); i++)
	{
		if(m_joints[i]->name == name)
			return m_joints[i];
	}

	return Joint::Ptr();
}

void RobotModel::newCommands()
{
	for(size_t i = 0; i < m_joints.size(); i++)
		m_joints[i]->lastCmd = m_joints[i]->cmd;
	m_relaxedWasSet = false;
}

/**
 * Do the dynamics calculations.
 **/
void RobotModel::doInverseDynamics()
{
	// Reset the model torques to zero
	for(size_t i = 0; i < m_joints.size(); ++i)
		m_joints[i]->feedback.modelTorque = 0.0;

	// Decide on a kinematic position data source
	SingleSupportModel::DataSource source = (m_useFeedbackPos() ? SingleSupportModel::MeasurementData : SingleSupportModel::CommandData);

	// Calculate the inverse dynamics as required
	if(m_useSupportInformation())
	{
		// Calculate the torques required for producing the desired velocities and accelerations (always calculated using the root link support model)
		m_rootSupportModel->doInverseDynamics(source, false, true, false);

		// Calculate the torques required to compensate for gravity (calculated using support coefficients and weighting across all single support models)
		double total = 0.0;
		for(size_t i = 0; i < m_models.size(); ++i)
			total += m_models[i]->doInverseDynamics(source, true, false, true);
		if(total == 0.0 && m_warnZeroTotalSuppCoeff())
			ROS_WARN_THROTTLE(1.0, "All support coefficients are currently zero");
	}
	else
	{
		// Calculate the torques required for the velocities, accelerations and gravity all together using only the root link support model
		m_rootSupportModel->doInverseDynamics(source, false, true, true);
	}

	// Calculate the CoM position (doesn't really belong here)
	m_rootSupportModel->computeCoM();
}

/**
 * Set a support coefficient. The coefficients are used as weights in the mixing
 * calculations (see SingleSupportModel::setCoefficient()).
 *
 * @param link The tip link used as base of the support model
 *   (e.g. right_foot_link)
 * @param coeff Support coefficient (positive or zero)
 **/
void RobotModel::setSupportCoefficient(const boost::shared_ptr<const urdf::Link>& link, double coeff)
{
	// Retrieve the single support model that corresponds to the given URDF link
	boost::shared_ptr<SingleSupportModel> model;
	for(size_t i = 0; i < m_models.size(); ++i)
	{
		if(m_models[i]->link() == link)
		{
			model = m_models[i];
			break;
		}
	}
	
	// Ensure that we found the model
	if(!model)
	{
		if(link)
			ROS_ERROR_THROTTLE(0.3, "setSupportCoefficient called with non-tip link '%s' (URDF)", link->name.c_str());
		else
			ROS_ERROR_THROTTLE(0.3, "setSupportCoefficient called with a null pointer non-tip link (URDF)");
		return;
	}
	
	// Set the required support coefficient
	setSupportCoefficient(model, coeff);
}

void RobotModel::setSupportCoefficient(const std::string& linkName, double coeff)
{
	// Retrieve the single support model that corresponds to the given link name
	boost::shared_ptr<SingleSupportModel> model;
	for(size_t i = 0; i < m_models.size(); ++i)
	{
		if(m_models[i]->link()->name == linkName)
		{
			model = m_models[i];
			break;
		}
	}
	
	// Ensure that we found the model
	if(!model)
	{
		ROS_ERROR_THROTTLE(0.3, "setSupportCoefficient called with non-tip link '%s' (string)", linkName.c_str());
		return;
	}
	
	// Set the required support coefficient
	setSupportCoefficient(model, coeff);
}

// For internal use only!! (we assume that linkModel references something in our m_models)
void RobotModel::setSupportCoefficient(const boost::shared_ptr<SingleSupportModel>& linkModel, double coeff)
{
	// Ensure that we have a valid model
	if(!linkModel)
	{
		ROS_ERROR("setSupportCoefficient called with an invalid reference to a SingleSupportModel");
		return;
	}

	// Set the model coefficient
	linkModel->setCoeff(coeff);

	// We need to normalize the coefficients to a sum of 1
	double coeffSum = 0.0;
	for(size_t i = 0; i < m_models.size(); ++i)
		coeffSum += m_models[i]->coeff();
	for(size_t i = 0; i < m_models.size(); ++i)
		m_models[i]->normCoeff(coeffSum);
}

void RobotModel::resetSupport()
{
	// Reset all the support coefficients to zero
	for(size_t i = 0; i < m_models.size(); ++i)
	{
		const boost::shared_ptr<SingleSupportModel>& model = m_models[i];
		model->setCoeff(0.0);
		model->normCoeff(0.0);
	}
}

/**
 * @param link_name Name of the tip link used as base of the support model (e.g. right_foot_link)
 **/
boost::shared_ptr<SingleSupportModel> RobotModel::supportModel(const std::string& link_name)
{
	for(size_t i = 0; i < m_models.size(); ++i)
	{
		if(m_models[i]->link()->name == link_name)
			return m_models[i];
	}

	return boost::shared_ptr<SingleSupportModel>();
}

// Update the robot orientation state parameters based on the given quaternion
void RobotModel::setRobotOrientation(const Eigen::Quaterniond& quaternion, const ros::Time& timestamp)
{
	// Pass the work onto the other function overload
	setRobotOrientation(quaternion, quaternion, timestamp);
}

// Update the robot orientation state parameters based on the given quaternions
void RobotModel::setRobotOrientation(const Eigen::Quaterniond& quaternion, const Eigen::Quaterniond& quaternionPR, const ros::Time& timestamp)
{
	// Save the timestamp
	m_robotOrientationTime = timestamp;

	// PART 1: Robot orientation
	
	// Update the robot orientation
	m_robotOrientation = quaternion.normalized();

	// Local aliases for the robot orientation quaternion components
	double w = m_robotOrientation.w();
	double x = m_robotOrientation.x();
	double y = m_robotOrientation.y();
	double z = m_robotOrientation.z();

	// Precalculate terms of the calculation
	double ysq = y*y;
	double hR11 = 0.5 - (ysq + z*z); // Half of the (1,1) entry (xGx) in the rotation matrix corresponding to the given quaternion
	double hR21 = w*z + x*y;         // Half of the (2,1) entry (yGx) in the rotation matrix corresponding to the given quaternion
	double hR31 = x*z - w*y;         // Half of the (3,1) entry (zGx) in the rotation matrix corresponding to the given quaternion
	double hR32 = w*x + y*z;         // Half of the (3,2) entry (zGy) in the rotation matrix corresponding to the given quaternion
	double hR33 = 0.5 - (x*x + ysq); // Half of the (3,3) entry (zGz) in the rotation matrix corresponding to the given quaternion

	// Calculate the ZYX Euler angles of the robot orientation
	double sth = -2.0 * hR31;
	sth = (sth >= 1.0 ? 1.0 : (sth <= -1.0 ? -1.0 : sth)); // Note: This should never trim off more than perhaps a few eps
	m_robotEYaw   = atan2(hR21,hR11);
	m_robotEPitch = asin(sth);
	m_robotERoll  = atan2(hR32,hR33);

	// Calculate the robot orientation without its ZYX Euler yaw component
	double hEYaw = 0.5 * m_robotEYaw;
	double hcEYaw = cos(hEYaw);
	double hsEYaw = sin(hEYaw); // The quaternion corresponding to the EYaw component of the given quaternion is now (hcEYaw,0,0,hsEYaw)
	m_robotOrientationNoEYaw.w() = w*hcEYaw + z*hsEYaw;
	m_robotOrientationNoEYaw.x() = x*hcEYaw + y*hsEYaw; // Evaluate m_robotOrientationNoEYaw as the quaternion
	m_robotOrientationNoEYaw.y() = y*hcEYaw - x*hsEYaw; // product (hcEYaw,0,0,-hsEYaw)*(w,x,y,z)
	m_robotOrientationNoEYaw.z() = z*hcEYaw - w*hsEYaw;

	// Calculate the fused angles of the robot orientation
	m_robotFYaw = 2.0*atan2(z,w);                  // Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
	if(m_robotFYaw >   M_PI) m_robotFYaw -= M_2PI; // m_robotFYaw is now in [-2*pi,pi]
	if(m_robotFYaw <= -M_PI) m_robotFYaw += M_2PI; // m_robotFYaw is now in (-pi,pi]
	m_robotFPitch = m_robotEPitch;                 // Fused pitch is equivalent to ZYX Euler pitch
	double sphi = 2.0 * hR32;
	sphi = (sphi >= 1.0 ? 1.0 : (sphi <= -1.0 ? -1.0 : sphi)); // Note: This should never trim off more than perhaps a few eps
	m_robotFRoll = asin(sphi);                     // Fused roll is the asin of the (3,2) rotation matrix entry (zGy)
	m_robotFHemi = (hR33 >= 0.0);                  // The orientation hemisphere depends only on the sign of the (3,3) rotation matrix entry (zGz)

	// Calculate the robot orientation without its fused yaw component
	double hFYaw = 0.5 * m_robotFYaw;
	double hcFYaw = cos(hFYaw);
	double hsFYaw = sin(hFYaw); // The quaternion corresponding to the fused yaw component of the given quaternion is now (hcFYaw,0,0,hsFYaw)
	m_robotOrientationNoFYaw.w() = w*hcFYaw + z*hsFYaw;
	m_robotOrientationNoFYaw.x() = x*hcFYaw + y*hsFYaw; // Evaluate m_robotOrientationNoFYaw as the quaternion
	m_robotOrientationNoFYaw.y() = y*hcFYaw - x*hsFYaw; // product (hcFYaw,0,0,-hsFYaw)*(w,x,y,z)
	m_robotOrientationNoFYaw.z() = z*hcFYaw - w*hsFYaw;

	// Calculate the robot heading parameters
	if(m_headingIsFusedYaw())
	{
		m_robotHeading = m_robotFYaw;
		m_robotOrientationNoHeading = m_robotOrientationNoFYaw;
	}
	else
	{
		m_robotHeading = m_robotEYaw;
		m_robotOrientationNoHeading = m_robotOrientationNoEYaw;
	}
	
	// PART 2: Robot fused pitch/roll orientation
	
	// Check the common case that two identical quaternions were passed
	if(quaternionPR.w() == quaternion.w() && quaternionPR.x() == quaternion.x() && quaternionPR.y() == quaternion.y() && quaternionPR.z() == quaternion.z()) // Eigen::Quaternion doesn't give a better way to check exact equality...
	{
		// Just copy out the already calculated orientation parameters to the PR member variables
		m_robotOrientationPR = m_robotOrientationNoFYaw;
		m_robotFYawPR = m_robotFYaw;
		m_robotFPitchPR = m_robotFPitch;
		m_robotFRollPR = m_robotFRoll;
		m_robotFHemiPR = m_robotFHemi;
	}
	else
	{
		// Normalise the provided quaternion
		Eigen::Quaterniond qPR = quaternionPR.normalized();
		
		// Local aliases for the robot orientation quaternion components
		double w = qPR.w();
		double x = qPR.x();
		double y = qPR.y();
		double z = qPR.z();
		
		// Precalculate terms of the calculation
		double hR31 = x*z - w*y;         // Half of the (3,1) entry (zGx) in the rotation matrix corresponding to the given quaternion
		double hR32 = w*x + y*z;         // Half of the (3,2) entry (zGy) in the rotation matrix corresponding to the given quaternion
		double hR33 = 0.5 - (x*x + y*y); // Half of the (3,3) entry (zGz) in the rotation matrix corresponding to the given quaternion

		// Calculate the fused angles of the orientation
		m_robotFYawPR = 2.0*atan2(z,w);                    // Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
		if(m_robotFYawPR >   M_PI) m_robotFYawPR -= M_2PI; // m_robotFYawPR is now in [-2*pi,pi]
		if(m_robotFYawPR <= -M_PI) m_robotFYawPR += M_2PI; // m_robotFYawPR is now in (-pi,pi]
		double sth = -2.0 * hR31;
		double sphi = 2.0 * hR32;
		sth = (sth >= 1.0 ? 1.0 : (sth <= -1.0 ? -1.0 : sth));     // Note: This should never trim off more than perhaps a few eps
		sphi = (sphi >= 1.0 ? 1.0 : (sphi <= -1.0 ? -1.0 : sphi)); // Note: This should never trim off more than perhaps a few eps
		m_robotFPitchPR = asin(sth);    // Fused pitch is the negative asin of the (3,1) rotation matrix entry (zGx)
		m_robotFRollPR = asin(sphi);    // Fused roll is the asin of the (3,2) rotation matrix entry (zGy)
		m_robotFHemiPR = (hR33 >= 0.0); // The orientation hemisphere depends only on the sign of the (3,3) rotation matrix entry (zGz)
		
		// Calculate the robot fused pitch/roll orientation without its fused yaw component
		double hFYaw = 0.5 * m_robotFYawPR;
		double hcFYaw = cos(hFYaw);
		double hsFYaw = sin(hFYaw); // The quaternion corresponding to the fused yaw component of the given quaternion is now (hcFYaw,0,0,hsFYaw)
		m_robotOrientationPR.w() = w*hcFYaw + z*hsFYaw;
		m_robotOrientationPR.x() = x*hcFYaw + y*hsFYaw; // Evaluate m_robotOrientationPR as the quaternion
		m_robotOrientationPR.y() = y*hcFYaw - x*hsFYaw; // product (hcFYaw,0,0,-hsFYaw)*(w,x,y,z)
		m_robotOrientationPR.z() = z*hcFYaw - w*hsFYaw;
	}
}

/**
 * @return ZMP calculated using the SingleSupportModel with the highest
 *   coefficient
 **/
tf::Stamped<tf::Vector3> RobotModel::zeroMomentPoint() const
{
	int idx_max = 0;
	for(size_t i = 1; i < m_models.size(); ++i)
	{
		if(m_models[i]->coeff() > m_models[idx_max]->coeff())
			idx_max = i;
	}
	RigidBodyDynamics::Math::Vector3d zmp = m_models[idx_max]->zeroMomentPoint();

	return tf::Stamped<tf::Vector3>(
		tf::Vector3(zmp.x(), zmp.y(), zmp.z()),
		ros::Time::now(),
		"/" + m_models[idx_max]->link()->name
	);
}

Eigen::Vector3d RobotModel::centerOfMass() const
{
	return m_rootSupportModel->centerOfMass();
}

void RobotModel::visualizeData(RCMarkerMan* markers)
{
	// Save the current ROS time
	ros::Time now = ros::Time::now();

	// Update the CoM visualisation markers
	Eigen::Vector3d com = centerOfMass();
	markers->CoM.update(com.x(), com.y(), com.z());
	
	// Update the gyroscope and accelerometer visualisation markers
	markers->GyroVec.update(GYRO_VEC_SCALE*m_robotAngularVelocity.x(), GYRO_VEC_SCALE*m_robotAngularVelocity.y(), GYRO_VEC_SCALE*m_robotAngularVelocity.z());
	markers->AccVec.update(ACC_VEC_SCALE*m_accelerationVector.x(), ACC_VEC_SCALE*m_accelerationVector.y(), ACC_VEC_SCALE*m_accelerationVector.z());
	
	// Update the magnetometer visualisation marker
	markers->MagVec.update(MAG_VEC_SCALE*m_magneticFieldVector.x(), MAG_VEC_SCALE*m_magneticFieldVector.y(), MAG_VEC_SCALE*m_magneticFieldVector.z());
	Eigen::Vector3d projmag = PROJ_MAG_SCALE * (m_robotOrientation * m_magneticFieldVector);
	projmag.z() = 0.0;
	projmag = m_robotOrientation.inverse() * projmag;
	markers->MagVec2D.update(projmag.x(), projmag.y(), projmag.z());

	// Publish plotter data
	if(m_plotRobotModelData() || m_PM.haveEvents())
	{
		// Set the plotting time stamp
		m_PM.setTimestamp(now);

		// Plot the robot model state
		m_PM.plotScalar(m_robotAngularVelocity.x(), PM_ANGVEL_X);
		m_PM.plotScalar(m_robotAngularVelocity.y(), PM_ANGVEL_Y);
		m_PM.plotScalar(m_robotAngularVelocity.z(), PM_ANGVEL_Z);
		m_PM.plotScalar(m_robotAngularVelocity.norm(), PM_ANGVEL_N);
		m_PM.plotScalar(m_accelerationVector.x(), PM_ACCVEC_X);
		m_PM.plotScalar(m_accelerationVector.y(), PM_ACCVEC_Y);
		m_PM.plotScalar(m_accelerationVector.z(), PM_ACCVEC_Z);
		m_PM.plotScalar(m_accelerationVector.norm(), PM_ACCVEC_N);
		m_PM.plotScalar(m_magneticFieldVector.x(), PM_MAGVEC_X);
		m_PM.plotScalar(m_magneticFieldVector.y(), PM_MAGVEC_Y);
		m_PM.plotScalar(m_magneticFieldVector.z(), PM_MAGVEC_Z);
		m_PM.plotScalar(m_magneticFieldVector.norm(), PM_MAGVEC_N);
		m_PM.plotScalar(m_robotEYaw, PM_EULER_YAW);
		m_PM.plotScalar(m_robotEPitch, PM_EULER_PITCH);
		m_PM.plotScalar(m_robotERoll, PM_EULER_ROLL);
		m_PM.plotScalar(m_robotFYaw, PM_FUSED_YAW);
		m_PM.plotScalar(m_robotFPitch, PM_FUSED_PITCH);
		m_PM.plotScalar(m_robotFRoll, PM_FUSED_ROLL);
		m_PM.plotScalar((m_robotFHemi ? 1.0 : -1.0), PM_FUSED_HEMI);
		m_PM.plotScalar(m_robotDFYaw, PM_FUSED_DYAW);
		m_PM.plotScalar(m_robotDFPitch, PM_FUSED_DPITCH);
		m_PM.plotScalar(m_robotDFRoll, PM_FUSED_DROLL);
		m_PM.plotScalar(m_robotHeading, PM_HEADING);
		m_PM.plotScalar(m_robotFYawPR, PM_FUSED_YAW_PR);
		m_PM.plotScalar(m_robotFPitchPR, PM_FUSED_PITCH_PR);
		m_PM.plotScalar(m_robotFRollPR, PM_FUSED_ROLL_PR);
		m_PM.plotScalar((m_robotFHemiPR ? 1.0 : -1.0), PM_FUSED_HEMI_PR);
		m_PM.plotScalar(m_robotDFYawPR, PM_FUSED_DYAW_PR);
		m_PM.plotScalar(m_robotDFPitchPR, PM_FUSED_DPITCH_PR);
		m_PM.plotScalar(m_robotDFRollPR, PM_FUSED_DROLL_PR);
		m_PM.plotScalar(m_temperature, PM_TEMPERATURE);
		m_PM.plotScalar(m_voltage, PM_VOLTAGE);
		m_PM.plotScalar(m_relaxed, PM_RELAXEDMODEL);
		m_PM.plotScalar(m_relaxedWasSet, PM_RELAXEDWRITTEN);
		m_PM.plotScalar(m_currentState.index() * 0.1, PM_ROBOT_STATE);

		// Plot support coefficients
		for(size_t i = 0; i < m_models.size(); i++)
			m_PM.plotScalar(m_models[i]->coeff(), PM_SUPPORT + i);

		// Publish and clear the plot data
		m_PM.publish();
		m_PM.clear(now);
	}
}

void RobotModel::publishTF(bool useMeasurement)
{
	//
	// Publish TF transforms
	//

	ros::Time now = ros::Time::now();

	if(useMeasurement)
		m_rootSupportModel->updateRBDLJointPos(SingleSupportModel::MeasurementData);
	else
		m_rootSupportModel->updateRBDLJointPos(SingleSupportModel::CommandData);

	tf::Quaternion tfRobotOrientation;
	tf::quaternionEigenToTF((m_egoRotIncludesHeading() ? robotOrientation() : robotOrientationPR()), tfRobotOrientation);
	if(m_robotOrientationTime == ros::Time(0))
		m_tf_buf[0].stamp_ = m_rootSupportModel->joint(1)->feedback.stamp;
	else
		m_tf_buf[0].stamp_ = m_robotOrientationTime;
	m_tf_buf[0].setRotation(tfRobotOrientation); // This is the rotation from `ego_rot` to `trunk_link` (normally kept yaw-free)

	for(size_t i = 1; i < m_rootSupportModel->mBodies.size(); ++i)
	{
		ROS_ASSERT(i < m_tf_buf.size());
		tf::StampedTransform* t = &m_tf_buf[i];

		rbdlToTF(m_rootSupportModel->X_lambda[i], t);
		t->stamp_ = m_rootSupportModel->joint(i-1)->feedback.stamp;
	}

	m_pub_tf.sendTransform(m_tf_buf);

	ros::Time fixedTime = now + ros::Duration(0.5);
	for(size_t i = 0; i < m_tf_fixed_buf.size(); ++i)
		m_tf_fixed_buf[i].stamp_ = fixedTime;

	m_pub_tf.sendTransform(m_tf_fixed_buf);

	//
	// Publish other RobotModel information
	//

	// Publish the robot state
	robotcontrol::RobotState RSmsg;
	RSmsg.stamp = now;
	RSmsg.frame_id = "/trunk_link";
	RSmsg.orientation.w = m_robotOrientation.w();
	RSmsg.orientation.x = m_robotOrientation.x();
	RSmsg.orientation.y = m_robotOrientation.y();
	RSmsg.orientation.z = m_robotOrientation.z();
	RSmsg.orientationPR.w = m_robotOrientationPR.w();
	RSmsg.orientationPR.x = m_robotOrientationPR.x();
	RSmsg.orientationPR.y = m_robotOrientationPR.y();
	RSmsg.orientationPR.z = m_robotOrientationPR.z();
	RSmsg.orientationNoEYaw.w = m_robotOrientationNoEYaw.w();
	RSmsg.orientationNoEYaw.x = m_robotOrientationNoEYaw.x();
	RSmsg.orientationNoEYaw.y = m_robotOrientationNoEYaw.y();
	RSmsg.orientationNoEYaw.z = m_robotOrientationNoEYaw.z();
	RSmsg.orientationNoFYaw.w = m_robotOrientationNoFYaw.w();
	RSmsg.orientationNoFYaw.x = m_robotOrientationNoFYaw.x();
	RSmsg.orientationNoFYaw.y = m_robotOrientationNoFYaw.y();
	RSmsg.orientationNoFYaw.z = m_robotOrientationNoFYaw.z();
	RSmsg.orientationNoHeading.w = m_robotOrientationNoHeading.w();
	RSmsg.orientationNoHeading.x = m_robotOrientationNoHeading.x();
	RSmsg.orientationNoHeading.y = m_robotOrientationNoHeading.y();
	RSmsg.orientationNoHeading.z = m_robotOrientationNoHeading.z();
	RSmsg.angVel.x = m_robotAngularVelocity.x();
	RSmsg.angVel.y = m_robotAngularVelocity.y();
	RSmsg.angVel.z = m_robotAngularVelocity.z();
	RSmsg.accVec.x = m_accelerationVector.x();
	RSmsg.accVec.y = m_accelerationVector.y();
	RSmsg.accVec.z = m_accelerationVector.z();
	RSmsg.magVec.x = m_magneticFieldVector.x();
	RSmsg.magVec.y = m_magneticFieldVector.y();
	RSmsg.magVec.z = m_magneticFieldVector.z();
	RSmsg.EYaw     = m_robotEYaw;
	RSmsg.EPitch   = m_robotEPitch;
	RSmsg.ERoll    = m_robotERoll;
	RSmsg.FYaw     = m_robotFYaw;
	RSmsg.FPitch   = m_robotFPitch;
	RSmsg.FRoll    = m_robotFRoll;
	RSmsg.FHemi    = (m_robotFHemi ? 1 : -1);
	RSmsg.heading  = m_robotHeading;
	RSmsg.FYawPR   = m_robotFYawPR;
	RSmsg.FPitchPR = m_robotFPitchPR;
	RSmsg.FRollPR  = m_robotFRollPR;
	RSmsg.FHemiPR  = (m_robotFHemiPR ? 1 : -1);
	RSmsg.temperature = m_temperature;
	RSmsg.voltage  = m_voltage;
	m_pub_robotState.publish(RSmsg);

	// Publish the robot heading
	robotcontrol::RobotHeading RHmsg;
	RHmsg.stamp = now;
	RHmsg.heading = m_robotHeading;
	RHmsg.orientationPR.w = m_robotOrientationPR.w();
	RHmsg.orientationPR.x = m_robotOrientationPR.x();
	RHmsg.orientationPR.y = m_robotOrientationPR.y();
	RHmsg.orientationPR.z = m_robotOrientationPR.z();
	RHmsg.FYawPR = m_robotFYawPR;
	m_pub_robotHeading.publish(RHmsg);
}

RobotModel::State RobotModel::registerState(const std::string& name)
{
	std::vector<std::string>::iterator it = std::find(m_states.begin(), m_states.end(), name);
	if(it == m_states.end())
	{
		State state(m_states.size());
		m_states.push_back(name);
		return state;
	}

	State state(it - m_states.begin());

	return state;
}

void RobotModel::setState(RobotModel::State state)
{
	ROS_ASSERT(state.m_idx >= 0 && state.m_idx < (int)m_states.size());

	if(state != m_currentState)
	{
		robotcontrol::State msg;
		msg.id = state.m_idx;
		msg.label = m_states[state.m_idx];

		m_pub_state.publish(msg);

		m_PM.plotEvent(msg.label);
	}

	m_currentState = state;
}

std::string RobotModel::stateLabel(const robotcontrol::RobotModel::State& state) const
{
	ROS_ASSERT(state.m_idx >= 0 && state.m_idx < (int)m_states.size());

	return m_states[state.m_idx];
}

std::string RobotModel::currentStateLabel() const
{
	return stateLabel(state());
}

void RobotModel::setRelaxed(bool relax)
{
	m_relaxed = relax;
	m_relaxedWasSet = true;
}

void RobotModel::processReadData()
{
	// Differentiate the fused angles (this is not in setRobotOrientation as we only ever want this to happen once a cycle)
	m_golayDFYaw.put(m_robotFYaw);
	m_golayDFPitch.put(m_robotFPitch);
	m_golayDFRoll.put(m_robotFRoll);
	m_golayDFYawPR.put(m_robotFYawPR);
	m_golayDFPitchPR.put(m_robotFPitchPR);
	m_golayDFRollPR.put(m_robotFRollPR);
	
	// Retrieve the fused angle velocities and store them in the required member variables
	m_robotDFYaw = m_golayDFYaw.value() / m_timerDuration;
	m_robotDFPitch = m_golayDFPitch.value() / m_timerDuration;
	m_robotDFRoll = m_golayDFRoll.value() / m_timerDuration;
	m_robotDFYawPR = m_golayDFYawPR.value() / m_timerDuration;
	m_robotDFPitchPR = m_golayDFPitchPR.value() / m_timerDuration;
	m_robotDFRollPR = m_golayDFRollPR.value() / m_timerDuration;
}

void RobotModel::configurePlotManager()
{
	// Configure the plot manager variable names
	m_PM.setName(PM_ANGVEL_X,        "AngVel/x");
	m_PM.setName(PM_ANGVEL_Y,        "AngVel/y");
	m_PM.setName(PM_ANGVEL_Z,        "AngVel/z");
	m_PM.setName(PM_ANGVEL_N,        "AngVel/norm");
	m_PM.setName(PM_ACCVEC_X,        "AccVec/x");
	m_PM.setName(PM_ACCVEC_Y,        "AccVec/y");
	m_PM.setName(PM_ACCVEC_Z,        "AccVec/z");
	m_PM.setName(PM_ACCVEC_N,        "AccVec/norm");
	m_PM.setName(PM_MAGVEC_X,        "MagVec/x");
	m_PM.setName(PM_MAGVEC_Y,        "MagVec/y");
	m_PM.setName(PM_MAGVEC_Z,        "MagVec/z");
	m_PM.setName(PM_MAGVEC_N,        "MagVec/norm");
	m_PM.setName(PM_EULER_YAW,       "Euler/Yaw");
	m_PM.setName(PM_EULER_PITCH,     "Euler/Pitch");
	m_PM.setName(PM_EULER_ROLL,      "Euler/Roll");
	m_PM.setName(PM_FUSED_YAW,       "Fused/Yaw");
	m_PM.setName(PM_FUSED_PITCH,     "Fused/Pitch");
	m_PM.setName(PM_FUSED_ROLL,      "Fused/Roll");
	m_PM.setName(PM_FUSED_HEMI,      "Fused/Hemi");
	m_PM.setName(PM_FUSED_DYAW,      "Fused/DYaw");
	m_PM.setName(PM_FUSED_DPITCH,    "Fused/DPitch");
	m_PM.setName(PM_FUSED_DROLL,     "Fused/DRoll");
	m_PM.setName(PM_HEADING,         "Heading");
	m_PM.setName(PM_FUSED_YAW_PR,    "FusedPR/Yaw");
	m_PM.setName(PM_FUSED_PITCH_PR,  "FusedPR/Pitch");
	m_PM.setName(PM_FUSED_ROLL_PR,   "FusedPR/Roll");
	m_PM.setName(PM_FUSED_HEMI_PR,   "FusedPR/Hemi");
	m_PM.setName(PM_FUSED_DYAW_PR,   "FusedPR/DYaw");
	m_PM.setName(PM_FUSED_DPITCH_PR, "FusedPR/DPitch");
	m_PM.setName(PM_FUSED_DROLL_PR,  "FusedPR/DRoll");
	m_PM.setName(PM_TEMPERATURE,     "Temperature");
	m_PM.setName(PM_VOLTAGE,         "Voltage");
	m_PM.setName(PM_RELAXEDMODEL,    "RelaxedModel");
	m_PM.setName(PM_RELAXEDWRITTEN,  "RelaxedWritten");
	m_PM.setName(PM_ROBOT_STATE,     "RobotState");

	// Check that we have been thorough
	if(!m_PM.checkNames())
		ROS_ERROR("Please review any warnings above that are related to the naming of plotter variables!");
}

}
