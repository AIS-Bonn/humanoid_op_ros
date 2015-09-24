// Model of the robot
// Authors: Max Schwarz <max.schwarz@uni-bonn.de>
//          Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
// Comment: The RobotModel class amounts to a central storage location for information pertaining to the robot,
//          and forms an information gateway between the hardware interface and the remaining code.

// Ensure header is included only once
#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

// Includes - Library
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>

// Includes - ROS packages
#include <robotcontrol/model/joint.h>
#include <robotcontrol/model/golay.h>
#include <config_server/parameter.h>
#include <plot_msgs/Plot.h>

// Defines
#define MIN_TIMER_DURATION      0.008
#define DEFAULT_TIMER_DURATION  0.010 // Must be in the range specified by MIN/MAX_TIMER_DURATION
#define MAX_TIMER_DURATION      0.125 // Note that this isn't a hard limit

// Forwards declare the URDF model class
namespace urdf
{
	class Model;
}

// Robotcontrol namespace
namespace robotcontrol
{

// Forwards declarations
class SingleSupportModel;
class RCMarkerMan;
class RobotControl;

/**
 * @brief Kinodynamic robot model
 *
 * This class represents the robot with all its links and joints. Since we need
 * to work with multiple dynamics bases, it consists of one SingleSupportModel
 * for each tip in the kinematic tree.
 *
 * Parameters on the config_server:
 *
 * Name             | Meaning
 * ---------------- | ---------------------------
 * useSupportInfo   | Off = Just use the trunk SingleSupportModel for dynamics
 *
 * Note that the term "`trunk_link` frame" in used in the documentation of this
 * class to refer to the URDF root link. The `trunk_global` frame is then defined
 * as a coordinate frame with the same origin as the `trunk_link` frame, but
 * rotated to be aligned with the global coordinate system. The term 'ZYX yaw'
 * is used to refer to the yaw component of a rotation as defined per ZYX Euler
 * angles. The term 'fused yaw' is used to refer to the yaw component of a rotation
 * as defined per fused angles.
 **/
class RobotModel
{
public:
	//! @brief RobotModel constructor
	explicit RobotModel(RobotControl* robotControl = NULL);

	//! @brief RobotModel destructor
	virtual ~RobotModel();

	//! @name Joint level operations
	//@{
	//! @brief Return number of joints
	size_t numJoints() const { return m_joints.size(); }

	//! @brief Access a joint by index
	inline const boost::shared_ptr<Joint>& operator[](int i) { return m_joints[i]; }

	//! @brief Access a joint by index
	inline const boost::shared_ptr<Joint>& joint(int i) { return m_joints[i]; }

	//! @brief Get joint index by name
	int jointIndex(const std::string& name);

	//! @brief Find a joint by name
	Joint::Ptr getJoint(const std::string& name);

	//! @brief Save the current joint commands as the old joint commands
	void newCommands();

	//! @brief Set the relaxation state of the robot
	void setRelaxed(bool relax);

	//! @brief Get the relaxation state of the robot
	inline bool isRelaxed() const { return m_relaxed; }

	//! @brief Process the read data (called by robotcontrol after it calls readJointStates() of the hardware interface)
	void processReadData();
	//@}

	//! @name Initialization
	//@{
	//! @brief Set the internal URDF model of the robot
	void setModel(const boost::shared_ptr<urdf::Model>& model);

	//! @brief Retrieve the internal URDF model of the robot
	inline boost::shared_ptr<urdf::Model> urdf() { return m_model; }

	//! @brief Add a joint to the robot model
	void addJoint(const boost::shared_ptr<Joint>& joint);

	//! @brief Initialize the kinodynamic trees (i.e. the SingleSupportModels)
	void initTrees();

	//! @brief Update the internal copy of the nominal execution rate of the robotcontrol node (Note: This should only be set by RobotControl itself, and is for the purpose of making this value available to RobotModel and the hardware interfaces)
	void setTimerDuration(float sec) { m_timerDuration = (sec < MIN_TIMER_DURATION ? MIN_TIMER_DURATION : sec); }

	//! @brief Retrieve the nominal execution rate of the robotcontrol node
	float timerDuration() { return m_timerDuration; }
	//@}

	//! @name Dynamic model
	//@{
	//! @brief Perform an inverse dynamics computation for the robot
	void doInverseDynamics();

	//! @brief Set the support coefficient of a particular link
	void setSupportCoefficient(const boost::shared_ptr<const urdf::Link>& link, double coeff);
	void setSupportCoefficient(const std::string& linkName, double coeff);

	//! @brief Reset the support coefficients to all zero
	void resetSupport();

	//! @brief Get the single support model with the given link as the dynamics base
	boost::shared_ptr<SingleSupportModel> supportModel(const std::string& link_name);
	//@}

	//! @name Robot state
	//@{
	//! @brief Set the robot orientation (rotation `trunk_global` &rarr; `trunk_link`, sets `robotOrientationPR` to @p quaternion also, automatically removing the fused yaw component)
	void setRobotOrientation(const Eigen::Quaterniond& quaternion, const ros::Time& timestamp = ros::Time(0));

	//! @brief Set the robot orientation and fused pitch/roll robot orientation (rotation `trunk_global` &rarr; `trunk_link`, the latter is fused yaw-free although @p quaternionPR is permitted to have a fused yaw component that is then automatically removed)
	void setRobotOrientation(const Eigen::Quaterniond& quaternion, const Eigen::Quaterniond& quaternionPR, const ros::Time& timestamp = ros::Time(0));

	//! @brief Get the robot orientation (rotation `trunk_global` &rarr; `trunk_link`, complete orientation including fused yaw)
	inline const Eigen::Quaterniond& robotOrientation() const { return m_robotOrientation; }

	//! @brief Get the fused pitch/roll robot orientation (rotation `trunk_global` &rarr; `trunk_link` with zeroed fused yaw, set independently from `robotOrientation` and may be independently estimated)
	inline const Eigen::Quaterniond& robotOrientationPR() const { return m_robotOrientationPR; }

	//! @brief Get the ZYX yaw-free robot orientation (rotation `trunk_global` &rarr; `trunk_link` minus its ZYX yaw component, derived from `robotOrientation`)
	inline const Eigen::Quaterniond& robotOrientationNoEYaw() const { return m_robotOrientationNoEYaw; }

	//! @brief Get the robot's ZYX Euler yaw angle (ZYX yaw of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientation`)
	inline double robotEYaw() const { return m_robotEYaw; }

	//! @brief Get the robot's ZYX Euler pitch angle (ZYX pitch of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientation`)
	inline double robotEPitch() const { return m_robotEPitch; }

	//! @brief Get the robot's ZYX Euler roll angle (ZYX roll of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientation`)
	inline double robotERoll() const { return m_robotERoll; }

	//! @brief Get the fused yaw-free robot orientation (rotation `trunk_global` &rarr; `trunk_link` minus its fused yaw component, derived from `robotOrientation`)
	inline const Eigen::Quaterniond& robotOrientationNoFYaw() const { return m_robotOrientationNoFYaw; }

	//! @brief Get the robot's fused yaw angle (fused yaw of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientation`)
	inline double robotFYaw() const { return m_robotFYaw; }

	//! @brief Get the robot's fused pitch angle (fused pitch of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientation`)
	inline double robotFPitch() const { return m_robotFPitch; }

	//! @brief Get the robot's fused roll angle (fused roll of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientation`)
	inline double robotFRoll() const { return m_robotFRoll; }

	//! @brief Get the robot's orientation z-hemisphere (hemisphere of `trunk_link` frame with respect to `trunk_global` frame, boolean flag where `true` implies `1` and `false` implies `-1`, derived from `robotOrientation`)
	inline bool robotFHemi() const { return m_robotFHemi; }

	//! @brief Get the robot's fused pitch angle (fused pitch of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientationPR`)
	inline double robotFPitchPR() const { return m_robotFPitchPR; }

	//! @brief Get the robot's fused roll angle (fused roll of `trunk_link` frame with respect to `trunk_global` frame, units in rad, derived from `robotOrientationPR`)
	inline double robotFRollPR() const { return m_robotFRollPR; }

	//! @brief Get the robot's orientation z-hemisphere (hemisphere of `trunk_link` frame with respect to `trunk_global` frame, boolean flag where `true` implies `1` and `false` implies `-1`, derived from `robotOrientationPR`)
	inline bool robotFHemiPR() const { return m_robotFHemiPR; }

	//! @brief Get the heading-free robot orientation (equivalent to either `robotOrientationNoEYaw()` or `robotOrientationNoFYaw()`, depending on the `headingIsFusedYaw` configuration parameter, derived from `robotOrientation`)
	inline const Eigen::Quaterniond& robotOrientationNoHeading() const { return m_robotOrientationNoHeading; }

	//! @brief Get the robot's heading angle (equivalent to either `robotEYaw()` or `robotFYaw()`, depending on the `headingIsFusedYaw` configuration parameter, derived from `robotOrientation`)
	inline double robotHeading() const { return m_robotHeading; }

	//! @brief Get the robot's fused yaw velocity (derived from `robotOrientation`)
	inline double robotDFYaw() const { return m_robotDFYaw; }

	//! @brief Get the robot's fused pitch velocity (derived from `robotOrientation`)
	inline double robotDFPitch() const { return m_robotDFPitch; }

	//! @brief Get the robot's fused roll velocity (derived from `robotOrientation`)
	inline double robotDFRoll() const { return m_robotDFRoll; }

	//! @brief Get the robot's fused pitch velocity (derived from `robotOrientationPR`)
	inline double robotDFPitchPR() const { return m_robotDFPitchPR; }

	//! @brief Get the robot's fused roll velocity (derived from `robotOrientationPR`)
	inline double robotDFRollPR() const { return m_robotDFRollPR; }

	//! @brief Set the robot angular velocity (`trunk_link` frame, units in rad/s)
	inline void setRobotAngularVelocity(const Eigen::Vector3d& velocity) { m_robotAngularVelocity = velocity; }

	//! @brief Get the robot angular velocity (`trunk_link` frame, units in rad/s)
	inline const Eigen::Vector3d& robotAngularVelocity() const { return m_robotAngularVelocity; }

	//! @brief Set the measured acceleration vector (`trunk_link` frame, units in ms<sup>-2</sup>, acceleration is taken to be in the inertial sign convention meaning that gravity contributes an acceleration opposite to its true direction)
	inline void setAccelerationVector(const Eigen::Vector3d& accVec) { m_accelerationVector = accVec; }

	//! @brief Get the measured acceleration vector (`trunk_link` frame, units in ms<sup>-2</sup>, acceleration is taken to be in the inertial sign convention meaning that gravity contributes an acceleration opposite to its true direction)
	inline const Eigen::Vector3d& accelerationVector() const { return m_accelerationVector; }

	//! @brief Set the measured magnetic field vector (`trunk_link` frame, units in gauss)
	inline void setMagneticFieldVector(const Eigen::Vector3d& magVec) { m_magneticFieldVector = magVec; }

	//! @brief Get the measured magnetic field vector (`trunk_link` frame, units in gauss)
	inline const Eigen::Vector3d& magneticFieldVector() const { return m_magneticFieldVector; }
	//@}

	//! @name Dynamics information
	//@{
	//! @brief Get the center of mass in trunk_link coordinates
	Eigen::Vector3d centerOfMass() const;

	//! @brief Get the zero moment point (the frame in which the output ZMP is expressed is given inside the stamped transform, and is the base frame of the single support model with the highest current support coefficient)
	tf::Stamped<tf::Vector3> zeroMomentPoint() const;

	//! @brief Get the total robot mass
	inline double mass() const { return m_mass; }
	//@}

	//! @name Robotcontrol state
	//@{
	//! @brief Robot state class
	class State
	{
	public:
		friend class RobotModel;
		State() : m_idx(-1) {} //!< @brief Default constructor
		bool operator==(const State& other) const { return m_idx == other.m_idx; } //!< @brief Equals operator
		bool operator!=(const State& other) const { return m_idx != other.m_idx; } //!< @brief Not-equals operator
	private:
		explicit State(int idx) : m_idx(idx) {} //!< @brief Private constructor by index value
		int m_idx; //!< @brief Internal state index
	};

	//! @brief Create and register a robot state by name
	State registerState(const std::string& name);

	//! @brief Set the current robot state
	void setState(State state);
	
	//! @brief Get the current robot state
	inline State state() const { return m_currentState; }

	//! @brief Get a string representation of the given robot state
	std::string stateLabel(const State& state) const;

	//! @brief Get a string representation of the current robot state
	std::string currentStateLabel() const;
	//@}

	//! @name Publishing and visualization
	//@{
	//! @brief Publish visualizations and plotter data
	void visualizeData(RCMarkerMan* markers);

	//! @brief Publish TF and other miscellaneous data
	void publishTF(bool useMeasurement);
	//@}

	//! Get access to the RobotControl instance owning this model (Note: This instance may be null if this class instance was constructed without providing a RobotControl instance)
	inline RobotControl* robotControl() { return m_robotControl; }

private:
	//! @brief Helper method for initTrees()
	void doInit(const boost::shared_ptr<const urdf::Link>& link);
	
	// Constants
	const std::string CONFIG_PARAM_PATH;

	//! @brief Pointed to the owning robot control instance
	RobotControl* m_robotControl;

	//! @brief The URDF model
	boost::shared_ptr<urdf::Model> m_model;

	//! @brief List of all joints
	std::vector<boost::shared_ptr<Joint> > m_joints;

	//! @brief List of all support models
	std::vector<boost::shared_ptr<SingleSupportModel> > m_models;
	boost::shared_ptr<SingleSupportModel> m_rootSupportModel;
	void createSupportModelFromURDFLink(const boost::shared_ptr<const urdf::Link>& supportLink);
	void setSupportCoefficient(const boost::shared_ptr<SingleSupportModel>& linkModel, double coeff);

	// Current robot state
	ros::Time m_robotOrientationTime;               //!< @brief Timestamp of m_robotOrientation (if zero, use joint feedback time)
	Eigen::Quaterniond m_robotOrientation;          //!< @brief Current estimated robot orientation
	Eigen::Quaterniond m_robotOrientationNoEYaw;    //!< @brief Current estimated ZYX yaw-free robot orientation (derived from `m_robotOrientation`)
	Eigen::Quaterniond m_robotOrientationNoFYaw;    //!< @brief Current estimated fused yaw-free robot orientation (derived from `m_robotOrientation`)
	Eigen::Quaterniond m_robotOrientationNoHeading; //!< @brief Current estimated heading-free robot orientation (heading is one of ZYX yaw/fused yaw, derived from `m_robotOrientation`)
	Eigen::Quaterniond m_robotOrientationPR;        //!< @brief Current estimated fused yaw-free robot orientation (estimated directly as pure fused pitch/roll, set independently to `m_robotOrientation` by the hardware interface)
	Eigen::Vector3d m_robotAngularVelocity;         //!< @brief Current measured angular velocity
	Eigen::Vector3d m_accelerationVector;           //!< @brief Current measured acceleration vector
	Eigen::Vector3d m_magneticFieldVector;          //!< @brief Current measured magnetic field vector
	double m_robotEYaw;                             //!< @brief Current estimated robot ZYX Euler yaw angle (derived from `m_robotOrientation`)
	double m_robotEPitch;                           //!< @brief Current estimated robot ZYX Euler pitch angle (derived from `m_robotOrientation`)
	double m_robotERoll;                            //!< @brief Current estimated robot ZYX Euler roll angle (derived from `m_robotOrientation`)
	double m_robotFYaw;                             //!< @brief Current estimated robot fused yaw angle (derived from `m_robotOrientation`)
	double m_robotFPitch;                           //!< @brief Current estimated robot fused pitch angle (derived from `m_robotOrientation`)
	double m_robotFRoll;                            //!< @brief Current estimated robot fused roll angle (derived from `m_robotOrientation`)
	bool   m_robotFHemi;                            //!< @brief Current estimated robot orientation hemisphere (derived from `m_robotOrientation`)
	double m_robotHeading;                          //!< @brief Current estimated robot heading (heading is one of ZYX yaw/fused yaw, derived from `m_robotOrientation`)
	double m_robotFPitchPR;                         //!< @brief Current estimated robot fused pitch angle (derived from `m_robotOrientationPR`)
	double m_robotFRollPR;                          //!< @brief Current estimated robot fused roll angle (derived from `m_robotOrientationPR`)
	double m_robotFHemiPR;                          //!< @brief Current estimated robot orientation hemisphere (derived from `m_robotOrientationPR`)

	// Golay derivatives
	robotcontrol::GolayDerivative<double,1,9> m_golayDFYaw;
	robotcontrol::GolayDerivative<double,1,9> m_golayDFPitch;
	robotcontrol::GolayDerivative<double,1,9> m_golayDFRoll;
	robotcontrol::GolayDerivative<double,1,9> m_golayDFPitchPR;
	robotcontrol::GolayDerivative<double,1,9> m_golayDFRollPR;
	double m_robotDFYaw;
	double m_robotDFPitch;
	double m_robotDFRoll;
	double m_robotDFPitchPR;
	double m_robotDFRollPR;

	// Configuration server parameters
	config_server::Parameter<bool> m_useSupportInformation; // Parameter whether to use support information in the calculation of the inverse dynamics
	config_server::Parameter<bool> m_useFeedbackPos;        // Parameter whether to use commanded or measured data for the inverse dynamics
	config_server::Parameter<bool> m_plotRobotModelData;
	config_server::Parameter<bool> m_egoRotIncludesHeading;
	config_server::Parameter<bool> m_headingIsFusedYaw;

	// ROS publishing
	ros::Publisher m_pub_robotState;
	ros::Publisher m_pub_robotHeading;

	// Plotting
	enum PlotIDs
	{
		PM_ANGVEL_X = 0,
		PM_ANGVEL_Y,
		PM_ANGVEL_Z,
		PM_ANGVEL_N,
		PM_ACCVEC_X,
		PM_ACCVEC_Y,
		PM_ACCVEC_Z,
		PM_ACCVEC_N,
		PM_MAGVEC_X,
		PM_MAGVEC_Y,
		PM_MAGVEC_Z,
		PM_MAGVEC_N,
		PM_EULER_YAW,
		PM_EULER_PITCH,
		PM_EULER_ROLL,
		PM_FUSED_YAW,
		PM_FUSED_PITCH,
		PM_FUSED_ROLL,
		PM_FUSED_HEMI,
		PM_FUSED_DYAW,
		PM_FUSED_DPITCH,
		PM_FUSED_DROLL,
		PM_FUSED_PITCH_PR,
		PM_FUSED_ROLL_PR,
		PM_FUSED_HEMI_PR,
		PM_FUSED_DPITCH_PR,
		PM_FUSED_DROLL_PR,
		PM_HEADING,
		PM_SUPPORT
	};
	plot_msgs::Plot m_plot;
	ros::Publisher m_pub_plot;

	// TF transforms
	tf::TransformBroadcaster m_pub_tf;
	std::vector<tf::StampedTransform> m_tf_buf;
	std::vector<tf::StampedTransform> m_tf_fixed_buf;
	int m_tf_fixed_counter;
	bool m_publishEgoRot;

	// Robot states
	State m_currentState;
	ros::Publisher m_pub_state;
	std::vector<std::string> m_states;

	// Internal variables
	double m_mass;
	bool m_relaxed;
	float m_timerDuration;
};

}

#endif /* ROBOTMODEL_H */
// EOF