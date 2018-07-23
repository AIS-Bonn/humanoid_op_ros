// Humanoid kinematics - Serial pose classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <humanoid_kinematics/serial/serial_pose_classes.h>

// Humanoid kinematics namespace
namespace humanoid_kinematics
{
	// Serial kinematics pose classes namespace
	namespace serial_pose_classes
	{
		// ############################
		// #### Joint pose classes ####
		// ############################

		//
		// JointLegPose class
		//

		// Field names
		const std::string JointLegPose::fieldName[NUM_FIELDS] = {
			"HipYaw",
			"HipRoll",
			"HipPitch",
			"KneePitch",
			"AnklePitch",
			"AnkleRoll"
		};

		// Short field names
		const std::string JointLegPose::fieldNameShort[NUM_FIELDS] = {
			"HY",
			"HR",
			"HP",
			"KP",
			"AP",
			"AR"
		};

		// Left joint names
		const std::string JointLegPose::jointNameLeft[NUM_FIELDS] = {
			"left_hip_yaw",
			"left_hip_roll",
			"left_hip_pitch",
			"left_knee_pitch",
			"left_ankle_pitch",
			"left_ankle_roll"
		};

		// Right joint names
		const std::string JointLegPose::jointNameRight[NUM_FIELDS] = {
			"right_hip_yaw",
			"right_hip_roll",
			"right_hip_pitch",
			"right_knee_pitch",
			"right_ankle_pitch",
			"right_ankle_roll"
		};

		// All joint names
		const std::string* const JointLegPose::jointName[NUM_LR] = {
			(LEFT == 0 ? jointNameLeft : jointNameRight),
			(LEFT == 0 ? jointNameRight : jointNameLeft)
		};

		// Zero function
		void JointLegPose::setZero()
		{
			// Set the zero pose
			hipYaw = 0.0;
			hipRoll = 0.0;
			hipPitch = 0.0;
			kneePitch = 0.0;
			anklePitch = 0.0;
			ankleRoll = 0.0;
		}

		// Pose field serialisation to array
		void JointLegPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[HIPYAW] = hipYaw;
			array[HIPROLL] = hipRoll;
			array[HIPPITCH] = hipPitch;
			array[KNEEPITCH] = kneePitch;
			array[ANKLEPITCH] = anklePitch;
			array[ANKLEROLL] = ankleRoll;
		}

		// Pose field deserialisation from array
		void JointLegPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			hipYaw = array[HIPYAW];
			hipRoll = array[HIPROLL];
			hipPitch = array[HIPPITCH];
			kneePitch = array[KNEEPITCH];
			anklePitch = array[ANKLEPITCH];
			ankleRoll = array[ANKLEROLL];
		}

		// Append function for joint information
		void JointLegPose::appendJointInfo(std::vector<JointInfo>& info) const
		{
			// Append information about all joints to the vector
			const std::string* name = jointName[limbIndex];
			for(std::size_t f = 0; f < NUM_FIELDS; f++)
				info.emplace_back(name[f], limbIndex, limbType);
		}

		//
		// JointArmPose class
		//

		// Constants
		const double JointArmPose::ShoulderPitchMax = 0.75*M_PI;

		// Field names
		const std::string JointArmPose::fieldName[NUM_FIELDS] = {
			"ShoulderPitch",
			"ShoulderRoll",
			"ElbowPitch"
		};

		// Short field names
		const std::string JointArmPose::fieldNameShort[NUM_FIELDS] = {
			"SP",
			"SR",
			"EP"
		};

		// Left joint names
		const std::string JointArmPose::jointNameLeft[NUM_FIELDS] = {
			"left_shoulder_pitch",
			"left_shoulder_roll",
			"left_elbow_pitch"
		};

		// Right joint names
		const std::string JointArmPose::jointNameRight[NUM_FIELDS] = {
			"right_shoulder_pitch",
			"right_shoulder_roll",
			"right_elbow_pitch"
		};

		// All joint names
		const std::string* const JointArmPose::jointName[NUM_LR] = {
			(LEFT == 0 ? jointNameLeft : jointNameRight),
			(LEFT == 0 ? jointNameRight : jointNameLeft)
		};

		// Zero function
		void JointArmPose::setZero()
		{
			// Set the zero pose
			shoulderPitch = 0.0;
			shoulderRoll = 0.0;
			elbowPitch = 0.0;
		}

		// Pose field serialisation to array
		void JointArmPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[SHOULDERPITCH] = shoulderPitch;
			array[SHOULDERROLL] = shoulderRoll;
			array[ELBOWPITCH] = elbowPitch;
		}

		// Pose field deserialisation from array
		void JointArmPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			shoulderPitch = array[SHOULDERPITCH];
			shoulderRoll = array[SHOULDERROLL];
			elbowPitch = array[ELBOWPITCH];
		}

		// Append function for joint information
		void JointArmPose::appendJointInfo(std::vector<JointInfo>& info) const
		{
			// Append information about all joints to the vector
			const std::string* name = jointName[limbIndex];
			for(std::size_t f = 0; f < NUM_FIELDS; f++)
				info.emplace_back(name[f], limbIndex, limbType);
		}

		//
		// JointHeadPose class
		//

		// Field names
		const std::string JointHeadPose::fieldName[NUM_FIELDS] = {
			"NeckYaw",
			"NeckPitch"
		};

		// Short field names
		const std::string JointHeadPose::fieldNameShort[NUM_FIELDS] = {
			"NY",
			"NP"
		};

		// Left joint names
		const std::string JointHeadPose::jointName[NUM_FIELDS] = {
			"neck_yaw",
			"head_pitch"
		};

		// Zero function
		void JointHeadPose::setZero()
		{
			// Set the zero pose
			neckYaw = 0.0;
			neckPitch = 0.0;
		}

		// Pose field serialisation to array
		void JointHeadPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[NECKYAW] = neckYaw;
			array[NECKPITCH] = neckPitch;
		}

		// Pose field deserialisation from array
		void JointHeadPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			neckYaw = array[NECKYAW];
			neckPitch = array[NECKPITCH];
		}

		// Append function for joint information
		void JointHeadPose::appendJointInfo(std::vector<JointInfo>& info) const
		{
			// Append information about all joints to the vector
			for(std::size_t f = 0; f < NUM_FIELDS; f++)
				info.emplace_back(jointName[f], limbIndex, limbType);
		}

		// ###############################
		// #### Abstract pose classes ####
		// ###############################

		//
		// AbsLegPose class
		//

		// Field names
		const std::string AbsLegPose::fieldName[NUM_FIELDS] = {
			"AngleX",
			"AngleY",
			"AngleZ",
			"FootAngleX",
			"FootAngleY",
			"Retraction"
		};

		// Short field names
		const std::string AbsLegPose::fieldNameShort[NUM_FIELDS] = {
			"Ax",
			"Ay",
			"Az",
			"Fx",
			"Fy",
			"Rt"
		};

		// Zero function
		void AbsLegPose::setZero()
		{
			// Set the zero pose
			angleX = 0.0;
			angleY = 0.0;
			angleZ = 0.0;
			footAngleX = 0.0;
			footAngleY = 0.0;
			retraction = 0.0;
		}

		// Pose field serialisation to array
		void AbsLegPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[ANGLEX] = angleX;
			array[ANGLEY] = angleY;
			array[ANGLEZ] = angleZ;
			array[FOOTANGLEX] = footAngleX;
			array[FOOTANGLEY] = footAngleY;
			array[RETRACTION] = retraction;
		}

		// Pose field deserialisation from array
		void AbsLegPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			angleX = array[ANGLEX];
			angleY = array[ANGLEY];
			angleZ = array[ANGLEZ];
			footAngleX = array[FOOTANGLEX];
			footAngleY = array[FOOTANGLEY];
			retraction = array[RETRACTION];
		}

		//
		// AbsArmPose class
		//

		// Field names
		const std::string AbsArmPose::fieldName[NUM_FIELDS] = {
			"AngleX",
			"AngleY",
			"Retraction"
		};

		// Short field names
		const std::string AbsArmPose::fieldNameShort[NUM_FIELDS] = {
			"Ax",
			"Ay",
			"Rt"
		};

		// Zero function
		void AbsArmPose::setZero()
		{
			// Set the zero pose
			angleX = 0.0;
			angleY = 0.0;
			retraction = 0.0;
		}

		// Pose field serialisation to array
		void AbsArmPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[ANGLEX] = angleX;
			array[ANGLEY] = angleY;
			array[RETRACTION] = retraction;
		}

		// Pose field deserialisation from array
		void AbsArmPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			angleX = array[ANGLEX];
			angleY = array[ANGLEY];
			retraction = array[RETRACTION];
		}

		//
		// AbsHeadPose class
		//

		// Field names
		const std::string AbsHeadPose::fieldName[NUM_FIELDS] = {
			"AngleY",
			"AngleZ"
		};

		// Short field names
		const std::string AbsHeadPose::fieldNameShort[NUM_FIELDS] = {
			"Ay",
			"Az"
		};

		// Zero function
		void AbsHeadPose::setZero()
		{
			// Set the zero pose
			angleY = 0.0;
			angleZ = 0.0;
		}

		// Pose field serialisation to array
		void AbsHeadPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[ANGLEY] = angleY;
			array[ANGLEZ] = angleZ;
		}

		// Pose field deserialisation from array
		void AbsHeadPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			angleY = array[ANGLEY];
			angleZ = array[ANGLEZ];
		}

		// ##############################
		// #### Inverse pose classes ####
		// ##############################

		//
		// InvLegPose class
		//

		// Field names
		const std::string InvLegPose::fieldName[NUM_FIELDS] = {
			"AnklePosX",
			"AnklePosY",
			"AnklePosZ",
			"FootRotW",
			"FootRotX",
			"FootRotY",
			"FootRotZ"
		};

		// Short field names
		const std::string InvLegPose::fieldNameShort[NUM_FIELDS] = {
			"ax",
			"ay",
			"az",
			"qw",
			"qx",
			"qy",
			"qz"
		};

		// Zero function
		void InvLegPose::setZero()
		{
			// Set the zero pose
			anklePos.setZero();
			footRot.setIdentity();
		}

		// Pose field serialisation to array
		void InvLegPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[ANKLEPOSX] = anklePos.x();
			array[ANKLEPOSY] = anklePos.y();
			array[ANKLEPOSZ] = anklePos.z();
			array[FOOTROTW] = footRot.w();
			array[FOOTROTX] = footRot.x();
			array[FOOTROTY] = footRot.y();
			array[FOOTROTZ] = footRot.z();
		}

		// Pose field deserialisation from array
		void InvLegPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			anklePos.x() = array[ANKLEPOSX];
			anklePos.y() = array[ANKLEPOSY];
			anklePos.z() = array[ANKLEPOSZ];
			footRot.w() = array[FOOTROTW];
			footRot.x() = array[FOOTROTX];
			footRot.y() = array[FOOTROTY];
			footRot.z() = array[FOOTROTZ];
		}

		//
		// InvArmPose class
		//

		// Field names
		const std::string InvArmPose::fieldName[NUM_FIELDS] = {
			"HandPosX",
			"HandPosY",
			"HandPosZ"
		};

		// Short field names
		const std::string InvArmPose::fieldNameShort[NUM_FIELDS] = {
			"hx",
			"hy",
			"hz"
		};

		// Zero function
		void InvArmPose::setZero()
		{
			// Set the zero pose
			handPos.setZero();
		}

		// Pose field serialisation to array
		void InvArmPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[HANDPOSX] = handPos.x();
			array[HANDPOSY] = handPos.y();
			array[HANDPOSZ] = handPos.z();
		}

		// Pose field deserialisation from array
		void InvArmPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			handPos.x() = array[HANDPOSX];
			handPos.y() = array[HANDPOSY];
			handPos.z() = array[HANDPOSZ];
		}

		//
		// InvHeadPose class
		//

		// Field names
		const std::string InvHeadPose::fieldName[NUM_FIELDS] = {
			"DirnVecX",
			"DirnVecY",
			"DirnVecZ",
			"HeadRotW",
			"HeadRotX",
			"HeadRotY",
			"HeadRotZ"
		};

		// Short field names
		const std::string InvHeadPose::fieldNameShort[NUM_FIELDS] = {
			"dx",
			"dy",
			"dz",
			"qw",
			"qx",
			"qy",
			"qz"
		};

		// Zero function
		void InvHeadPose::setZero()
		{
			// Set the zero pose
			dirnVec = rot_conv::VecUnitX();
			headRot.setIdentity();
		}

		// Pose field serialisation to array
		void InvHeadPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[DIRNVECX] = dirnVec.x();
			array[DIRNVECY] = dirnVec.y();
			array[DIRNVECZ] = dirnVec.z();
			array[HEADROTW] = headRot.w();
			array[HEADROTX] = headRot.x();
			array[HEADROTY] = headRot.y();
			array[HEADROTZ] = headRot.z();
		}

		// Pose field deserialisation from array
		void InvHeadPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			dirnVec.x() = array[DIRNVECX];
			dirnVec.y() = array[DIRNVECY];
			dirnVec.z() = array[DIRNVECZ];
			headRot.w() = array[HEADROTW];
			headRot.x() = array[HEADROTX];
			headRot.y() = array[HEADROTY];
			headRot.z() = array[HEADROTZ];
		}

		// Helper function to convert from a direction vector to a head rotation
		void InvHeadPose::RotFromVec(const rot_conv::Vec3& dirnVec, rot_conv::Quat& headRot)
		{
			// Calculate the head y-axis
			rot_conv::Vec3 yhat;
			double xynorm = sqrt(dirnVec.x()*dirnVec.x() + dirnVec.y()*dirnVec.y());
			if(xynorm <= 0.0)
				yhat = rot_conv::VecUnitY();
			else
			{
				yhat.x() = -dirnVec.y() / xynorm;
				yhat.y() = dirnVec.x() / xynorm;
				yhat.z() = 0.0;
			}

			// Calculate the head z-axis
			rot_conv::Vec3 zhat = dirnVec.cross(yhat);

			// Calculate the head rotation
			rot_conv::Rotmat R;
			R << dirnVec.x(), yhat.x(), zhat.x(),
			     dirnVec.y(), yhat.y(), zhat.y(),
			     dirnVec.z(), yhat.z(), zhat.z();

			// Return the required head rotation as a quaternion
			rot_conv::QuatFromRotmat(R, headRot);
		}

		// Helper function to convert from a head rotation to a direction vector
		void InvHeadPose::VecFromRot(const rot_conv::Quat& headRot, rot_conv::Vec3& dirnVec)
		{
			// Calculate and return the required direction vector
			rot_conv::AxisXFromQuat(headRot, dirnVec);
		}

		// ##########################
		// #### Tip pose classes ####
		// ##########################

		//
		// LegTipPose class
		//

		// Field names
		const std::string LegTipPose::fieldName[NUM_FIELDS] = {
			"PosX",
			"PosY",
			"PosZ",
			"RotW",
			"RotX",
			"RotY",
			"RotZ"
		};

		// Short field names
		const std::string LegTipPose::fieldNameShort[NUM_FIELDS] = {
			"px",
			"py",
			"pz",
			"qw",
			"qx",
			"qy",
			"qz"
		};

		// Zero function
		void LegTipPose::setZero()
		{
			// Set the zero pose
			pos.setZero();
			rot.setIdentity();
		}

		// Pose field serialisation to array
		void LegTipPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[POSX] = pos.x();
			array[POSY] = pos.y();
			array[POSZ] = pos.z();
			array[ROTW] = rot.w();
			array[ROTX] = rot.x();
			array[ROTY] = rot.y();
			array[ROTZ] = rot.z();
		}

		// Pose field deserialisation from array
		void LegTipPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			pos.x() = array[POSX];
			pos.y() = array[POSY];
			pos.z() = array[POSZ];
			rot.w() = array[ROTW];
			rot.x() = array[ROTX];
			rot.y() = array[ROTY];
			rot.z() = array[ROTZ];
		}

		//
		// ArmTipPose class
		//

		// Field names
		const std::string ArmTipPose::fieldName[NUM_FIELDS] = {
			"PosX",
			"PosY",
			"PosZ",
			"RotW",
			"RotX",
			"RotY",
			"RotZ"
		};

		// Short field names
		const std::string ArmTipPose::fieldNameShort[NUM_FIELDS] = {
			"px",
			"py",
			"pz",
			"qw",
			"qx",
			"qy",
			"qz"
		};

		// Zero function
		void ArmTipPose::setZero()
		{
			// Set the zero pose
			pos.setZero();
			rot.setIdentity();
		}

		// Pose field serialisation to array
		void ArmTipPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[POSX] = pos.x();
			array[POSY] = pos.y();
			array[POSZ] = pos.z();
			array[ROTW] = rot.w();
			array[ROTX] = rot.x();
			array[ROTY] = rot.y();
			array[ROTZ] = rot.z();
		}

		// Pose field deserialisation from array
		void ArmTipPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			pos.x() = array[POSX];
			pos.y() = array[POSY];
			pos.z() = array[POSZ];
			rot.w() = array[ROTW];
			rot.x() = array[ROTX];
			rot.y() = array[ROTY];
			rot.z() = array[ROTZ];
		}

		//
		// HeadTipPose class
		//

		// Field names
		const std::string HeadTipPose::fieldName[NUM_FIELDS] = {
			"PosX",
			"PosY",
			"PosZ",
			"RotW",
			"RotX",
			"RotY",
			"RotZ"
		};

		// Short field names
		const std::string HeadTipPose::fieldNameShort[NUM_FIELDS] = {
			"px",
			"py",
			"pz",
			"qw",
			"qx",
			"qy",
			"qz"
		};

		// Zero function
		void HeadTipPose::setZero()
		{
			// Set the zero pose
			pos.setZero();
			rot.setIdentity();
		}

		// Pose field serialisation to array
		void HeadTipPose::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[POSX] = pos.x();
			array[POSY] = pos.y();
			array[POSZ] = pos.z();
			array[ROTW] = rot.w();
			array[ROTX] = rot.x();
			array[ROTY] = rot.y();
			array[ROTZ] = rot.z();
		}

		// Pose field deserialisation from array
		void HeadTipPose::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			pos.x() = array[POSX];
			pos.y() = array[POSY];
			pos.z() = array[POSZ];
			rot.w() = array[ROTW];
			rot.x() = array[ROTX];
			rot.y() = array[ROTY];
			rot.z() = array[ROTZ];
		}

		// #####################################
		// #### Joint pose velocity classes ####
		// #####################################

		//
		// JointLegPoseVel class
		//

		// Field names
		const std::string JointLegPoseVel::fieldName[NUM_FIELDS] = {
			"HipYawVel",
			"HipRollVel",
			"HipPitchVel",
			"KneePitchVel",
			"AnklePitchVel",
			"AnkleRollVel"
		};

		// Short field names
		const std::string JointLegPoseVel::fieldNameShort[NUM_FIELDS] = {
			"HYV",
			"HRV",
			"HPV",
			"KPV",
			"APV",
			"ARV"
		};

		// Zero function
		void JointLegPoseVel::setZero()
		{
			// Set the zero pose
			hipYawVel = 0.0;
			hipRollVel = 0.0;
			hipPitchVel = 0.0;
			kneePitchVel = 0.0;
			anklePitchVel = 0.0;
			ankleRollVel = 0.0;
		}

		// Pose field serialisation to array
		void JointLegPoseVel::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[HIPYAWVEL] = hipYawVel;
			array[HIPROLLVEL] = hipRollVel;
			array[HIPPITCHVEL] = hipPitchVel;
			array[KNEEPITCHVEL] = kneePitchVel;
			array[ANKLEPITCHVEL] = anklePitchVel;
			array[ANKLEROLLVEL] = ankleRollVel;
		}

		// Pose field deserialisation from array
		void JointLegPoseVel::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			hipYawVel = array[HIPYAWVEL];
			hipRollVel = array[HIPROLLVEL];
			hipPitchVel = array[HIPPITCHVEL];
			kneePitchVel = array[KNEEPITCHVEL];
			anklePitchVel = array[ANKLEPITCHVEL];
			ankleRollVel = array[ANKLEROLLVEL];
		}

		// ########################################
		// #### Abstract pose velocity classes ####
		// ########################################

		//
		// AbsLegPoseVel class
		//

		// Field names
		const std::string AbsLegPoseVel::fieldName[NUM_FIELDS] = {
			"AngleXVel",
			"AngleYVel",
			"AngleZVel",
			"FootAngleXVel",
			"FootAngleYVel",
			"RetractionVel"
		};

		// Short field names
		const std::string AbsLegPoseVel::fieldNameShort[NUM_FIELDS] = {
			"AxV",
			"AyV",
			"AzV",
			"FxV",
			"FyV",
			"RtV"
		};

		// Zero function
		void AbsLegPoseVel::setZero()
		{
			// Set the zero pose
			angleXVel = 0.0;
			angleYVel = 0.0;
			angleZVel = 0.0;
			footAngleXVel = 0.0;
			footAngleYVel = 0.0;
			retractionVel = 0.0;
		}

		// Pose field serialisation to array
		void AbsLegPoseVel::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[ANGLEXVEL] = angleXVel;
			array[ANGLEYVEL] = angleYVel;
			array[ANGLEZVEL] = angleZVel;
			array[FOOTANGLEXVEL] = footAngleXVel;
			array[FOOTANGLEYVEL] = footAngleYVel;
			array[RETRACTIONVEL] = retractionVel;
		}

		// Pose field deserialisation from array
		void AbsLegPoseVel::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			angleXVel = array[ANGLEXVEL];
			angleYVel = array[ANGLEYVEL];
			angleZVel = array[ANGLEZVEL];
			footAngleXVel = array[FOOTANGLEXVEL];
			footAngleYVel = array[FOOTANGLEYVEL];
			retractionVel = array[RETRACTIONVEL];
		}

		// #######################################
		// #### Inverse pose velocity classes ####
		// #######################################

		//
		// InvLegPoseVel class
		//

		// Field names
		const std::string InvLegPoseVel::fieldName[NUM_FIELDS] = {
			"AnkleVelX",
			"AnkleVelY",
			"AnkleVelZ",
			"FootAngVelX",
			"FootAngVelY",
			"FootAngVelZ"
		};

		// Short field names
		const std::string InvLegPoseVel::fieldNameShort[NUM_FIELDS] = {
			"vx",
			"vy",
			"vz",
			"wx",
			"wy",
			"wz"
		};

		// Zero function
		void InvLegPoseVel::setZero()
		{
			// Set the zero pose
			ankleVel.setZero();
			footAngVel.setZero();
		}

		// Pose field serialisation to array
		void InvLegPoseVel::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[ANKLEVELX] = ankleVel.x();
			array[ANKLEVELY] = ankleVel.y();
			array[ANKLEVELZ] = ankleVel.z();
			array[FOOTANGVELX] = footAngVel.x();
			array[FOOTANGVELY] = footAngVel.y();
			array[FOOTANGVELZ] = footAngVel.z();
		}

		// Vele field deserialisation from array
		void InvLegPoseVel::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			ankleVel.x() = array[ANKLEVELX];
			ankleVel.y() = array[ANKLEVELY];
			ankleVel.z() = array[ANKLEVELZ];
			footAngVel.x() = array[FOOTANGVELX];
			footAngVel.y() = array[FOOTANGVELY];
			footAngVel.z() = array[FOOTANGVELZ];
		}

		// ###################################
		// #### Tip pose velocity classes ####
		// ###################################

		//
		// LegTipPoseVel class
		//

		// Field names
		const std::string LegTipPoseVel::fieldName[NUM_FIELDS] = {
			"VelX",
			"VelY",
			"VelZ",
			"AngVelX",
			"AngVelY",
			"AngVelZ"
		};

		// Short field names
		const std::string LegTipPoseVel::fieldNameShort[NUM_FIELDS] = {
			"vx",
			"vy",
			"vz",
			"wx",
			"wy",
			"wz"
		};

		// Zero function
		void LegTipPoseVel::setZero()
		{
			// Set the zero pose
			vel.setZero();
			angVel.setZero();
		}

		// Pose field serialisation to array
		void LegTipPoseVel::toArray(double* array) const
		{
			// Transcribe the data fields to the array
			array[VELX] = vel.x();
			array[VELY] = vel.y();
			array[VELZ] = vel.z();
			array[ANGVELX] = angVel.x();
			array[ANGVELY] = angVel.y();
			array[ANGVELZ] = angVel.z();
		}

		// Pose field deserialisation from array
		void LegTipPoseVel::fromArray(const double* array)
		{
			// Transcribe the data fields from the array
			vel.x() = array[VELX];
			vel.y() = array[VELY];
			vel.z() = array[VELZ];
			angVel.x() = array[ANGVELX];
			angVel.y() = array[ANGVELY];
			angVel.z() = array[ANGVELZ];
		}
	}
}
// EOF