#include "RobotModel.h"
#include "Globals.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include "State.h"
// #include <GL/Glu.h>
#include "util/GLlib.h"

// The RobotModel is an important object in the framework that describes a specific
// humanoid robot model and offers interfaces to manipulate and query the pose and
// meta information of the robot. For kinematics, three levels of abstraction are
// supported. A low level interface based on joint angles (setPose()) allows to
// manipulate the kinematic chain directly with joint angles. A medium level,
// abstract kinematic interface allows manipulation on a more abstract level using
// leg extension, leg angle and foot angle as parameters and calculating the
// coordination of single joints internally. The highest level consists of an inverse
// kinematic interface that can be used to provide a pose based on end effector
// positions. Consistency of all three levels is maintained at all times.

// To complement the stateless kinematic interface, a "walk" interface is provided,
// that is supposed to be used in combination with continuous motion trajectories in
// joint angle space. The walk interface does not generate a walk. It takes a walk as
// a parameter and maintains the pose of the robot model as well as meta information,
// such as the current support leg and a fixed support foot location on the floor.

// The main functions of the robot model are
// - description of the kinematic structure and masses
// - loading a calibration file that defines signs, offsets and limits for the joints.
// - a "walk" interface for continuous motion trajectories including trunk attitude information.
//   that also takes care of support foot tracking
// - a stateless interface to set the model in a pose using joint targets, the abstract kinematic
//   interface and/or the inverse kinematic interface
// - meta information interface for CoM location, support sole angle, step vector and such.
// - openGL visualization code in the draw() method.

namespace indep_cpg_gait
{
	RobotModel::RobotModel()
	{
		//FIXME change length sttings
		name = "Simon";

		// Model definition parameters.
		hipWidth = 0.36; // Distance between the left and right hip joint.
		footOffsetX = 0.1; // Forward offset between the foot center and the ankle joint.
		footWidth = 0.2; // Width of the foot plate.
		footLength = 0.4; // Length of the foot plate.
		footHeight = 0.04; // Height of the foot plate.
		jointRadius = 0.06; // Radius of every joint. Joints are modeled as spheres.
		legSegmentLength = 0.5; // Length of the leg segments (thigh and shank) not including the joint radius.
		legThickness = 0.12; // Thickness of the leg hull. Only for collision shape and visuals.
		hipDiscHeight = 0.5; // Height of the hip disc for the hip Z rotation.
		trunkHeight = 0.8; // The height of the trunk.
		headRadius = 0.15; // Radius of the head.
		armThickness = 0.08; // Thickness of the arm hull. Only for collision shape and visuals.
		armSegmentLength = 0.35; // Length of the arm segments upper arm and lower arm not including the joint radius.

		armLength = 0.4;  //2.0*(2.0*jointRadius+armSegmentLength);//FIXME
		legLength = 0.44; //2.0*(2.0*jointRadius+legSegmentLength);//FIXME

		footMass = 1.0;
		legSegmentMass = 1.0;
		armSegmentMass = 0.5;
		trunkMass = 3.0;
		headMass = 0.5;
		hipDiscMass = 1.0;

// 		loadCalib(); //Just for simulator

		// Define the kinematic chain hierarchy.
		// The base is the root frame.
		base.setReferenceFrame(NULL);
		imu.setReferenceFrame(&base);
		neck.setReferenceFrame(&base);
		camera.setReferenceFrame(&neck);
		lShoulder.setReferenceFrame(&base);
		lElbow.setReferenceFrame(&lShoulder);
		lHand.setReferenceFrame(&lElbow);
		rShoulder.setReferenceFrame(&base);
		rElbow.setReferenceFrame(&rShoulder);
		rHand.setReferenceFrame(&rElbow);
		rHip.setReferenceFrame(&base);
		rKnee.setReferenceFrame(&rHip);
		rAnkle.setReferenceFrame(&rKnee);
		rFootFloorPoint.setReferenceFrame(&rAnkle);
		lHip.setReferenceFrame(&base);
		lKnee.setReferenceFrame(&lHip);
		lAnkle.setReferenceFrame(&lKnee);
		lFootFloorPoint.setReferenceFrame(&lAnkle);

		base.setTranslation(0, 0.5*hipWidth, jointRadius);
		imu.setTranslation(0, 0, 0.5*trunkHeight);
		neck.setTranslation(0, 0, trunkHeight + jointRadius);
		camera.setTranslation(headRadius, 0, jointRadius + headRadius);
		rShoulder.setTranslation(0, -(0.5*hipWidth + 0.5*legThickness + jointRadius), trunkHeight - jointRadius);
		rElbow.setTranslation(0, 0, -(jointRadius + armSegmentLength + jointRadius));
		rHand.setTranslation(0, 0, -(jointRadius + armSegmentLength));
		lShoulder.setTranslation(0, 0.5*hipWidth + 0.5*legThickness + jointRadius, trunkHeight - jointRadius);
		lElbow.setTranslation(0, 0, -(jointRadius + armSegmentLength + jointRadius));
		lHand.setTranslation(0, 0, -(jointRadius + armSegmentLength));
		lHip.setTranslation(0, 0.5*hipWidth, -jointRadius);
		lKnee.setTranslation(0, 0, -(jointRadius + legSegmentLength + jointRadius));
		lAnkle.setTranslation(0, 0, -(jointRadius + legSegmentLength + jointRadius));
		lFootFloorPoint.setTranslation(0, 0, -(footHeight + jointRadius));
		rHip.setTranslation(0, -0.5*hipWidth, -jointRadius);
		rKnee.setTranslation(0, 0, -(jointRadius + legSegmentLength + jointRadius));
		rAnkle.setTranslation(0, 0, -(jointRadius + legSegmentLength + jointRadius));
		rFootFloorPoint.setTranslation(0, 0, -(footHeight + jointRadius));

		supportLegSign = 1;
		lastSupportChangeTime = 0;

		reset();
	}

	// Rests the model to a 0 pose and 0 location.
	void RobotModel::reset()
	{
		setPose(Pose());
		base.setPosition(0, 0, 0);
		base.setRotation(Quaternion());
		supportExchange = false;
	}

	// Resets the model's position to the center of the world coordinate frame.
	void RobotModel::resetPosition()
	{
		base.setPosition(0, 0, 0);
		footStep.setPosition(0, 0, 0);
	}

	// Simulates a walk. It assumes to be fed with continuous joint angle and fused angle data.
	// It sets the model into the given pose and rotates the model such that the trunk angle
	// matches the fused angle. A fixed location of the support foot on the floor is maintained
	// to implement a realistic motion model. The sign of the support foot is flipped when the
	// swing foot z coordinate is less than the current support foot z coordinate. It also sets
	// the support exchange boolean flag. The stepVector() makes most sense when this method is
	// used and the support exchange indicates true.
	void RobotModel::walk(const Pose& pose, Vec2f fusedAngle)
	{
		// Perform support exchange if needed.
		// This will set the footStep frame to the location of the new support foot.
		double now = stopWatch.time();
		supportExchange = false;
		int lowerFoot = rFootFloorPoint.position().z < lFootFloorPoint.position().z ? 1 : -1;
		if (lowerFoot != supportLegSign
				&& now - lastSupportChangeTime > 0.1  // with hysteresis
				&& fabs(fusedAngle.x) < 1.2
				&& fabs(fusedAngle.y) < 1.2)
		{
			lastSupportChangeTime = now;
			supportExchange = true;
			setSupportLeg(lowerFoot);
		}

		// Set the robot into the pose according to the joint angles and the fused angle.
		setPose(pose);
		applyFusedAngle(fusedAngle);
		alignWithFootStep();
	}

	// Rotates the model such that the trunk matches the provided fused angle.
	void RobotModel::applyFusedAngle(Vec2f fusedAngle)
	{
		base.setOrientation(Quaternion(Vec(1,0,0), fusedAngle.x) * Quaternion(Vec(0,1,0), fusedAngle.y));
	}

	// Sets the support leg sign and updates the footStep frame.
	void RobotModel::setSupportLeg(int sls)
	{
		supportLegSign = sls;

		if (supportLegSign == 1)
		{
			footStep.setPosition(Vec(rFootFloorPoint.position().x, rFootFloorPoint.position().y, 0));
			footStep.setOrientation(Quaternion(Vec(0,0,1), getRotationAngles(rFootFloorPoint).z));
		}
		else
		{
			footStep.setPosition(Vec(lFootFloorPoint.position().x, lFootFloorPoint.position().y, 0));
			footStep.setOrientation(Quaternion(Vec(0,0,1), getRotationAngles(lFootFloorPoint).z));
		}
	}

	// Aligns the model such that the support foot matches the footStep frame.
	void RobotModel::alignWithFootStep()
	{
		if (supportLegSign == 1)
		{
			base.rotate(Quaternion(base.transformOf(Vec(0,0,1)), getRotationAngles(footStep).z - getRotationAngles(rFootFloorPoint).z));
			base.translate(footStep.position() - rFootFloorPoint.position());
		}
		else
		{
			base.rotate(Quaternion(base.transformOf(Vec(0,0,1)), getRotationAngles(footStep).z - getRotationAngles(lFootFloorPoint).z));
			base.translate(footStep.position() - lFootFloorPoint.position());
		}
	}

	void RobotModel::alignModel(Vec2f fusedAngle)
	{
		if (supportLegSign == 1)
		{
			base.setOrientation(Quaternion(Vec(0,0,1), 0));
			double angle = getRotationAngles(footStep).z + getRotationAngles(base).z - getRotationAngles(rFootFloorPoint).z;
			Vec2f x(1,0);
			Vec2f y(0,1);
			x.rotate(angle);
			y.rotate(angle);
			base.setOrientation(Quaternion(Vec(x.x,x.y,0), fusedAngle.x)
				* Quaternion(Vec(y.x,y.y,0), fusedAngle.y)
				* Quaternion(Vec(0,0,1), angle));
			base.translate(footStep.position() - rFootFloorPoint.position());
		}
		else
		{
			base.setOrientation(Quaternion(Vec(0,0,1), 0));
			double angle = getRotationAngles(footStep).z + getRotationAngles(base).z - getRotationAngles(lFootFloorPoint).z;
			Vec2f x(1,0);
			Vec2f y(0,1);
			x.rotate(angle);
			y.rotate(angle);
			base.setOrientation(Quaternion(Vec(x.x,x.y,0), fusedAngle.x)
				* Quaternion(Vec(y.x,y.y,0), fusedAngle.y)
				* Quaternion(Vec(0,0,1), angle));
			base.translate(footStep.position() - lFootFloorPoint.position());
		}
	}

	// Sets the kinematic model in a pose defined by low level joint angles.
	// It also updates the abstract kinematic and inverse kinematic pose representations.
	// All pose representations have to be consistent at all times.
	void RobotModel::setPose(const Pose& pose)
	{
		this->pose = pose;
		applyPose(pose);

		abstractPose.headPose = pose.headPose;
		abstractPose.trunkPose = pose.trunkPose;
		abstractPose.leftArmPose = armInterfaceJointToAbstract(pose.leftArmPose);
		abstractPose.rightArmPose = armInterfaceJointToAbstract(pose.rightArmPose);
		abstractPose.leftLegPose = legInterfaceJointToAbstract(pose.leftLegPose);
		abstractPose.rightLegPose = legInterfaceJointToAbstract(pose.rightLegPose);

		inversePose.headPose = pose.headPose;
		inversePose.trunkPose = pose.trunkPose;
		inversePose.leftArmPose = armInterfaceAbstractToInverse(abstractPose.leftArmPose);
		inversePose.rightArmPose = armInterfaceAbstractToInverse(abstractPose.rightArmPose);
		inversePose.leftLegPose = legInterfaceAbstractToInverse(abstractPose.leftLegPose);
		inversePose.rightLegPose = legInterfaceAbstractToInverse(abstractPose.rightLegPose);
	}

	// Adds a joint level pose to the current pose.
	void RobotModel::addPose(const Pose& pose)
	{
		setPose(this->pose + pose);
	}

	// Sets the kinematic model in an abstract pose.
	void RobotModel::setAbstractPose(const AbstractPose& abstractPose)
	{
		pose.headPose = abstractPose.headPose;
		pose.trunkPose = abstractPose.trunkPose;
		pose.leftArmPose = armInterfaceAbstractToJoint(abstractPose.leftArmPose);
		pose.rightArmPose = armInterfaceAbstractToJoint(abstractPose.rightArmPose);
		pose.leftLegPose = legInterfaceAbstractToJoint(abstractPose.leftLegPose);
		pose.rightLegPose = legInterfaceAbstractToJoint(abstractPose.rightLegPose);
		applyPose(pose);

		this->abstractPose = abstractPose;

		inversePose.headPose = abstractPose.headPose;
		inversePose.trunkPose = abstractPose.trunkPose;
		inversePose.leftArmPose = armInterfaceAbstractToInverse(abstractPose.leftArmPose);
		inversePose.rightArmPose = armInterfaceAbstractToInverse(abstractPose.rightArmPose);
		inversePose.leftLegPose = legInterfaceAbstractToInverse(abstractPose.leftLegPose);
		inversePose.rightLegPose = legInterfaceAbstractToInverse(abstractPose.rightLegPose);
	}

	// Adds a kinematic pose to the current one.
	void RobotModel::addAbstractPose(const AbstractPose& abstractPose)
	{
		setAbstractPose(this->abstractPose + abstractPose);
	}

	// Sets the kinematic model in an abstract pose.
	void RobotModel::setInversePose(const InversePose& inversePose)
	{
		this->inversePose = inversePose;

		abstractPose.headPose = inversePose.headPose;
		abstractPose.trunkPose = inversePose.trunkPose;
		abstractPose.leftArmPose = armInterfaceInverseToAbstract(inversePose.leftArmPose);
		abstractPose.rightArmPose = armInterfaceInverseToAbstract(inversePose.rightArmPose);
		abstractPose.leftLegPose = legInterfaceInverseToAbstract(inversePose.leftLegPose);
		abstractPose.rightLegPose = legInterfaceInverseToAbstract(inversePose.rightLegPose);

		pose.headPose = abstractPose.headPose;
		pose.trunkPose = abstractPose.trunkPose;

		pose.leftArmPose = armInterfaceAbstractToJoint(abstractPose.leftArmPose);
		pose.rightArmPose = armInterfaceAbstractToJoint(abstractPose.rightArmPose);
		pose.leftLegPose = legInterfaceAbstractToJoint(abstractPose.leftLegPose);
		pose.rightLegPose = legInterfaceAbstractToJoint(abstractPose.rightLegPose);
		applyPose(pose);
	}

	// Adds a kinematic pose to the current one.
	void RobotModel::addInversePose(const InversePose& inversePose)
	{
		setInversePose(this->inversePose + inversePose);
	}

	// Applies the joint angles to the model. This is used internally to avoid repetitions.
	void RobotModel::applyPose(const Pose& pose)
	{
		// The order in which rotations are applied to the joints is important.
		// We are following the mechanical servo order of NimbRo robots.
		// shoulder: y -> x
		// hip: z -> x -> y
		// ankle: y -> x

		neck.setRotation(Quaternion(Vec(0, 0, 1), pose.headPose.neck.z)
				* Quaternion(Vec(0, 1, 0), pose.headPose.neck.y));

		lShoulder.setRotation(Quaternion(Vec(0, 1, 0), pose.leftArmPose.shoulder.y)
				* Quaternion(Vec(1, 0, 0), pose.leftArmPose.shoulder.x)
				* Quaternion(Vec(0, 0, 1), pose.leftArmPose.shoulder.z));

		lElbow.setRotation(Quaternion(Vec(0, 1, 0), pose.leftArmPose.elbow.y));

		rShoulder.setRotation(Quaternion(Vec(0, 1, 0), pose.rightArmPose.shoulder.y)
				* Quaternion(Vec(1, 0, 0), pose.rightArmPose.shoulder.x)
				* Quaternion(Vec(0, 0, 1), pose.rightArmPose.shoulder.z));

		rElbow.setRotation(Quaternion(Vec(0, 1, 0), pose.rightArmPose.elbow.y));

		lHip.setRotation(Quaternion(Vec(0, 0, 1), pose.leftLegPose.hip.z)
				* Quaternion(Vec(1, 0, 0), pose.leftLegPose.hip.x)
				* Quaternion(Vec(0, 1, 0), pose.leftLegPose.hip.y));

		lKnee.setRotation(Quaternion(Vec(0, 1, 0), pose.leftLegPose.knee.y));

		lAnkle.setRotation(Quaternion(Vec(0, 1, 0), pose.leftLegPose.ankle.y)
				* Quaternion(Vec(1, 0, 0), pose.leftLegPose.ankle.x));

		rHip.setRotation(Quaternion(Vec(0, 0, 1), pose.rightLegPose.hip.z)
				* Quaternion(Vec(1, 0, 0), pose.rightLegPose.hip.x)
				* Quaternion(Vec(0, 1, 0), pose.rightLegPose.hip.y));

		rKnee.setRotation(Quaternion(Vec(0, 1, 0), pose.rightLegPose.knee.y));

		rAnkle.setRotation(Quaternion(Vec(0, 1, 0), pose.rightLegPose.ankle.y)
				* Quaternion(Vec(1, 0, 0), pose.rightLegPose.ankle.x));
	}

	// Returns the trunk rotation angles in the world reference frame.
	Vec RobotModel::trunkAngle()
	{
		return getRotationAngles(base);
	}

	// Returns the sole rotation angles in the world reference frame.
	Vec RobotModel::soleAngle()
	{
		if (supportLegSign == 1)
			return getRotationAngles(rFootFloorPoint);
		else
			return getRotationAngles(lFootFloorPoint);
	}

	// Returns the vector pointing from the support foot to the swing foot in the base coordinate frame.
	Vec RobotModel::stepVector()
	{
		Vec l = base.coordinatesOf(lFootFloorPoint.position());
		Vec r = base.coordinatesOf(rFootFloorPoint.position());
		return supportLegSign * (l - r);
	}

	// Returns the vector from the support foot to the base in the base coordinate frame.
	Vec RobotModel::comVector()
	{
		if (supportLegSign == 1)
			return rFootFloorPoint.coordinatesOf(base.position());
		else
			return lFootFloorPoint.coordinatesOf(base.position());
	}

	// Returns a complete action struct with the pose that the model has currently struck.
	// Pose, abstract pose and inverse kinematics pose are consistent.
	Action RobotModel::getAction()
	{
		Action action;
		action.inversePose = inversePose;
		action.abstractPose = abstractPose;
		action.pose = pose;
		return action;
	}


	// Inverse Kinematic Arm Interface. Computes an abstract arm pose from a Cartesian end effector pose.
	AbstractArmPose RobotModel::armInterfaceInverseToAbstract(const InverseArmPose& iap)
	{
		// First lame version: confined on the xz plane.

		// Project the target onto the xz plane.
		Vec target(iap.handPosition.x, 0, iap.handPosition.z - armLength);

		AbstractArmPose aap;
		aap.compliance = iap.compliance;
		aap.armExtension = qBound(0.0, 1.0-target.norm()/armLength, 1.0);

		target.setValue(iap.handPosition.x, 0, iap.handPosition.z - armLength);
		target.projectOnPlane(Vec(0,1,0));
		aap.armAngle.y = -atan2(target.x, -target.z);

		return aap;
	}

	// Inverse Kinematic Arm Interface inverse :). Computes a Cartesian end effector pose from an abstract pose.
	InverseArmPose RobotModel::armInterfaceAbstractToInverse(const AbstractArmPose& aap)
	{
		InverseArmPose iap;

		Quaternion qy(Vec(0,1,0), aap.armAngle.y);
		Quaternion qx(qy * Vec(1,0,0), aap.armAngle.x);
		Quaternion qz(qx * qy * Vec(0,0,1), aap.armAngle.z);

		Vec target(0, 0, -qBound(0.0, 1.0-aap.armExtension, 1.0)*armLength);
		target = qz * qx * qy * target;

		iap.handPosition.x = target.x;
		iap.handPosition.y = target.y;
		iap.handPosition.z = target.z + armLength;

		iap.compliance = aap.compliance;

		return iap;
	}

	// Abstract Arm Interface. Computes joint angles from an abstract pose.
	ArmPose RobotModel::armInterfaceAbstractToJoint(const AbstractArmPose& aap)
	{
		double alpha = acos(qBound(0.0, 1.0-aap.armExtension, 1.0));

		ArmPose ap;
		ap.shoulder.x = aap.armAngle.x;
		ap.shoulder.y = aap.armAngle.y + alpha;
		ap.shoulder.z = aap.armAngle.z;
		ap.elbow.y = -2.0*alpha;
		ap.shoulder.compliance = aap.compliance;
		ap.elbow.compliance = aap.compliance;

		return ap;
	}

	// Abstract Arm Interface inverse. Calculates an abstract pose from joint angles.
	AbstractArmPose RobotModel::armInterfaceJointToAbstract(const ArmPose& ap)
	{
		AbstractArmPose aap;
		aap.armExtension = 1.0-cos(0.5*ap.elbow.y);
		aap.armAngle.x = ap.shoulder.x;
		aap.armAngle.y = ap.shoulder.y + 0.5*ap.elbow.y;
		aap.armAngle.z = ap.shoulder.z;
		aap.compliance = qMax(ap.shoulder.compliance, ap.elbow.compliance);
		return aap;
	}


	// Inverse Kinematic Leg Interface. Computes an abstract leg pose from a Cartesian end effector position.
	AbstractLegPose RobotModel::legInterfaceInverseToAbstract(const InverseLegPose& ilp)
	{
		// The target vector is defined such that 0 equals the 0 position of the robot.
		Vec target(ilp.footPosition.x, ilp.footPosition.y, ilp.footPosition.z - legLength);

		AbstractLegPose alp;
		alp.legExtension = qBound(0.0, 1.0-target.norm()/legLength, 1.0);
		alp.legAngle.z = ilp.footAngle.z;

		// Rotate the target around the z axis by the leg angle z.
		Vec2f v(target.x, target.y);
		v.rotate(-alp.legAngle.z);
		target.x = v.x;
		target.y = v.y;

		// Compute the leg angles using spherical coordinates.
		alp.legAngle.x = atan2(target.y, -target.z);
		alp.legAngle.y = target.norm() > 0 ? -asin(target.x/target.norm()) : 0;

		// Rotate the foot angle by the leg angle z.
		// This way the foot angle is expressed relative to the trunk.
		// Including a pitch angle correction due to the skewed pitch joint.
		Vec2f w(ilp.footAngle.x, ilp.footAngle.y);
		w.rotate(alp.legAngle.z);
		alp.footAngle.x = w.x;
		alp.footAngle.y = w.y / cos(alp.legAngle.x);

		// But the leg angle x changes the orientation of the pitch axis.
		// So the foot angle y will change the foot angle z as well.
		// Compensate it with adjusting the leg angle z.
		// But that changes the foot angle vector again.
		// So everything becomes recursive and slower.
		// For now I will just accept the error.

		alp.compliance = ilp.compliance;

		return alp;
	}


	// Inverse Kinematic Leg Interface inverse :). Computes a Cartesian end effector position from an abstract pose.
	InverseLegPose RobotModel::legInterfaceAbstractToInverse(const AbstractLegPose& alp)
	{
		InverseLegPose ilp;

		Quaternion qz(Vec(0,0,1), alp.legAngle.z);
		Quaternion qx(qz * Vec(1,0,0), alp.legAngle.x);
		Quaternion qy(qx * qz * Vec(0,1,0), alp.legAngle.y);

		Vec target(0, 0, -qBound(0.0, 1.0-alp.legExtension, 1.0)*legLength);
		target = qy * qx * qz * target;

		ilp.footPosition.x = target.x;
		ilp.footPosition.y = target.y;
		ilp.footPosition.z = target.z + legLength;

		// Rotate the foot angle by the leg angle z.
		// This way the foot angle is expressed relative to the trunk.
		// Including a pitch angle correction due to the skewed pitch joint.
		Vec2f w(alp.footAngle.x, alp.footAngle.y * cos(alp.legAngle.x));
		w.rotate(-alp.legAngle.z);
		ilp.footAngle.x = w.x;
		ilp.footAngle.y = w.y;
		ilp.footAngle.z = alp.legAngle.z;

		ilp.compliance = alp.compliance;

		return ilp;
	}

	// Abstract Leg Interface. Computes joint angles from an abstract pose.
	LegPose RobotModel::legInterfaceAbstractToJoint(const AbstractLegPose& alp)
	{
		double alpha = acos(qBound(0.0, 1.0-alp.legExtension, 1.0));

		LegPose lp;
		lp.hip.z = alp.legAngle.z;
		lp.hip.x = alp.legAngle.x;
		lp.hip.y = alp.legAngle.y - alpha;
		lp.knee.y = 2.0*alpha;
		lp.ankle.y = alp.footAngle.y;
		lp.ankle.x = alp.footAngle.x;

		lp.ankle.y += -alp.legAngle.y - alpha;
		lp.ankle.x += -alp.legAngle.x;

		lp.hip.compliance = alp.compliance;
		lp.knee.compliance = alp.compliance;
		lp.ankle.compliance = alp.compliance;

		return lp;
	}

	// Abstract Leg Interface inverse. Computes an abstract pose from joint angles.
	AbstractLegPose RobotModel::legInterfaceJointToAbstract(const LegPose& lp)
	{
		AbstractLegPose alp;
		alp.legExtension = 1.0-cos(0.5*lp.knee.y);
		alp.legAngle.x = lp.hip.x;
		alp.legAngle.y = lp.hip.y + 0.5*lp.knee.y;
		alp.legAngle.z = lp.hip.z;
		alp.footAngle.x = lp.hip.x + lp.ankle.x;
		alp.footAngle.y = lp.hip.y + lp.ankle.y + lp.knee.y;
		alp.compliance = qMax(qMax(lp.hip.compliance, lp.knee.compliance), lp.ankle.compliance);
		return alp;
	}


	// Returns the orthogonal extrinsic rotation angles of the frame with respect to the world coordinate frame.
	// Orthogonal angles are not Euler angles. They are measured with respect to a fixed coordinate frame.
	// The angle around the x-axis is measured by projecting the rotated z-axis (or y) into the plane that has
	// the world x-axis as normal. Then, the angle is calculated between the projected vector and the world
	// z-axis (or y). This is repeated for each axes respectively.
	Vec RobotModel::getRotationAngles(const Frame& frame)
	{
		Vec absoluteAngle;
		Vec v;

		// The "up" vector.
		Vec up = frame.orientation() * Vec(0,0,1);

		v = up;
		v.projectOnPlane(Vec(1,0,0));
		absoluteAngle.x = atan2(v.y, v.z);

		v = up;
		v.projectOnPlane(Vec(0,1,0));
		absoluteAngle.y = atan2(v.x, v.z);

		// The "front" vector.
		v = frame.orientation() * Vec(1,0,0);
		v.projectOnPlane(Vec(0,0,1));
		absoluteAngle.z = atan2(v.y, v.x);

		return absoluteAngle;
	}

	// Loads the calib file and writes the result into the signs, offsets, upperLimits, and lowerLimits data structures.
	// The name has to be set correctly for this to work.
	void RobotModel::loadCalib()
	{
		// Open the file.
		QFile file;
		file.setFileName("conf/" + name + ".calib");
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
			qDebug() << "Could not open file" << file.fileName();
		QTextStream in(&file);


		// Read the file line by line.
		QString line;
		QStringList list;
		bool ok;
		while (!in.atEnd())
		{
			line = in.readLine();

			// Ignore comments and empty lines.
			if (line.startsWith('#') or not line.contains('='))
				continue;

			list = line.split("=");
			QString jointName = list[0].trimmed();

			list = list[1].split(" ", QString::SkipEmptyParts);
			int error = 0;
			double sign = list[0].toDouble(&ok); error += ok;
			double offset = list[1].toDouble(&ok); error += ok;
			double upperLimit = list[2].toDouble(&ok); error += ok;
			double lowerLimit = list[3].toDouble(&ok); error += ok;
			if (error != 4)
				qDebug() << "Something could not be parsed in " << file.fileName() << "joint " << jointName;

			if (jointName == "neck.x")
			{
				signs.headPose.neck.x = sign;
				offsets.headPose.neck.x = offset;
				upperLimits.headPose.neck.x = upperLimit;
				lowerLimits.headPose.neck.x = lowerLimit;
			}
			if (jointName == "neck.y")
			{
				signs.headPose.neck.y = sign;
				offsets.headPose.neck.y = offset;
				upperLimits.headPose.neck.y = upperLimit;
				lowerLimits.headPose.neck.y = lowerLimit;
			}
			if (jointName == "neck.z")
			{
				signs.headPose.neck.z = sign;
				offsets.headPose.neck.z = offset;
				upperLimits.headPose.neck.z = upperLimit;
				lowerLimits.headPose.neck.z = lowerLimit;
			}

			if (jointName == "leftShoulder.x")
			{
				signs.leftArmPose.shoulder.x = sign;
				offsets.leftArmPose.shoulder.x = offset;
				upperLimits.leftArmPose.shoulder.x = upperLimit;
				lowerLimits.leftArmPose.shoulder.x = lowerLimit;
			}
			if (jointName == "leftShoulder.y")
			{
				signs.leftArmPose.shoulder.y = sign;
				offsets.leftArmPose.shoulder.y = offset;
				upperLimits.leftArmPose.shoulder.y = upperLimit;
				lowerLimits.leftArmPose.shoulder.y = lowerLimit;
			}
			if (jointName == "leftShoulder.z")
			{
				signs.leftArmPose.shoulder.z = sign;
				offsets.leftArmPose.shoulder.z = offset;
				upperLimits.leftArmPose.shoulder.z = upperLimit;
				lowerLimits.leftArmPose.shoulder.z = lowerLimit;
			}
			if (jointName.startsWith("leftElbow"))
			{
				signs.leftArmPose.elbow.y = sign;
				offsets.leftArmPose.elbow.y = offset;
				upperLimits.leftArmPose.elbow.y = upperLimit;
				lowerLimits.leftArmPose.elbow.y = lowerLimit;
			}

			if (jointName == "rightShoulder.x")
			{
				signs.rightArmPose.shoulder.x = sign;
				offsets.rightArmPose.shoulder.x = offset;
				upperLimits.rightArmPose.shoulder.x = upperLimit;
				lowerLimits.rightArmPose.shoulder.x = lowerLimit;
			}
			if (jointName == "rightShoulder.y")
			{
				signs.rightArmPose.shoulder.y = sign;
				offsets.rightArmPose.shoulder.y = offset;
				upperLimits.rightArmPose.shoulder.y = upperLimit;
				lowerLimits.rightArmPose.shoulder.y = lowerLimit;
			}
			if (jointName == "rightShoulder.z")
			{
				signs.rightArmPose.shoulder.z = sign;
				offsets.rightArmPose.shoulder.z = offset;
				upperLimits.rightArmPose.shoulder.z = upperLimit;
				lowerLimits.rightArmPose.shoulder.z = lowerLimit;
			}
			if (jointName.startsWith("rightElbow"))
			{
				signs.rightArmPose.elbow.y = sign;
				offsets.rightArmPose.elbow.y = offset;
				upperLimits.rightArmPose.elbow.y = upperLimit;
				lowerLimits.rightArmPose.elbow.y = lowerLimit;
			}

			if (jointName == "leftHip.x")
			{
				signs.leftLegPose.hip.x = sign;
				offsets.leftLegPose.hip.x = offset;
				upperLimits.leftLegPose.hip.x = upperLimit;
				lowerLimits.leftLegPose.hip.x = lowerLimit;
			}
			if (jointName == "leftHip.y")
			{
				signs.leftLegPose.hip.y = sign;
				offsets.leftLegPose.hip.y = offset;
				upperLimits.leftLegPose.hip.y = upperLimit;
				lowerLimits.leftLegPose.hip.y = lowerLimit;
			}
			if (jointName == "leftHip.z")
			{
				signs.leftLegPose.hip.z = sign;
				offsets.leftLegPose.hip.z = offset;
				upperLimits.leftLegPose.hip.z = upperLimit;
				lowerLimits.leftLegPose.hip.z = lowerLimit;
			}
			if (jointName == "leftKnee")
			{
				signs.leftLegPose.knee.y = sign;
				offsets.leftLegPose.knee.y = offset;
				upperLimits.leftLegPose.knee.y = upperLimit;
				lowerLimits.leftLegPose.knee.y = lowerLimit;
			}
			if (jointName == "leftAnkle.x")
			{
				signs.leftLegPose.ankle.x = sign;
				offsets.leftLegPose.ankle.x = offset;
				upperLimits.leftLegPose.ankle.x = upperLimit;
				lowerLimits.leftLegPose.ankle.x = lowerLimit;
			}
			if (jointName == "leftAnkle.y")
			{
				signs.leftLegPose.ankle.y = sign;
				offsets.leftLegPose.ankle.y = offset;
				upperLimits.leftLegPose.ankle.y = upperLimit;
				lowerLimits.leftLegPose.ankle.y = lowerLimit;
			}

			if (jointName == "rightHip.x")
			{
				signs.rightLegPose.hip.x = sign;
				offsets.rightLegPose.hip.x = offset;
				upperLimits.rightLegPose.hip.x = upperLimit;
				lowerLimits.rightLegPose.hip.x = lowerLimit;
			}
			if (jointName == "rightHip.y")
			{
				signs.rightLegPose.hip.y = sign;
				offsets.rightLegPose.hip.y = offset;
				upperLimits.rightLegPose.hip.y = upperLimit;
				lowerLimits.rightLegPose.hip.y = lowerLimit;
			}
			if (jointName == "rightHip.z")
			{
				signs.rightLegPose.hip.z = sign;
				offsets.rightLegPose.hip.z = offset;
				upperLimits.rightLegPose.hip.z = upperLimit;
				lowerLimits.rightLegPose.hip.z = lowerLimit;
			}
			if (jointName == "rightKnee")
			{
				signs.rightLegPose.knee.y = sign;
				offsets.rightLegPose.knee.y = offset;
				upperLimits.rightLegPose.knee.y = upperLimit;
				lowerLimits.rightLegPose.knee.y = lowerLimit;
			}
			if (jointName == "rightAnkle.x")
			{
				signs.rightLegPose.ankle.x = sign;
				offsets.rightLegPose.ankle.x = offset;
				upperLimits.rightLegPose.ankle.x = upperLimit;
				lowerLimits.rightLegPose.ankle.x = lowerLimit;
			}
			if (jointName == "rightAnkle.y")
			{
				signs.rightLegPose.ankle.y = sign;
				offsets.rightLegPose.ankle.y = offset;
				upperLimits.rightLegPose.ankle.y = upperLimit;
				lowerLimits.rightLegPose.ankle.y = lowerLimit;
			}
		}
	}

	void RobotModel::draw(float r, float g, float b, float a)
	{
// 		// Draw the inverse kinematic targets.
// 		glPushMatrix();
// 		glMultMatrixd(base.worldMatrix());
// 		glTranslatef(lShoulder.translation().x,
// 				lShoulder.translation().y,
// 				lShoulder.translation().z);
// 		glTranslatef(0, 0, -armLength);
// 		glTranslatef(inversePose.leftArmPose.handPosition.x,
// 				inversePose.leftArmPose.handPosition.y,
// 				inversePose.leftArmPose.handPosition.z);
// 		glColor3f(0, 0, 1);
// 		GLlib::drawSphere(0.05);
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(base.worldMatrix());
// 		glTranslatef(rShoulder.translation().x,
// 				rShoulder.translation().y,
// 				rShoulder.translation().z);
// 		glTranslatef(0, 0, -armLength);
// 		glTranslatef(inversePose.rightArmPose.handPosition.x,
// 				inversePose.rightArmPose.handPosition.y,
// 				inversePose.rightArmPose.handPosition.z);
// 		glColor3f(1, 0, 0);
// 		GLlib::drawSphere(0.05);
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(base.worldMatrix());
// 		glTranslatef(lHip.translation().x,
// 				lHip.translation().y,
// 				lHip.translation().z);
// 		glTranslatef(0, 0, -legLength);
// 		glTranslatef(inversePose.leftLegPose.footPosition.x,
// 				inversePose.leftLegPose.footPosition.y,
// 				inversePose.leftLegPose.footPosition.z);
// 		glColor3f(0, 0, 1);
// 		GLlib::drawSphere(0.05);
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(base.worldMatrix());
// 		glTranslatef(rHip.translation().x,
// 				rHip.translation().y,
// 				rHip.translation().z);
// 		glTranslatef(0, 0, -legLength);
// 		glTranslatef(inversePose.rightLegPose.footPosition.x,
// 				inversePose.rightLegPose.footPosition.y,
// 				inversePose.rightLegPose.footPosition.z);
// 		glColor3f(1, 0, 0);
// 		GLlib::drawSphere(0.05);
// 		glPopMatrix();
// 
// 
// 		// The bones.
// 		glLineWidth(4);
// 		glColor3f(0.2, 0.2, 0.2);
// 		glBegin( GL_LINES );
// 		glVertex3fv(base.position());
// 		glVertex3fv(neck.position());
// 		glVertex3fv(lShoulder.position());
// 		glVertex3fv(rShoulder.position());
// 		glVertex3fv(lShoulder.position());
// 		glVertex3fv(lElbow.position());
// 		glVertex3fv(lElbow.position());
// 		glVertex3fv(lHand.position());
// 		glVertex3fv(rShoulder.position());
// 		glVertex3fv(rElbow.position());
// 		glVertex3fv(rElbow.position());
// 		glVertex3fv(rHand.position());
// 		glVertex3fv(base.position());
// 		glVertex3fv(lHip.position());
// 		glVertex3fv(base.position());
// 		glVertex3fv(rHip.position());
// 		glVertex3fv(lHip.position());
// 		glVertex3fv(lKnee.position());
// 		glVertex3fv(rHip.position());
// 		glVertex3fv(rKnee.position());
// 		glVertex3fv(lKnee.position());
// 		glVertex3fv(lAnkle.position());
// 		glVertex3fv(rKnee.position());
// 		glVertex3fv(rAnkle.position());
// 		glVertex3fv(lAnkle.position());
// 		glVertex3fv(lFootFloorPoint.position());
// 		glVertex3fv(rAnkle.position());
// 		glVertex3fv(rFootFloorPoint.position());
// 		glEnd();
// 		glLineWidth(1);
// 
// 
// 		// Now draw all the joints.
// 
// 		// BASE
// 		glPushMatrix();
// 		glMultMatrixd(base.worldMatrix());
// 		drawJoint();
// 		glPopMatrix();
// 
// 		// HEAD
// 		glPushMatrix();
// 		glMultMatrixd(neck.worldMatrix());
// 		drawJoint(pose.headPose.neck.temperature.maxComponent());
// 		glPopMatrix();
// 
// 		// LEFT ARM
// 		glPushMatrix();
// 		glMultMatrixd(lShoulder.worldMatrix());
// 		drawJoint(pose.leftArmPose.shoulder.temperature.maxComponent());
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(lElbow.worldMatrix());
// 		drawJoint(pose.leftArmPose.elbow.temperature.y);
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(lHand.worldMatrix());
// 		drawJoint();
// 		glPopMatrix();
// 
// 		// RIGHT ARM
// 		glPushMatrix();
// 		glMultMatrixd(rShoulder.worldMatrix());
// 		drawJoint(pose.rightArmPose.shoulder.temperature.maxComponent());
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(rElbow.worldMatrix());
// 		drawJoint(pose.rightArmPose.elbow.temperature.y);
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(rHand.worldMatrix());
// 		drawJoint();
// 		glPopMatrix();
// 
// 		// LEFT LEG
// 		glPushMatrix();
// 		glMultMatrixd(lHip.worldMatrix());
// 		drawJoint(pose.leftLegPose.hip.temperature.maxComponent());
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(lKnee.worldMatrix());
// 		drawJoint(pose.leftLegPose.knee.temperature.y);
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(lAnkle.worldMatrix());
// 		drawJoint(pose.leftLegPose.ankle.temperature.maxComponent());
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(lFootFloorPoint.worldMatrix());
// 		drawJoint();
// 		glPopMatrix();
// 
// 		// RIGHT LEG
// 		glPushMatrix();
// 		glMultMatrixd(rHip.worldMatrix());
// 		drawJoint(pose.rightLegPose.hip.temperature.maxComponent());
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(rKnee.worldMatrix());
// 		drawJoint(pose.rightLegPose.knee.temperature.y);
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(rAnkle.worldMatrix());
// 		drawJoint(pose.rightLegPose.ankle.temperature.maxComponent());
// 		glPopMatrix();
// 
// 		glPushMatrix();
// 		glMultMatrixd(rFootFloorPoint.worldMatrix());
// 		drawJoint();
// 		glPopMatrix();
// 
// 
// 		// Now in a second sweep, draw the semi transparent hull.
// 
// 		// left foot
// 		glPushMatrix();
// 		glMultMatrixd(lFootFloorPoint.worldMatrix());
// 		glTranslatef(footOffsetX, 0, 0.5*footHeight);
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*footLength, 0.5*footWidth, 0.5*footHeight);
// 		glPopMatrix();
// 
// 		// right foot
// 		glPushMatrix();
// 		glMultMatrixd(rFootFloorPoint.worldMatrix());
// 		glTranslatef(footOffsetX, 0, 0.5*footHeight);
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*footLength, 0.5*footWidth, 0.5*footHeight);
// 		glPopMatrix();
// 
// 		// left ankle
// 		glPushMatrix();
// 		glMultMatrixd(lAnkle.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// right ankle
// 		glPushMatrix();
// 		glMultMatrixd(rAnkle.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// left shank
// 		glPushMatrix();
// 		glMultMatrixd(lKnee.worldMatrix());
// 		glTranslatef(0, 0, -(jointRadius+0.5*legSegmentLength));
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*legThickness, 0.5*legThickness, 0.5*legSegmentLength);
// 		glPopMatrix();
// 
// 		// right shank
// 		glPushMatrix();
// 		glMultMatrixd(rKnee.worldMatrix());
// 		glTranslatef(0, 0, -(jointRadius+0.5*legSegmentLength));
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*legThickness, 0.5*legThickness, 0.5*legSegmentLength);
// 		glPopMatrix();
// 
// 		// left knee
// 		glPushMatrix();
// 		glMultMatrixd(lKnee.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// right knee
// 		glPushMatrix();
// 		glMultMatrixd(rKnee.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// left thigh
// 		glPushMatrix();
// 		glMultMatrixd(lHip.worldMatrix());
// 		glTranslatef(0, 0, -(jointRadius+0.5*legSegmentLength));
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*legThickness, 0.5*legThickness, 0.5*legSegmentLength);
// 		glPopMatrix();
// 
// 		// right thigh
// 		glPushMatrix();
// 		glMultMatrixd(rHip.worldMatrix());
// 		glTranslatef(0, 0, -(jointRadius+0.5*legSegmentLength));
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*legThickness, 0.5*legThickness, 0.5*legSegmentLength);
// 		glPopMatrix();
// 
// 		// left hip
// 		glPushMatrix();
// 		glMultMatrixd(lHip.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// right hip
// 		glPushMatrix();
// 		glMultMatrixd(rHip.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// left lower arm
// 		glPushMatrix();
// 		glMultMatrixd(lHand.worldMatrix());
// 		glTranslatef(0, 0, 0.5*armSegmentLength);
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*armThickness, 0.5*armThickness, 0.5*armSegmentLength);
// 		glPopMatrix();
// 
// 		// right lower arm
// 		glPushMatrix();
// 		glMultMatrixd(rHand.worldMatrix());
// 		glTranslatef(0, 0, 0.5*armSegmentLength);
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*armThickness, 0.5*armThickness, 0.5*armSegmentLength);
// 		glPopMatrix();
// 
// 		// left elbow
// 		glPushMatrix();
// 		glMultMatrixd(lElbow.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// right elbow
// 		glPushMatrix();
// 		glMultMatrixd(rElbow.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// left upper arm
// 		glPushMatrix();
// 		glMultMatrixd(lShoulder.worldMatrix());
// 		glTranslatef(0, 0, -(jointRadius + 0.5*armSegmentLength));
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*armThickness, 0.5*armThickness, 0.5*armSegmentLength);
// 		glPopMatrix();
// 
// 		// right upper arm
// 		glPushMatrix();
// 		glMultMatrixd(rShoulder.worldMatrix());
// 		glTranslatef(0, 0, -(jointRadius + 0.5*armSegmentLength));
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(0.5*armThickness, 0.5*armThickness, 0.5*armSegmentLength);
// 		glPopMatrix();
// 
// 		// left shoulder
// 		glPushMatrix();
// 		glMultMatrixd(lShoulder.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// right shoulder
// 		glPushMatrix();
// 		glMultMatrixd(rShoulder.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// trunk
// 		glPushMatrix();
// 		glMultMatrixd(base.worldMatrix());
// 		glTranslatef(0, 0, 0.5*trunkHeight);
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBox(legThickness, 0.5*(hipWidth+legThickness), 0.5*trunkHeight);
// 		glPopMatrix();
// 
// 		// neck
// 		glPushMatrix();
// 		glMultMatrixd(neck.worldMatrix());
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(jointRadius);
// 		glPopMatrix();
// 
// 		// head
// 		glPushMatrix();
// 		glMultMatrixd(neck.worldMatrix());
// 		glTranslatef(0, 0, jointRadius + headRadius);
// 		glColor4f(r, g, b, a);
// 		GLlib::drawBorderedSphere(headRadius);
// 		glPopMatrix();

	}

	void RobotModel::drawJoint(double temperature)
	{
// 		static GLUquadric* quadric = gluNewQuadric();

// 		Vec3f colorCold(0.5, 0.5, 0.5);
// 		Vec3f colorHot(1.0, 0.2, 0.2);
// 		Vec3f color;
// 		double alpha = 0;
// 
// 		alpha = qBound(0.0, (temperature - 35.0) / (70.0 - 35.0), 1.0);
// 		color = colorCold + alpha*(colorHot-colorCold);
// 
// 		glColor3f(color.x, color.y, color.z);
// 		gluSphere(quadric, 0.01+alpha*0.02, 32, 32);
// 
// 	//	glColor4f(color.x, color.y, color.z, alpha);
// 	//	gluSphere(quadric, 0.016, 32, 32);
// 
// 		GLlib::drawFrame(0.05);
	}
}