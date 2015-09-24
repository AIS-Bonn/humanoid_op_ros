#ifndef ACTION_H_
#define ACTION_H_

#include "util/Vec3f.h"
#include "util/Vec2f.h"

namespace indep_cpg_gait
{
	// The joint config is the deepest level data structure that describes the configuration of one joint.
	struct JointConfig
	{
		double x;
		double y;
		double z;
		double compliance;
		Vec3f velocity;
		Vec3f torque;
		Vec3f temperature;

		JointConfig()
		{
			x = 0;
			y = 0;
			z = 0;
			compliance = 1.0;
		}

		JointConfig(double x, double y, double z, double compliance = 1.0)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->compliance = compliance;
		}

		inline JointConfig operator+(const JointConfig& v) const {return JointConfig(x+v.x, y+v.y, z+v.z, compliance);}
		inline JointConfig operator-() const {return JointConfig(-x, -y, -z, compliance);}
		inline JointConfig operator-(const JointConfig& v) const {return JointConfig(x-v.x, y-v.y, z-v.z, compliance);}
		inline JointConfig operator+=(const JointConfig& v) {x+=v.x; y+=v.y; z+=v.z; return *this;}
		inline JointConfig operator-=(const JointConfig& v) {x-=v.x; y-=v.y; z-=v.z; return *this;}
		inline JointConfig& operator*=(const double scalar) {x*=scalar; y*=scalar; z*=scalar; return *this;}
		inline JointConfig& operator/=(const double scalar) {x/=scalar; y/=scalar; z/=scalar; return *this;}
		inline bool operator==(const JointConfig& v) const {return (x==v.x) && (y==v.y) && (z==v.z) && (velocity==v.velocity);}
		inline bool operator!=(const JointConfig& v) const {return (x!=v.x) || (y!=v.y) || (z!=v.z) || (velocity!=v.velocity);}
	};

	// The arm pose describes joint angles for one arm.
	struct ArmPose
	{
		JointConfig shoulder;
		JointConfig elbow;

		ArmPose()
		{
		}

		ArmPose(const JointConfig& shoulder, const JointConfig& elbow)
		{
			this->shoulder = shoulder;
			this->elbow = elbow;
		}

		inline ArmPose operator+(const ArmPose& v) const {return ArmPose(shoulder+v.shoulder, elbow+v.elbow);}
		inline ArmPose operator-() const {return ArmPose(-shoulder, -elbow);}
		inline ArmPose operator-(const ArmPose& v) const {return ArmPose(shoulder-v.shoulder, elbow-v.elbow);}
		inline ArmPose operator+=(const ArmPose& v) {shoulder+=v.shoulder; elbow+=v.elbow; return *this;}
		inline ArmPose operator-=(const ArmPose& v) {shoulder-=v.shoulder; elbow-=v.elbow; return *this;}
		inline ArmPose& operator*=(const double scalar) {shoulder*=scalar; elbow*=scalar; return *this;}
		inline ArmPose& operator/=(const double scalar) {shoulder/=scalar; elbow/=scalar; return *this;}
		inline bool operator==(const ArmPose& v) const {return (shoulder==v.shoulder) && (elbow==v.elbow);}
		inline bool operator!=(const ArmPose& v) const {return (shoulder!=v.shoulder) || (elbow!=v.elbow);}

	};

	// The abstract arm pose describes an abstract end effector position for the arm.
	struct AbstractArmPose
	{
		Vec3f armAngle;
		double armExtension;
		double compliance;

		AbstractArmPose()
		{
			armExtension = 0.0;
			compliance = 1.0;
		}

		AbstractArmPose(const Vec3f& armAngle, double armExtension, double compliance = 1.0)
		{
			this->armAngle = armAngle;
			this->armExtension = armExtension;
			this->compliance = compliance;
		}

		inline AbstractArmPose operator+(const AbstractArmPose& v) const {return AbstractArmPose(armAngle+v.armAngle, armExtension+v.armExtension, compliance);}
		inline AbstractArmPose operator-() const {return AbstractArmPose(-armAngle, -armExtension, compliance);}
		inline AbstractArmPose operator-(const AbstractArmPose& v) const {return AbstractArmPose(armAngle-v.armAngle, armExtension-v.armExtension, compliance);}
		inline AbstractArmPose operator+=(const AbstractArmPose& v) {armAngle+=v.armAngle; armExtension+=v.armExtension; return *this;}
		inline AbstractArmPose operator-=(const AbstractArmPose& v) {armAngle-=v.armAngle; armExtension-=v.armExtension; return *this;}
		inline AbstractArmPose& operator*=(const double scalar){armAngle*=scalar; armExtension*=scalar; return *this;}
		inline AbstractArmPose& operator/=(const double scalar){armAngle/=scalar; armExtension/=scalar; return *this;}
		inline bool operator==(const AbstractArmPose& v) const {return (armAngle==v.armAngle) && (armExtension==v.armExtension);}
		inline bool operator!=(const AbstractArmPose& v) const {return (armAngle!=v.armAngle) || (armExtension!=v.armExtension);}
	};

	// The inverse arm pose describes an end effector position in Cartesian space for the arm.
	struct InverseArmPose
	{
		Vec3f handPosition;
		double compliance;

		InverseArmPose()
		{
			compliance = 1.0;
		}

		InverseArmPose(const Vec3f& handPosition, double compliance = 1.0)
		{
			this->handPosition = handPosition;
			this->compliance = compliance;
		}

		inline InverseArmPose operator+(const InverseArmPose& v) const {return InverseArmPose(handPosition+v.handPosition, compliance);}
		inline InverseArmPose operator-() const {return InverseArmPose(-handPosition, compliance);}
		inline InverseArmPose operator-(const InverseArmPose& v) const {return InverseArmPose(handPosition-v.handPosition, compliance);}
		inline InverseArmPose operator+=(const InverseArmPose& v) {handPosition+=v.handPosition; return *this;}
		inline InverseArmPose operator-=(const InverseArmPose& v) {handPosition-=v.handPosition; return *this;}
		inline InverseArmPose& operator*=(const double scalar){handPosition*=scalar; return *this;}
		inline InverseArmPose& operator/=(const double scalar){handPosition/=scalar; return *this;}
		inline bool operator==(const InverseArmPose& v) const {return (handPosition==v.handPosition);}
		inline bool operator!=(const InverseArmPose& v) const {return (handPosition!=v.handPosition);}
	};

	// The leg pose describes joint angles for one leg.
	struct LegPose
	{
		JointConfig hip;
		JointConfig knee;
		JointConfig ankle;

		LegPose()
		{
		}

		LegPose(const JointConfig& hip, const JointConfig& knee, const JointConfig& ankle)
		{
			this->hip = hip;
			this->knee = knee;
			this->ankle = ankle;
		}

		inline LegPose operator+(const LegPose& v) const {return LegPose(hip+v.hip, knee+v.knee, ankle+v.ankle);}
		inline LegPose operator-() const {return LegPose(-hip, -knee, -ankle);}
		inline LegPose operator-(const LegPose& v) const {return LegPose(hip-v.hip, knee-v.knee, ankle-v.ankle);}
		inline LegPose operator+=(const LegPose& v) {hip+=v.hip; knee+=v.knee; ankle+=v.ankle; return *this;}
		inline LegPose operator-=(const LegPose& v) {hip-=v.hip; knee-=v.knee; ankle-=v.ankle; return *this;}
		inline LegPose& operator*=(const double scalar) {hip*=scalar; knee*=scalar; ankle*=scalar; return *this;}
		inline LegPose& operator/=(const double scalar) {hip/=scalar; knee/=scalar; ankle/=scalar; return *this;}
		inline bool operator==(const LegPose& v) const {return (hip==v.hip) && (knee==v.knee) && (ankle==v.ankle);}
		inline bool operator!=(const LegPose& v) const {return (hip!=v.hip) || (knee!=v.knee) || (ankle!=v.ankle);}
	};

	// The abstract leg pose describes an abstract end effector position for the leg.
	struct AbstractLegPose
	{
		Vec3f legAngle;
		double legExtension;
		Vec2f footAngle;
		double compliance;

		AbstractLegPose()
		{
			legExtension = 0.0;
			compliance = 1.0;
		}

		AbstractLegPose(const Vec3f& legAngle, double legExtension, const Vec2f& footAngle, double compliance = 1.0)
		{
			this->legAngle = legAngle;
			this->legExtension = legExtension;
			this->footAngle = footAngle;
			this->compliance = compliance;
		}

		inline AbstractLegPose operator+(const AbstractLegPose& v) const {return AbstractLegPose(legAngle+v.legAngle, legExtension+v.legExtension, footAngle+v.footAngle, compliance);}
		inline AbstractLegPose operator-() const {return AbstractLegPose(-legAngle, -legExtension, -footAngle, compliance);}
		inline AbstractLegPose operator-(const AbstractLegPose& v) const {return AbstractLegPose(legAngle-v.legAngle, legExtension-v.legExtension, footAngle-v.footAngle, compliance);}
		inline AbstractLegPose operator+=(const AbstractLegPose& v) {legAngle+=v.legAngle; legExtension+=v.legExtension; footAngle+=v.footAngle; return *this;}
		inline AbstractLegPose operator-=(const AbstractLegPose& v) {legAngle-=v.legAngle; legExtension-=v.legExtension; footAngle-=v.footAngle; return *this;}
		inline AbstractLegPose& operator*=(const double scalar){legAngle*=scalar; legExtension*=scalar; footAngle*=scalar; return *this;}
		inline AbstractLegPose& operator/=(const double scalar){legAngle/=scalar; legExtension/=scalar; footAngle/=scalar; return *this;}
		inline bool operator==(const AbstractLegPose& v) const {return (legAngle==v.legAngle) && (legExtension==v.legExtension) && (footAngle==v.footAngle);}
		inline bool operator!=(const AbstractLegPose& v) const {return (legAngle!=v.legAngle) || (legExtension!=v.legExtension) || (footAngle!=v.footAngle);}
	};

	// The inverse leg pose describes an end effector position in Cartesian space for the leg.
	struct InverseLegPose
	{
		Vec3f footPosition;
		Vec3f footAngle;
		double compliance;

		InverseLegPose()
		{
			compliance = 1.0;
		}

		InverseLegPose(const Vec3f& footPosition, const Vec3f& footAngle, double compliance = 1.0)
		{
			this->footPosition = footPosition;
			this->footAngle = footAngle;
			this->compliance = compliance;
		}

		inline InverseLegPose operator+(const InverseLegPose& v) const {return InverseLegPose(footPosition+v.footPosition, footAngle+v.footAngle, compliance);}
		inline InverseLegPose operator-() const {return InverseLegPose(-footPosition, -footAngle, compliance);}
		inline InverseLegPose operator-(const InverseLegPose& v) const {return InverseLegPose(footPosition-v.footPosition, footAngle-v.footAngle, compliance);}
		inline InverseLegPose operator+=(const InverseLegPose& v) {footPosition+=v.footPosition; footAngle+=v.footAngle; return *this;}
		inline InverseLegPose operator-=(const InverseLegPose& v) {footPosition-=v.footPosition; footAngle-=v.footAngle; return *this;}
		inline InverseLegPose& operator*=(const double scalar){footPosition*=scalar; footAngle*=scalar; return *this;}
		inline InverseLegPose& operator/=(const double scalar){footPosition/=scalar; footAngle/=scalar; return *this;}
		inline bool operator==(const InverseLegPose& v) const {return (footPosition==v.footPosition) && (footAngle==v.footAngle);}
		inline bool operator!=(const InverseLegPose& v) const {return (footPosition!=v.footPosition) || (footAngle!=v.footAngle);}
	};


	// The head pose describes joint angles for the neck.
	struct HeadPose
	{
		JointConfig neck;

		HeadPose()
		{

		}

		HeadPose(const JointConfig& neck)
		{
			this->neck = neck;
		}

		inline HeadPose operator+(const HeadPose& v) const {return HeadPose(neck+v.neck);}
		inline HeadPose operator-() const {return HeadPose(-neck);}
		inline HeadPose operator-(const HeadPose& v) const {return HeadPose(neck-v.neck);}
		inline HeadPose operator+=(const HeadPose& v) {neck+=v.neck; return *this;}
		inline HeadPose operator-=(const HeadPose& v) {neck-=v.neck; return *this;}
		inline HeadPose& operator*=(const double scalar) {neck*=scalar; return *this;}
		inline HeadPose& operator/=(const double scalar) {neck/=scalar; return *this;}
		inline bool operator==(const HeadPose& v) const {return (neck==v.neck);}
		inline bool operator!=(const HeadPose& v) const {return (neck!=v.neck);}

	};

	// The trunk pose describes joint angles for the trunk.
	struct TrunkPose
	{
		JointConfig spine;

		TrunkPose()
		{
		}

		TrunkPose(const JointConfig& spine)
		{
			this->spine = spine;
		}

		inline TrunkPose operator+(const TrunkPose& v) const {return TrunkPose(spine+v.spine);}
		inline TrunkPose operator-() const {return TrunkPose(-spine);}
		inline TrunkPose operator-(const TrunkPose& v) const {return TrunkPose(spine-v.spine);}
		inline TrunkPose operator+=(const TrunkPose& v) {spine+=v.spine; return *this;}
		inline TrunkPose operator-=(const TrunkPose& v) {spine-=v.spine; return *this;}
		inline TrunkPose& operator*=(const double scalar) {spine*=scalar; return *this;}
		inline TrunkPose& operator/=(const double scalar) {spine/=scalar; return *this;}
		inline bool operator==(const TrunkPose& v) const {return (spine==v.spine);}
		inline bool operator!=(const TrunkPose& v) const {return (spine!=v.spine);}
	};

	// The pose describes full body joint angles and compliances.
	struct Pose
	{
		HeadPose headPose;
		TrunkPose trunkPose;
		ArmPose leftArmPose;
		ArmPose rightArmPose;
		LegPose leftLegPose;
		LegPose rightLegPose;

		Pose()
		{

		}

		Pose(const HeadPose& headPose, const TrunkPose& trunkPose, const ArmPose& leftArmPose, const ArmPose& rightArmPose, const LegPose& leftLegPose, const LegPose& rightLegPose)
		{
			this->headPose = headPose;
			this->trunkPose = trunkPose;
			this->leftArmPose = leftArmPose;
			this->rightArmPose = rightArmPose;
			this->leftLegPose = leftLegPose;
			this->rightLegPose = rightLegPose;
		}

		inline Pose operator+(const Pose& v) const {return Pose(headPose+v.headPose, trunkPose+v.trunkPose, leftArmPose+v.leftArmPose, rightArmPose+v.rightArmPose, leftLegPose+v.leftLegPose, rightLegPose+v.rightLegPose);}
		inline Pose operator-() const {return Pose(-headPose, -trunkPose, -leftArmPose, -rightArmPose, -leftLegPose, -rightLegPose);}
		inline Pose operator-(const Pose& v) const {return Pose(headPose-v.headPose, trunkPose-v.trunkPose, leftArmPose-v.leftArmPose, rightArmPose-v.rightArmPose, leftLegPose-v.leftLegPose, rightLegPose-v.rightLegPose);}
		inline Pose operator+=(const Pose& v) {headPose+=v.headPose; trunkPose+=v.trunkPose; leftArmPose+=v.leftArmPose; rightArmPose+=v.rightArmPose; leftLegPose+=v.leftLegPose; rightLegPose+=v.rightLegPose; return *this;}
		inline Pose operator-=(const Pose& v) {headPose-=v.headPose; trunkPose-=v.trunkPose; leftArmPose-=v.leftArmPose; rightArmPose-=v.rightArmPose; leftLegPose-=v.leftLegPose; rightLegPose-=v.rightLegPose; return *this;}
		inline Pose& operator*=(const double scalar) {headPose*=scalar; trunkPose*=scalar; leftArmPose*=scalar; rightArmPose*=scalar; leftLegPose*=scalar; rightLegPose*=scalar; return *this;}
		inline Pose& operator/=(const double scalar) {headPose/=scalar; trunkPose/=scalar; leftArmPose/=scalar; rightArmPose/=scalar; leftLegPose/=scalar; rightLegPose/=scalar; return *this;}
		inline bool operator==(const Pose& v) const {return (headPose==v.headPose) && (trunkPose==v.trunkPose) && (leftArmPose==v.leftArmPose) && (rightArmPose==v.rightArmPose) && (leftLegPose==v.leftLegPose) && (rightLegPose==v.rightLegPose);}
		inline bool operator!=(const Pose& v) const {return (headPose!=v.headPose) || (trunkPose!=v.trunkPose) || (leftArmPose!=v.leftArmPose) || (rightArmPose!=v.rightArmPose) || (leftLegPose!=v.leftLegPose) || (rightLegPose!=v.rightLegPose);}
	};

	// The abstract pose is a full body pose on the abstract end effector level.
	struct AbstractPose
	{
		HeadPose headPose;
		TrunkPose trunkPose;
		AbstractArmPose leftArmPose;
		AbstractArmPose rightArmPose;
		AbstractLegPose leftLegPose;
		AbstractLegPose rightLegPose;

		AbstractPose()
		{

		}

		AbstractPose(const HeadPose& headPose, const TrunkPose& trunkPose, const AbstractArmPose& leftArmPose, const AbstractArmPose& rightArmPose, const AbstractLegPose& leftLegPose, const AbstractLegPose& rightLegPose)
		{
			this->headPose = headPose;
			this->trunkPose = trunkPose;
			this->leftArmPose = leftArmPose;
			this->rightArmPose = rightArmPose;
			this->leftLegPose = leftLegPose;
			this->rightLegPose = rightLegPose;
		}

		inline AbstractPose operator+(const AbstractPose& v) const {return AbstractPose(headPose+v.headPose, trunkPose+v.trunkPose, leftArmPose+v.leftArmPose, rightArmPose+v.rightArmPose, leftLegPose+v.leftLegPose, rightLegPose+v.rightLegPose);}
		inline AbstractPose operator-() const {return AbstractPose(-headPose, -trunkPose, -leftArmPose, -rightArmPose, -leftLegPose, -rightLegPose);}
		inline AbstractPose operator-(const AbstractPose& v) const {return AbstractPose(headPose-v.headPose, trunkPose-v.trunkPose, leftArmPose-v.leftArmPose, rightArmPose-v.rightArmPose, leftLegPose-v.leftLegPose, rightLegPose-v.rightLegPose);}
		inline AbstractPose operator+=(const AbstractPose& v) {headPose+=v.headPose; trunkPose+=v.trunkPose; leftArmPose+=v.leftArmPose; rightArmPose+=v.rightArmPose; leftLegPose+=v.leftLegPose; rightLegPose+=v.rightLegPose; return *this;}
		inline AbstractPose operator-=(const AbstractPose& v) {headPose-=v.headPose; trunkPose-=v.trunkPose; leftArmPose-=v.leftArmPose; rightArmPose-=v.rightArmPose; leftLegPose-=v.leftLegPose; rightLegPose-=v.rightLegPose; return *this;}
		inline AbstractPose& operator*=(const double scalar) {headPose*=scalar; trunkPose*=scalar; leftArmPose*=scalar; rightArmPose*=scalar; leftLegPose*=scalar; rightLegPose*=scalar; return *this;}
		inline AbstractPose& operator/=(const double scalar) {headPose/=scalar; trunkPose/=scalar; leftArmPose/=scalar; rightArmPose/=scalar; leftLegPose/=scalar; rightLegPose/=scalar; return *this;}
		inline bool operator==(const AbstractPose& v) const {return (headPose==v.headPose) && (trunkPose==v.trunkPose) && (leftArmPose==v.leftArmPose) && (rightArmPose==v.rightArmPose)&& (leftLegPose==v.leftLegPose) && (rightLegPose==v.rightLegPose);}
		inline bool operator!=(const AbstractPose& v) const {return (headPose!=v.headPose) || (trunkPose!=v.trunkPose) || (leftArmPose!=v.leftArmPose) || (rightArmPose!=v.rightArmPose)|| (leftLegPose!=v.leftLegPose) || (rightLegPose!=v.rightLegPose);}
	};

	// The inverse pose is a full body pose defined by end effector poses in Cartesian space.
	struct InversePose
	{
		HeadPose headPose;
		TrunkPose trunkPose;
		InverseArmPose leftArmPose;
		InverseArmPose rightArmPose;
		InverseLegPose leftLegPose;
		InverseLegPose rightLegPose;

		InversePose()
		{

		}

		InversePose(const HeadPose& headPose, const TrunkPose& trunkPose, const InverseArmPose& leftArmPose, const InverseArmPose& rightArmPose, const InverseLegPose& leftLegPose, const InverseLegPose& rightLegPose)
		{
			this->headPose = headPose;
			this->trunkPose = trunkPose;
			this->leftArmPose = leftArmPose;
			this->rightArmPose = rightArmPose;
			this->leftLegPose = leftLegPose;
			this->rightLegPose = rightLegPose;
		}

		inline InversePose operator+(const InversePose& v) const {return InversePose(headPose+v.headPose, trunkPose+v.trunkPose, leftArmPose+v.leftArmPose, rightArmPose+v.rightArmPose, leftLegPose+v.leftLegPose, rightLegPose+v.rightLegPose);}
		inline InversePose operator-() const {return InversePose(-headPose, -trunkPose, -leftArmPose, -rightArmPose, -leftLegPose, -rightLegPose);}
		inline InversePose operator-(const InversePose& v) const {return InversePose(headPose-v.headPose, trunkPose-v.trunkPose, leftArmPose-v.leftArmPose, rightArmPose-v.rightArmPose, leftLegPose-v.leftLegPose, rightLegPose-v.rightLegPose);}
		inline InversePose operator+=(const InversePose& v) {headPose+=v.headPose; trunkPose+=v.trunkPose; leftArmPose+=v.leftArmPose; rightArmPose+=v.rightArmPose; leftLegPose+=v.leftLegPose; rightLegPose+=v.rightLegPose; return *this;}
		inline InversePose operator-=(const InversePose& v) {headPose-=v.headPose; trunkPose-=v.trunkPose; leftArmPose-=v.leftArmPose; rightArmPose-=v.rightArmPose; leftLegPose-=v.leftLegPose; rightLegPose-=v.rightLegPose; return *this;}
		inline InversePose& operator*=(const double scalar) {headPose*=scalar; trunkPose*=scalar; leftArmPose*=scalar; rightArmPose*=scalar; leftLegPose*=scalar; rightLegPose*=scalar; return *this;}
		inline InversePose& operator/=(const double scalar) {headPose/=scalar; trunkPose/=scalar; leftArmPose/=scalar; rightArmPose/=scalar; leftLegPose/=scalar; rightLegPose/=scalar; return *this;}
		inline bool operator==(const InversePose& v) const {return (headPose==v.headPose) && (trunkPose==v.trunkPose) && (leftArmPose==v.leftArmPose) && (rightArmPose==v.rightArmPose)&& (leftLegPose==v.leftLegPose) && (rightLegPose==v.rightLegPose);}
		inline bool operator!=(const InversePose& v) const {return (headPose!=v.headPose) || (trunkPose!=v.trunkPose) || (leftArmPose!=v.leftArmPose) || (rightArmPose!=v.rightArmPose)|| (leftLegPose!=v.leftLegPose) || (rightLegPose!=v.rightLegPose);}
	};

	// And finally, the complete Action object contains pose commands on three levels.
	// Joint level, abstract pose level and inverse pose level.
	struct Action
	{
		Pose pose;
		AbstractPose abstractPose;
		InversePose inversePose;
	};

}
#endif /* ACTION_H_ */
