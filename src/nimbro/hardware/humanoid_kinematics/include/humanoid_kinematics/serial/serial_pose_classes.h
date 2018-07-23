// Humanoid kinematics - Serial pose classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef SERIAL_POSE_CLASSES_H
#define SERIAL_POSE_CLASSES_H

// Includes
#include <humanoid_kinematics/kinematics_common.h>
#include <humanoid_kinematics/pose_classes.h>
#include <Eigen/QR>

// Humanoid kinematics namespace
namespace humanoid_kinematics
{
	/**
	* @namespace serial_pose_classes
	*
	* @brief Serial kinematics pose classes namespace.
	**/
	namespace serial_pose_classes
	{
		// Import the stream operators from rot_conv
		using rot_conv::operator<<;

		// Pose velocity vector and Jacobian types
		typedef Eigen::Matrix<double, 6, 1> LegVelVec;
		typedef Eigen::Matrix<double, 6, 6> LegJacobian;
		typedef Eigen::ColPivHouseholderQR<LegJacobian> LegJacobianSolver;

		//
		// Joint pose classes
		//

		//! Serial joint leg pose class
		class JointLegPose : public pose_classes::JointLegPoseBase
		{
		public:
			// Constructor/destructor
			JointLegPose(LimbIndex limbIndex) : JointLegPoseBase(limbIndex) {}
			JointLegPose(LimbIndex limbIndex, double hipYaw, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll) : JointLegPoseBase(limbIndex), hipYaw(hipYaw), hipRoll(hipRoll), hipPitch(hipPitch), kneePitch(kneePitch), anklePitch(anklePitch), ankleRoll(ankleRoll) {}
			virtual ~JointLegPose() = default;

			// Data fields
			double hipYaw;
			double hipRoll;
			double hipPitch;
			double kneePitch;
			double anklePitch;
			double ankleRoll;

			// Field enumeration
			enum Field : std::size_t
			{
				HIPYAW,
				HIPROLL,
				HIPPITCH,
				KNEEPITCH,
				ANKLEPITCH,
				ANKLEROLL,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline JointLegPose ZeroL() { return JointLegPose(LEFT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }
			static inline JointLegPose ZeroR() { return JointLegPose(RIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

			// Joint information functions
			virtual void appendJointInfo(std::vector<JointInfo>& info) const override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];

			// Joint names
			static const std::string jointNameLeft[NUM_FIELDS];
			static const std::string jointNameRight[NUM_FIELDS];
			static const std::string* const jointName[NUM_LR];
		};

		//! Serial joint arm pose class
		class JointArmPose : public pose_classes::JointArmPoseBase
		{
		public:
			// Constructor/destructor
			JointArmPose(LimbIndex limbIndex) : JointArmPoseBase(limbIndex) {}
			JointArmPose(LimbIndex limbIndex, double shoulderPitch, double shoulderRoll, double elbowPitch) : JointArmPoseBase(limbIndex), shoulderPitch(shoulderPitch), shoulderRoll(shoulderRoll), elbowPitch(elbowPitch) {}
			virtual ~JointArmPose() = default;

			// Data fields
			double shoulderPitch;
			double shoulderRoll;
			double elbowPitch;

			// Field enumeration
			enum Field : std::size_t
			{
				SHOULDERPITCH,
				SHOULDERROLL,
				ELBOWPITCH,
				NUM_FIELDS
			};

			// Constants
			static const double ShoulderPitchMax;

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline JointArmPose ZeroL() { return JointArmPose(LEFT, 0.0, 0.0, 0.0); }
			static inline JointArmPose ZeroR() { return JointArmPose(RIGHT, 0.0, 0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

			// Joint information functions
			virtual void appendJointInfo(std::vector<JointInfo>& info) const override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];

			// Joint names
			static const std::string jointNameLeft[NUM_FIELDS];
			static const std::string jointNameRight[NUM_FIELDS];
			static const std::string* const jointName[NUM_LR];
		};

		//! Serial joint head pose class
		class JointHeadPose : public pose_classes::JointHeadPoseBase
		{
		public:
			// Constructor/destructor
			JointHeadPose() : JointHeadPoseBase() {}
			JointHeadPose(double neckYaw, double neckPitch) : JointHeadPoseBase(), neckYaw(neckYaw), neckPitch(neckPitch) {}
			virtual ~JointHeadPose() = default;

			// Data fields
			double neckYaw;
			double neckPitch;

			// Field enumeration
			enum Field : std::size_t
			{
				NECKYAW,
				NECKPITCH,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline JointHeadPose Zero() { return JointHeadPose(0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

			// Joint information functions
			virtual void appendJointInfo(std::vector<JointInfo>& info) const override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];

			// Joint names
			static const std::string jointName[NUM_FIELDS];
		};

		//! Serial joint pose class
		class JointPose : public pose_classes::JointPoseBase
		{
		public:
			// Constructor/destructor
			JointPose() : JointPoseBase(), legL(LEFT), legR(RIGHT), armL(LEFT), armR(RIGHT), head() {}
			JointPose(const JointLegPose& legL, const JointLegPose& legR, const JointArmPose& armL, const JointArmPose& armR, const JointHeadPose& head = JointHeadPose::Zero()) : JointPose() { this->legL = legL; this->legR = legR; this->armL = armL; this->armR = armR; this->head = head; }
			virtual ~JointPose() = default;

			// Limbs
			JointLegPose legL;
			JointLegPose legR;
			JointArmPose armL;
			JointArmPose armR;
			JointHeadPose head;

			// Limb get functions
			JointLegPose& leg(LimbIndex limbIndex) { return (limbIndex == RIGHT ? legR : legL); }
			const JointLegPose& leg(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? legR : legL); }
			JointArmPose& arm(LimbIndex limbIndex) { return (limbIndex == RIGHT ? armR : armL); }
			const JointArmPose& arm(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? armR : armL); }

			// Field enumeration
			enum Field : std::size_t
			{
				NUM_FIELDS = 2*JointLegPose::NUM_FIELDS + 2*JointArmPose::NUM_FIELDS + JointHeadPose::NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual void appendFieldNames(std::vector<std::string>& names) const override { appendFieldNamesHelper(names, legL, legR, armL, armR, head); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { appendFieldNamesShortHelper(names, legL, legR, armL, armR, head); }

			// Zero functions
			virtual void setZero() override { legL.setZero(); legR.setZero(); armL.setZero(); armR.setZero(); head.setZero(); }
			static inline JointPose Zero() { return JointPose(JointLegPose::ZeroL(), JointLegPose::ZeroR(), JointArmPose::ZeroL(), JointArmPose::ZeroR(), JointHeadPose::Zero()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override { toArrayHelper(array, legL, legR, armL, armR, head); }
			virtual void fromArray(const double* array) override { fromArrayHelper(array, legL, legR, armL, armR, head); }

			// Joint information functions
			virtual void appendJointInfo(std::vector<JointInfo>& info) const override { appendJointInfoHelper(info, legL, legR, armL, armR, head); }
		};

		// Joint pose class stream operators
		inline std::ostream& operator<<(std::ostream& os, const JointLegPose& JLP) { return os << "JLP{" << JLP.limbIndex << ", " << JLP.hipYaw << ", " << JLP.hipRoll << ", " << JLP.hipPitch << ", " << JLP.kneePitch << ", " << JLP.anklePitch << ", " << JLP.ankleRoll << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const JointArmPose& JAP) { return os << "JAP{" << JAP.limbIndex << ", " << JAP.shoulderPitch << ", " << JAP.shoulderRoll << ", " << JAP.elbowPitch << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const JointHeadPose& JHP) { return os << "JHP{" << JHP.limbIndex << ", " << JHP.neckYaw << ", " << JHP.neckPitch << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const JointPose& JP) { return os << "JP[" << JP.legL << ", " << JP.legR << ", " << JP.armL << ", " << JP.armR << ", " << JP.head << "]"; }

		//
		// Abstract pose classes
		//

		//! Serial abstract leg pose class
		class AbsLegPose : public pose_classes::AbsLegPoseBase
		{
		public:
			// Constructor/destructor
			AbsLegPose(LimbIndex limbIndex) : AbsLegPoseBase(limbIndex) {}
			AbsLegPose(LimbIndex limbIndex, double angleX, double angleY, double angleZ, double footAngleX, double footAngleY, double retraction) : AbsLegPoseBase(limbIndex), angleX(angleX), angleY(angleY), angleZ(angleZ), footAngleX(footAngleX), footAngleY(footAngleY), retraction(retraction) {}
			virtual ~AbsLegPose() = default;

			// Data fields
			double angleX;
			double angleY;
			double angleZ;
			double footAngleX;
			double footAngleY;
			double retraction;

			// Field enumeration
			enum Field : std::size_t
			{
				ANGLEX,
				ANGLEY,
				ANGLEZ,
				FOOTANGLEX,
				FOOTANGLEY,
				RETRACTION,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline AbsLegPose ZeroL() { return AbsLegPose(LEFT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }
			static inline AbsLegPose ZeroR() { return AbsLegPose(RIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial abstract arm pose class
		class AbsArmPose : public pose_classes::AbsArmPoseBase
		{
		public:
			// Constructor/destructor
			AbsArmPose(LimbIndex limbIndex) : AbsArmPoseBase(limbIndex) {}
			AbsArmPose(LimbIndex limbIndex, double angleX, double angleY, double retraction) : AbsArmPoseBase(limbIndex), angleX(angleX), angleY(angleY), retraction(retraction) {}
			virtual ~AbsArmPose() = default;

			// Data fields
			double angleX;
			double angleY;
			double retraction;

			// Field enumeration
			enum Field : std::size_t
			{
				ANGLEX,
				ANGLEY,
				RETRACTION,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline AbsArmPose ZeroL() { return AbsArmPose(LEFT, 0.0, 0.0, 0.0); }
			static inline AbsArmPose ZeroR() { return AbsArmPose(RIGHT, 0.0, 0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial abstract head pose class
		class AbsHeadPose : public pose_classes::AbsHeadPoseBase
		{
		public:
			// Constructor/destructor
			AbsHeadPose() : AbsHeadPoseBase() {}
			AbsHeadPose(double angleY, double angleZ) : AbsHeadPoseBase(), angleY(angleY), angleZ(angleZ) {}
			virtual ~AbsHeadPose() = default;

			// Data fields
			double angleY;
			double angleZ;

			// Field enumeration
			enum Field : std::size_t
			{
				ANGLEY,
				ANGLEZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline AbsHeadPose Zero() { return AbsHeadPose(0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial abstract pose class
		class AbsPose : public pose_classes::AbsPoseBase
		{
		public:
			// Constructor/destructor
			AbsPose() : AbsPoseBase(), legL(LEFT), legR(RIGHT), armL(LEFT), armR(RIGHT), head() {}
			AbsPose(const AbsLegPose& legL, const AbsLegPose& legR, const AbsArmPose& armL, const AbsArmPose& armR, const AbsHeadPose& head = AbsHeadPose::Zero()) : AbsPose() { this->legL = legL; this->legR = legR; this->armL = armL; this->armR = armR; this->head = head; }
			virtual ~AbsPose() = default;

			// Limbs
			AbsLegPose legL;
			AbsLegPose legR;
			AbsArmPose armL;
			AbsArmPose armR;
			AbsHeadPose head;

			// Limb get functions
			AbsLegPose& leg(LimbIndex limbIndex) { return (limbIndex == RIGHT ? legR : legL); }
			const AbsLegPose& leg(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? legR : legL); }
			AbsArmPose& arm(LimbIndex limbIndex) { return (limbIndex == RIGHT ? armR : armL); }
			const AbsArmPose& arm(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? armR : armL); }

			// Field enumeration
			enum Field : std::size_t
			{
				NUM_FIELDS = 2*AbsLegPose::NUM_FIELDS + 2*AbsArmPose::NUM_FIELDS + AbsHeadPose::NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual void appendFieldNames(std::vector<std::string>& names) const override { appendFieldNamesHelper(names, legL, legR, armL, armR, head); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { appendFieldNamesShortHelper(names, legL, legR, armL, armR, head); }

			// Zero functions
			virtual void setZero() override { legL.setZero(); legR.setZero(); armL.setZero(); armR.setZero(); head.setZero(); }
			static inline AbsPose Zero() { return AbsPose(AbsLegPose::ZeroL(), AbsLegPose::ZeroR(), AbsArmPose::ZeroL(), AbsArmPose::ZeroR(), AbsHeadPose::Zero()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override { toArrayHelper(array, legL, legR, armL, armR, head); }
			virtual void fromArray(const double* array) override { fromArrayHelper(array, legL, legR, armL, armR, head); }
		};

		// Abstract pose class stream operators
		inline std::ostream& operator<<(std::ostream& os, const AbsLegPose& ALP) { return os << "ALP{" << ALP.limbIndex << ", " << ALP.angleX << ", " << ALP.angleY << ", " << ALP.angleZ << ", " << ALP.footAngleX << ", " << ALP.footAngleY << ", " << ALP.retraction << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const AbsArmPose& AAP) { return os << "AAP{" << AAP.limbIndex << ", " << AAP.angleX << ", " << AAP.angleY << ", " << AAP.retraction << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const AbsHeadPose& AHP) { return os << "AHP{" << AHP.limbIndex << ", " << AHP.angleY << ", " << AHP.angleZ << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const AbsPose& AP) { return os << "AP[" << AP.legL << ", " << AP.legR << ", " << AP.armL << ", " << AP.armR << ", " << AP.head << "]"; }

		//
		// Inverse pose classes
		//

		//! Serial inverse leg pose class
		class InvLegPose : public pose_classes::InvLegPoseBase
		{
		public:
			// Constructor/destructor
			InvLegPose(LimbIndex limbIndex) : InvLegPoseBase(limbIndex) {}
			InvLegPose(LimbIndex limbIndex, const rot_conv::Vec3& anklePos, const rot_conv::Quat& footRot) : InvLegPoseBase(limbIndex), anklePos(anklePos), footRot(footRot) {}
			virtual ~InvLegPose() = default;

			// Data fields
			rot_conv::Vec3 anklePos;
			rot_conv::Quat footRot;

			// Field enumeration
			enum Field : std::size_t
			{
				ANKLEPOSX,
				ANKLEPOSY,
				ANKLEPOSZ,
				FOOTROTW,
				FOOTROTX,
				FOOTROTY,
				FOOTROTZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline InvLegPose ZeroL() { return InvLegPose(LEFT, rot_conv::Vec3::Zero(), rot_conv::Quat::Identity()); }
			static inline InvLegPose ZeroR() { return InvLegPose(RIGHT, rot_conv::Vec3::Zero(), rot_conv::Quat::Identity()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

			// Get functions
			virtual rot_conv::Vec3 invPos() const override { return anklePos; }
			virtual rot_conv::Quat invRot() const override { return footRot; }

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) override { anklePos += shift; }
			virtual void rotateTip(const rot_conv::Quat& q) override { footRot = q * footRot; }
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) override { footRot = q * footRot; anklePos = origin + rot_conv::QuatRotVec(q, anklePos - origin); }

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial inverse arm pose class
		class InvArmPose : public pose_classes::InvArmPoseBase
		{
		public:
			// Constructor/destructor
			InvArmPose(LimbIndex limbIndex) : InvArmPoseBase(limbIndex) {}
			InvArmPose(LimbIndex limbIndex, const rot_conv::Vec3& handPos) : InvArmPoseBase(limbIndex), handPos(handPos) {}
			virtual ~InvArmPose() = default;

			// Data fields
			rot_conv::Vec3 handPos;

			// Field enumeration
			enum Field : std::size_t
			{
				HANDPOSX,
				HANDPOSY,
				HANDPOSZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline InvArmPose ZeroL() { return InvArmPose(LEFT, rot_conv::Vec3::Zero()); }
			static inline InvArmPose ZeroR() { return InvArmPose(RIGHT, rot_conv::Vec3::Zero()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

			// Get functions
			virtual rot_conv::Vec3 invPos() const override { return handPos; }
			virtual rot_conv::Quat invRot() const override { return rot_conv::Quat::Identity(); }

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) override { handPos += shift; }
			virtual void rotateTip(const rot_conv::Quat& q) override {}
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) override { handPos = origin + rot_conv::QuatRotVec(q, handPos - origin); }

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial inverse head pose class
		class InvHeadPose : public pose_classes::InvHeadPoseBase
		{
		public:
			// Constructor/destructor
			InvHeadPose() : InvHeadPoseBase() {}
			InvHeadPose(const rot_conv::Vec3& dirnVec) : InvHeadPoseBase(), dirnVec(dirnVec) { RotFromVec(this->dirnVec, this->headRot); }
			InvHeadPose(const rot_conv::Quat& headRot) : InvHeadPoseBase() { VecFromRot(headRot, this->dirnVec); RotFromVec(this->dirnVec, this->headRot); }
			virtual ~InvHeadPose() = default;

			// Data fields
			rot_conv::Vec3 dirnVec;
			rot_conv::Quat headRot;

			// Field enumeration
			enum Field : std::size_t
			{
				DIRNVECX,
				DIRNVECY,
				DIRNVECZ,
				HEADROTW,
				HEADROTX,
				HEADROTY,
				HEADROTZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline InvHeadPose Zero() { return InvHeadPose(rot_conv::VecUnitX()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

			// Get functions
			virtual rot_conv::Vec3 invPos() const override { return rot_conv::Vec3::Zero(); }
			virtual rot_conv::Quat invRot() const override { return headRot; }

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) override {}
			virtual void rotateTip(const rot_conv::Quat& q) override { dirnVec = rot_conv::QuatRotVec(q, dirnVec); RotFromVec(dirnVec, headRot); }
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) override { dirnVec = rot_conv::QuatRotVec(q, dirnVec); RotFromVec(dirnVec, headRot); }

			// Helper functions
			static void RotFromVec(const rot_conv::Vec3& dirnVec, rot_conv::Quat& headRot);
			static rot_conv::Quat RotFromVec(const rot_conv::Vec3& dirnVec) { rot_conv::Quat headRot; RotFromVec(dirnVec, headRot); return headRot; }
			static void VecFromRot(const rot_conv::Quat& headRot, rot_conv::Vec3& dirnVec);
			static rot_conv::Vec3 VecFromRot(const rot_conv::Quat& headRot) { rot_conv::Vec3 dirnVec; VecFromRot(headRot, dirnVec); return dirnVec; }

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial inverse pose class
		class InvPose : public pose_classes::InvPoseBase
		{
		public:
			// Constructor/destructor
			InvPose() : InvPoseBase(), legL(LEFT), legR(RIGHT), armL(LEFT), armR(RIGHT), head() {}
			InvPose(const InvLegPose& legL, const InvLegPose& legR, const InvArmPose& armL, const InvArmPose& armR, const InvHeadPose& head = InvHeadPose::Zero()) : InvPose() { this->legL = legL; this->legR = legR; this->armL = armL; this->armR = armR; this->head = head; }
			virtual ~InvPose() = default;

			// Limbs
			InvLegPose legL;
			InvLegPose legR;
			InvArmPose armL;
			InvArmPose armR;
			InvHeadPose head;

			// Limb get functions
			InvLegPose& leg(LimbIndex limbIndex) { return (limbIndex == RIGHT ? legR : legL); }
			const InvLegPose& leg(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? legR : legL); }
			InvArmPose& arm(LimbIndex limbIndex) { return (limbIndex == RIGHT ? armR : armL); }
			const InvArmPose& arm(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? armR : armL); }

			// Field enumeration
			enum Field : std::size_t
			{
				NUM_FIELDS = 2*InvLegPose::NUM_FIELDS + 2*InvArmPose::NUM_FIELDS + InvHeadPose::NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual void appendFieldNames(std::vector<std::string>& names) const override { appendFieldNamesHelper(names, legL, legR, armL, armR, head); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { appendFieldNamesShortHelper(names, legL, legR, armL, armR, head); }

			// Zero functions
			virtual void setZero() override { legL.setZero(); legR.setZero(); armL.setZero(); armR.setZero(); head.setZero(); }
			static inline InvPose Zero() { return InvPose(InvLegPose::ZeroL(), InvLegPose::ZeroR(), InvArmPose::ZeroL(), InvArmPose::ZeroR(), InvHeadPose::Zero()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override { toArrayHelper(array, legL, legR, armL, armR, head); }
			virtual void fromArray(const double* array) override { fromArrayHelper(array, legL, legR, armL, armR, head); }
		};

		// Inverse pose class stream operators
		inline std::ostream& operator<<(std::ostream& os, const InvLegPose& ILP) { return os << "ILP{" << ILP.limbIndex << ", " << ILP.anklePos << ", " << ILP.footRot << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const InvArmPose& IAP) { return os << "IAP{" << IAP.limbIndex << ", " << IAP.handPos << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const InvHeadPose& IHP) { return os << "IHP{" << IHP.limbIndex << ", " << IHP.dirnVec << ", " << IHP.headRot << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const InvPose& IP) { return os << "IP[" << IP.legL << ", " << IP.legR << ", " << IP.armL << ", " << IP.armR << ", " << IP.head << "]"; }

		//
		// Tip pose classes
		//

		//! Serial leg tip pose class
		class LegTipPose : public pose_classes::LegTipPoseBase
		{
		public:
			// Constructor/destructor
			LegTipPose(LimbIndex limbIndex) : LegTipPoseBase(limbIndex) {}
			LegTipPose(LimbIndex limbIndex, const rot_conv::Vec3& pos, const rot_conv::Quat& rot) : LegTipPoseBase(limbIndex, pos, rot) {}
			virtual ~LegTipPose() = default;

			// Field enumeration
			enum Field : std::size_t
			{
				POSX,
				POSY,
				POSZ,
				ROTW,
				ROTX,
				ROTY,
				ROTZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline LegTipPose ZeroL() { return LegTipPose(LEFT, rot_conv::Vec3::Zero(), rot_conv::Quat::Identity()); } // Note: This does NOT necessarily correspond to the zero pose of the robot due to configured tip offsets!
			static inline LegTipPose ZeroR() { return LegTipPose(RIGHT, rot_conv::Vec3::Zero(), rot_conv::Quat::Identity()); } // Note: This does NOT necessarily correspond to the zero pose of the robot due to configured tip offsets!

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial arm tip pose class
		class ArmTipPose : public pose_classes::ArmTipPoseBase
		{
		public:
			// Constructor/destructor
			ArmTipPose(LimbIndex limbIndex) : ArmTipPoseBase(limbIndex) {}
			ArmTipPose(LimbIndex limbIndex, const rot_conv::Vec3& pos, const rot_conv::Quat& rot) : ArmTipPoseBase(limbIndex, pos, rot) {}
			virtual ~ArmTipPose() = default;

			// Field enumeration
			enum Field : std::size_t
			{
				POSX,
				POSY,
				POSZ,
				ROTW,
				ROTX,
				ROTY,
				ROTZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline ArmTipPose ZeroL() { return ArmTipPose(LEFT, rot_conv::Vec3::Zero(), rot_conv::Quat::Identity()); } // Note: This does NOT necessarily correspond to the zero pose of the robot due to configured tip offsets!
			static inline ArmTipPose ZeroR() { return ArmTipPose(RIGHT, rot_conv::Vec3::Zero(), rot_conv::Quat::Identity()); } // Note: This does NOT necessarily correspond to the zero pose of the robot due to configured tip offsets!

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial head tip pose class
		class HeadTipPose : public pose_classes::HeadTipPoseBase
		{
		public:
			// Constructor/destructor
			HeadTipPose() : HeadTipPoseBase() {}
			HeadTipPose(const rot_conv::Vec3& pos, const rot_conv::Quat& rot) : HeadTipPoseBase(pos, rot) {}
			virtual ~HeadTipPose() = default;

			// Field enumeration
			enum Field : std::size_t
			{
				POSX,
				POSY,
				POSZ,
				ROTW,
				ROTX,
				ROTY,
				ROTZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline HeadTipPose Zero() { return HeadTipPose(rot_conv::Vec3::Zero(), rot_conv::Quat::Identity()); } // Note: This does NOT necessarily correspond to the zero pose of the robot due to configured tip offsets!

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		//! Serial tip pose class
		class TipPose : public pose_classes::TipPoseBase
		{
		public:
			// Constructor/destructor
			TipPose() : TipPoseBase(), legL(LEFT), legR(RIGHT), armL(LEFT), armR(RIGHT), head() {}
			TipPose(const LegTipPose& legL, const LegTipPose& legR, const ArmTipPose& armL, const ArmTipPose& armR, const HeadTipPose& head = HeadTipPose::Zero()) : TipPose() { this->legL = legL; this->legR = legR; this->armL = armL; this->armR = armR; this->head = head; }
			virtual ~TipPose() = default;

			// Limbs
			LegTipPose legL;
			LegTipPose legR;
			ArmTipPose armL;
			ArmTipPose armR;
			HeadTipPose head;

			// Limb get functions
			LegTipPose& leg(LimbIndex limbIndex) { return (limbIndex == RIGHT ? legR : legL); }
			const LegTipPose& leg(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? legR : legL); }
			ArmTipPose& arm(LimbIndex limbIndex) { return (limbIndex == RIGHT ? armR : armL); }
			const ArmTipPose& arm(LimbIndex limbIndex) const { return (limbIndex == RIGHT ? armR : armL); }

			// Field enumeration
			enum Field : std::size_t
			{
				NUM_FIELDS = 2*LegTipPose::NUM_FIELDS + 2*ArmTipPose::NUM_FIELDS + HeadTipPose::NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual void appendFieldNames(std::vector<std::string>& names) const override { appendFieldNamesHelper(names, legL, legR, armL, armR, head); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { appendFieldNamesShortHelper(names, legL, legR, armL, armR, head); }

			// Zero functions
			virtual void setZero() override { legL.setZero(); legR.setZero(); armL.setZero(); armR.setZero(); head.setZero(); }
			static inline TipPose Zero() { return TipPose(LegTipPose::ZeroL(), LegTipPose::ZeroR(), ArmTipPose::ZeroL(), ArmTipPose::ZeroR(), HeadTipPose::Zero()); } // Note: This does NOT necessarily correspond to the zero pose of the robot due to configured tip offsets!

			// Pose field array serialisation
			virtual void toArray(double* array) const override { toArrayHelper(array, legL, legR, armL, armR, head); }
			virtual void fromArray(const double* array) override { fromArrayHelper(array, legL, legR, armL, armR, head); }
		};

		// Tip pose class stream operators
		inline std::ostream& operator<<(std::ostream& os, const LegTipPose& LTP) { return os << "LTP{" << LTP.limbIndex << ", " << LTP.pos << ", " << LTP.rot << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const ArmTipPose& ATP) { return os << "ATP{" << ATP.limbIndex << ", " << ATP.pos << ", " << ATP.rot << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const HeadTipPose& HTP) { return os << "HTP{" << HTP.limbIndex << ", " << HTP.pos << ", " << HTP.rot << "}"; }
		inline std::ostream& operator<<(std::ostream& os, const TipPose& TP) { return os << "TP[" << TP.legL << ", " << TP.legR << ", " << TP.armL << ", " << TP.armR << ", " << TP.head << "]"; }

		//
		// Joint pose velocity classes
		//

		//! Serial joint leg pose velocity class
		class JointLegPoseVel : public pose_classes::JointLegPoseVelBase
		{
		public:
			// Constructor/destructor
			JointLegPoseVel(LimbIndex limbIndex) : JointLegPoseVelBase(limbIndex) {}
			JointLegPoseVel(LimbIndex limbIndex, double hipYawVel, double hipRollVel, double hipPitchVel, double kneePitchVel, double anklePitchVel, double ankleRollVel) : JointLegPoseVelBase(limbIndex), hipYawVel(hipYawVel), hipRollVel(hipRollVel), hipPitchVel(hipPitchVel), kneePitchVel(kneePitchVel), anklePitchVel(anklePitchVel), ankleRollVel(ankleRollVel) {}
			virtual ~JointLegPoseVel() = default;

			// Data fields
			double hipYawVel;
			double hipRollVel;
			double hipPitchVel;
			double kneePitchVel;
			double anklePitchVel;
			double ankleRollVel;

			// Field enumeration
			enum Field : std::size_t
			{
				HIPYAWVEL,
				HIPROLLVEL,
				HIPPITCHVEL,
				KNEEPITCHVEL,
				ANKLEPITCHVEL,
				ANKLEROLLVEL,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline JointLegPoseVel ZeroL() { return JointLegPoseVel(LEFT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }
			static inline JointLegPoseVel ZeroR() { return JointLegPoseVel(RIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		// Joint pose velocity class stream operators
		inline std::ostream& operator<<(std::ostream& os, const JointLegPoseVel& JLPV) { return os << "JLPV{" << JLPV.limbIndex << ", " << JLPV.hipYawVel << ", " << JLPV.hipRollVel << ", " << JLPV.hipPitchVel << ", " << JLPV.kneePitchVel << ", " << JLPV.anklePitchVel << ", " << JLPV.ankleRollVel << "}"; }

		//
		// Abstract pose velocity classes
		//

		//! Serial abstract leg pose velocity class
		class AbsLegPoseVel : public pose_classes::AbsLegPoseVelBase
		{
		public:
			// Constructor/destructor
			AbsLegPoseVel(LimbIndex limbIndex) : AbsLegPoseVelBase(limbIndex) {}
			AbsLegPoseVel(LimbIndex limbIndex, double angleXVel, double angleYVel, double angleZVel, double footAngleXVel, double footAngleYVel, double retractionVel) : AbsLegPoseVelBase(limbIndex), angleXVel(angleXVel), angleYVel(angleYVel), angleZVel(angleZVel), footAngleXVel(footAngleXVel), footAngleYVel(footAngleYVel), retractionVel(retractionVel) {}
			virtual ~AbsLegPoseVel() = default;

			// Data fields
			double angleXVel;
			double angleYVel;
			double angleZVel;
			double footAngleXVel;
			double footAngleYVel;
			double retractionVel;

			// Field enumeration
			enum Field : std::size_t
			{
				ANGLEXVEL,
				ANGLEYVEL,
				ANGLEZVEL,
				FOOTANGLEXVEL,
				FOOTANGLEYVEL,
				RETRACTIONVEL,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline AbsLegPoseVel ZeroL() { return AbsLegPoseVel(LEFT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }
			static inline AbsLegPoseVel ZeroR() { return AbsLegPoseVel(RIGHT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		// Abstract pose velocity class stream operators
		inline std::ostream& operator<<(std::ostream& os, const AbsLegPoseVel& ALPV) { return os << "ALPV{" << ALPV.limbIndex << ", " << ALPV.angleXVel << ", " << ALPV.angleYVel << ", " << ALPV.angleZVel << ", " << ALPV.footAngleXVel << ", " << ALPV.footAngleYVel << ", " << ALPV.retractionVel << "}"; }

		//
		// Inverse pose velocity classes
		//

		//! Serial inverse leg pose velocity class
		class InvLegPoseVel : public pose_classes::InvLegPoseVelBase
		{
		public:
			// Constructor/destructor
			InvLegPoseVel(LimbIndex limbIndex) : InvLegPoseVelBase(limbIndex) {}
			InvLegPoseVel(LimbIndex limbIndex, const rot_conv::Vec3& ankleVel, const rot_conv::AngVel& footAngVel) : InvLegPoseVelBase(limbIndex), ankleVel(ankleVel), footAngVel(footAngVel) {}
			virtual ~InvLegPoseVel() = default;

			// Data fields
			rot_conv::Vec3 ankleVel;
			rot_conv::AngVel footAngVel;

			// Field enumeration
			enum Field : std::size_t
			{
				ANKLEVELX,
				ANKLEVELY,
				ANKLEVELZ,
				FOOTANGVELX,
				FOOTANGVELY,
				FOOTANGVELZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline InvLegPoseVel ZeroL() { return InvLegPoseVel(LEFT, rot_conv::Vec3::Zero(), rot_conv::AngVel::Zero()); }
			static inline InvLegPoseVel ZeroR() { return InvLegPoseVel(RIGHT, rot_conv::Vec3::Zero(), rot_conv::AngVel::Zero()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

			// Get functions
			virtual rot_conv::Vec3 invVel() const override { return ankleVel; }
			virtual rot_conv::AngVel invAngVel() const override { return footAngVel; }

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		// Inverse pose velocity class stream operators
		inline std::ostream& operator<<(std::ostream& os, const InvLegPoseVel& ILPV) { return os << "ILPV{" << ILPV.limbIndex << ", " << ILPV.ankleVel << ", " << ILPV.footAngVel << "}"; }

		//
		// Tip pose velocity classes
		//

		//! Serial leg tip pose velocity class
		class LegTipPoseVel : public pose_classes::LegTipPoseVelBase
		{
		public:
			// Constructor/destructor
			LegTipPoseVel(LimbIndex limbIndex) : LegTipPoseVelBase(limbIndex) {}
			LegTipPoseVel(LimbIndex limbIndex, const rot_conv::Vec3& vel, const rot_conv::AngVel& angVel) : LegTipPoseVelBase(limbIndex, vel, angVel) {}
			virtual ~LegTipPoseVel() = default;

			// Field enumeration
			enum Field : std::size_t
			{
				VELX,
				VELY,
				VELZ,
				ANGVELX,
				ANGVELY,
				ANGVELZ,
				NUM_FIELDS
			};

			// Number of fields in pose
			virtual std::size_t numFields() const override { return NUM_FIELDS; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const override { return (field < NUM_FIELDS ? fieldName[field] : UnknownField()); }
			virtual std::string getFieldNameShort(std::size_t field) const override { return (field < NUM_FIELDS ? fieldNameShort[field] : UnknownFieldShort()); }
			virtual void appendFieldNames(std::vector<std::string>& names) const override { names.insert(names.end(), fieldName, fieldName + NUM_FIELDS); }
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const override { names.insert(names.end(), fieldNameShort, fieldNameShort + NUM_FIELDS); }

			// Zero functions
			virtual void setZero() override;
			static inline LegTipPoseVel ZeroL() { return LegTipPoseVel(LEFT, rot_conv::Vec3::Zero(), rot_conv::AngVel::Zero()); }
			static inline LegTipPoseVel ZeroR() { return LegTipPoseVel(RIGHT, rot_conv::Vec3::Zero(), rot_conv::AngVel::Zero()); }

			// Pose field array serialisation
			virtual void toArray(double* array) const override;
			virtual void fromArray(const double* array) override;

		private:
			// Field names
			static const std::string fieldName[NUM_FIELDS];
			static const std::string fieldNameShort[NUM_FIELDS];
		};

		// Tip pose velocity class stream operators
		inline std::ostream& operator<<(std::ostream& os, const LegTipPoseVel& LTPV) { return os << "LTPV{" << LTPV.limbIndex << ", " << LTPV.vel << ", " << LTPV.angVel << "}"; }

		//
		// Pose velocity vector conversions
		//

		// Conversion: Leg pose velocity --> Leg velocity vector
		inline void VecFromJointVel(const JointLegPoseVel& JLPV, LegVelVec& JLPVelVec) { JLPVelVec << JLPV.hipYawVel, JLPV.hipRollVel, JLPV.hipPitchVel, JLPV.kneePitchVel, JLPV.anklePitchVel, JLPV.ankleRollVel; }
		inline void VecFromAbsVel(const AbsLegPoseVel& ALPV, LegVelVec& ALPVelVec) { ALPVelVec << ALPV.angleXVel, ALPV.angleYVel, ALPV.angleZVel, ALPV.footAngleXVel, ALPV.footAngleYVel, ALPV.retractionVel; }
		inline void VecFromInvVel(const InvLegPoseVel& ILPV, LegVelVec& ILPVelVec) { ILPVelVec << ILPV.ankleVel, ILPV.footAngVel; }
		inline void VecFromTipVel(const LegTipPoseVel& LTPV, LegVelVec& LTPVelVec) { LTPVelVec << LTPV.vel, LTPV.angVel; }
		inline LegVelVec VecFromJointVel(const JointLegPoseVel& JLPV) { LegVelVec JLPVelVec; VecFromJointVel(JLPV, JLPVelVec); return JLPVelVec; }
		inline LegVelVec VecFromAbsVel(const AbsLegPoseVel& ALPV) { LegVelVec ALPVelVec; VecFromAbsVel(ALPV, ALPVelVec); return ALPVelVec; }
		inline LegVelVec VecFromInvVel(const InvLegPoseVel& ILPV) { LegVelVec ILPVelVec; VecFromInvVel(ILPV, ILPVelVec); return ILPVelVec; }
		inline LegVelVec VecFromTipVel(const LegTipPoseVel& LTPV) { LegVelVec LTPVelVec; VecFromTipVel(LTPV, LTPVelVec); return LTPVelVec; }

		// Conversion: Leg velocity vector --> Leg pose velocity
		inline void JointVelFromVec(const LegVelVec& JLPVelVec, JointLegPoseVel& JLPV) { JLPV.hipYawVel = JLPVelVec.coeff(0); JLPV.hipRollVel = JLPVelVec.coeff(1); JLPV.hipPitchVel = JLPVelVec.coeff(2); JLPV.kneePitchVel = JLPVelVec.coeff(3); JLPV.anklePitchVel = JLPVelVec.coeff(4); JLPV.ankleRollVel = JLPVelVec.coeff(5); }
		inline void AbsVelFromVec(const LegVelVec& ALPVelVec, AbsLegPoseVel& ALPV) { ALPV.angleXVel = ALPVelVec.coeff(0); ALPV.angleYVel = ALPVelVec.coeff(1); ALPV.angleZVel = ALPVelVec.coeff(2); ALPV.footAngleXVel = ALPVelVec.coeff(3); ALPV.footAngleYVel = ALPVelVec.coeff(4); ALPV.retractionVel = ALPVelVec.coeff(5); }
		inline void InvVelFromVec(const LegVelVec& ILPVelVec, InvLegPoseVel& ILPV) { ILPV.ankleVel = ILPVelVec.head<3>(); ILPV.footAngVel = ILPVelVec.tail<3>(); }
		inline void TipVelFromVec(const LegVelVec& LTPVelVec, LegTipPoseVel& LTPV) { LTPV.vel = LTPVelVec.head<3>(); LTPV.angVel = LTPVelVec.tail<3>(); }
		inline JointLegPoseVel JointVelFromVec(const LegVelVec& JLPVelVec, LimbIndex limbIndex) { JointLegPoseVel JLPV(limbIndex); JointVelFromVec(JLPVelVec, JLPV); return JLPV; }
		inline AbsLegPoseVel AbsVelFromVec(const LegVelVec& ALPVelVec, LimbIndex limbIndex) { AbsLegPoseVel ALPV(limbIndex); AbsVelFromVec(ALPVelVec, ALPV); return ALPV; }
		inline InvLegPoseVel InvVelFromVec(const LegVelVec& ILPVelVec, LimbIndex limbIndex) { InvLegPoseVel ILPV(limbIndex); InvVelFromVec(ILPVelVec, ILPV); return ILPV; }
		inline LegTipPoseVel TipVelFromVec(const LegVelVec& LTPVelVec, LimbIndex limbIndex) { LegTipPoseVel LTPV(limbIndex); TipVelFromVec(LTPVelVec, LTPV); return LTPV; }

		//
		// Pose velocity Jacobian conversions
		//

		// Conversion: Jacobian solver --> Jacobian
		inline LegJacobian LegJacobFromSolver(const LegJacobianSolver& LJS) { return LJS.inverse(); } // Note: It is numerically advised whenever possible to use the LegJacobianSolver directly instead of the LegJacobian!
	}
}

#endif
// EOF