// Humanoid kinematics - Generic pose classes
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef POSE_CLASSES_H
#define POSE_CLASSES_H

// Includes
#include <humanoid_kinematics/kinematics_common.h>
#include <rot_conv/rot_conv.h>
#include <type_traits>
#include <vector>

// Humanoid kinematics namespace
namespace humanoid_kinematics
{
	/**
	* @namespace pose_classes
	*
	* @brief Humanoid kinematics pose classes namespace.
	**/
	namespace pose_classes
	{
		//
		// Root base classes
		//

		//! Base class of all pose classes
		class Pose
		{
		public:
			// Constructor/destructor
			Pose() = default;
			virtual ~Pose() = default;

			// Number of fields in pose
			virtual std::size_t numFields() const = 0;

			// Field name functions
			virtual void appendFieldNames(std::vector<std::string>& names) const = 0;
			virtual void appendFieldNamesShort(std::vector<std::string>& names) const = 0;
			void getFieldNames(std::vector<std::string>& names) const { names.clear(); appendFieldNames(names); }
			void getFieldNamesShort(std::vector<std::string>& names) const { names.clear(); appendFieldNamesShort(names); }

			// Zero function
			virtual void setZero() = 0; // Note: This should set the object to a well-defined zero pose, even if this does not have every field as actually numerically zero

			// Pose field array serialisation
			virtual void toArray(double* array) const = 0;   // The caller must ensure that 'array' points to at least numFields() elements
			virtual void fromArray(const double* array) = 0; // The caller must ensure that 'array' points to at least numFields() elements

			// Pose field vector serialisation
			void fromVector(const std::vector<double>& vec) { if(vec.size() >= numFields()) fromArray(vec.data()); else setZero(); } // Note: If vec does not contain enough fields then setZero() is called
			void toVector(std::vector<double>& vec) const { vec.resize(numFields()); toArray(vec.data()); }
		};

		//! Base class of all position classes
		class PosBase
		{
		public:
			// Constructor/destructor
			PosBase() = default;
			virtual ~PosBase() = default;
		};

		//! Base class of all velocity classes
		class VelBase
		{
		public:
			// Constructor/destructor
			VelBase() = default;
			virtual ~VelBase() = default;
		};

		//! Base class of all joint classes
		class JointBase
		{
		public:
			// Constructor/destructor
			JointBase() = default;
			virtual ~JointBase() = default;
		};

		//! Base class of all abstract classes
		class AbsBase
		{
		public:
			// Constructor/destructor
			AbsBase() = default;
			virtual ~AbsBase() = default;
		};

		//! Base class of all inverse classes
		class InvBase
		{
		public:
			// Constructor/destructor
			InvBase() = default;
			virtual ~InvBase() = default;
		};

		//! Base class of all tip classes
		class TipBase
		{
		public:
			// Constructor/destructor
			TipBase() = default;
			virtual ~TipBase() = default;
		};

		//
		// Position base classes
		//

		//! Base class of all joint position classes
		class JointPosBase : public JointBase, public PosBase
		{
		public:
			// Constructor/destructor
			JointPosBase() = default;
			virtual ~JointPosBase() = default;

			// Joint information functions
			virtual void appendJointInfo(std::vector<JointInfo>& info) const = 0;
			void getJointInfo(std::vector<JointInfo>& info) const { info.clear(); appendJointInfo(info); }
		};

		//! Base class of all abstract position classes
		class AbsPosBase : public AbsBase, public PosBase
		{
		public:
			// Constructor/destructor
			AbsPosBase() = default;
			virtual ~AbsPosBase() = default;
		};

		//! Base class of all inverse position classes
		class InvPosBase : public InvBase, public PosBase
		{
		public:
			// Constructor/destructor
			InvPosBase() = default;
			virtual ~InvPosBase() = default;

			// Get functions
			virtual rot_conv::Vec3 invPos() const = 0;
			virtual rot_conv::Quat invRot() const = 0;

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) = 0;
			virtual void rotateTip(const rot_conv::Quat& q) = 0;
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) = 0;
		};

		//! Base class of all tip position classes
		class TipPosBase : public TipBase, public PosBase
		{
		public:
			// Constructor/destructor
			TipPosBase() = default;
			virtual ~TipPosBase() = default;

			// Get functions
			virtual rot_conv::Vec3 tipPos() const = 0;
			virtual rot_conv::Quat tipRot() const = 0;

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) = 0;
			virtual void rotateTip(const rot_conv::Quat& q) = 0;
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) = 0;
		};

		//
		// Velocity base classes
		//

		//! Base class of all joint velocity classes
		class JointVelBase : public JointBase, public VelBase
		{
		public:
			// Constructor/destructor
			JointVelBase() = default;
			virtual ~JointVelBase() = default;
		};

		//! Base class of all abstract velocity classes
		class AbsVelBase : public AbsBase, public VelBase
		{
		public:
			// Constructor/destructor
			AbsVelBase() = default;
			virtual ~AbsVelBase() = default;
		};

		//! Base class of all inverse velocity classes
		class InvVelBase : public InvBase, public VelBase
		{
		public:
			// Constructor/destructor
			InvVelBase() = default;
			virtual ~InvVelBase() = default;

			// Get functions
			virtual rot_conv::Vec3 invVel() const = 0;
			virtual rot_conv::AngVel invAngVel() const = 0;
		};

		//! Base class of all tip velocity classes
		class TipVelBase : public TipBase, public VelBase
		{
		public:
			// Constructor/destructor
			TipVelBase() = default;
			virtual ~TipVelBase() = default;

			// Get functions
			virtual rot_conv::Vec3 tipVel() const = 0;
			virtual rot_conv::AngVel tipAngVel() const = 0;
		};

		//
		// Limb pose base classes
		//

		//! Base class of all limb pose classes
		class LimbPose : public Pose
		{
		public:
			// Constructor/destructor
			explicit LimbPose(LimbIndex limbIndex, LimbType limbType)
			 : Pose()
			 , limbType(ensureLimbType(limbType))
			 , limbIndex(ensureLimbIndexN(limbIndex))
			 , limbSign(limbSignOf(this->limbIndex))
			 , isLeft(this->limbIndex == LEFT)
			{}
			virtual ~LimbPose() = default;

			// Assignment operator (required due to the non-static const members, assigns all non-static non-const data members of this class)
			LimbPose& operator=(const LimbPose& other) { return *this; }

			// Constant functions
			static inline std::string UnknownField() { return "UnknownField"; }
			static inline std::string UnknownFieldShort() { return "UF"; }

			// Field name functions
			virtual std::string getFieldName(std::size_t field) const = 0;
			virtual std::string getFieldNameShort(std::size_t field) const = 0;

			// Limb name
			std::string limbName() const { return std::string(LimbIndexName[limbIndex]) + LimbTypeName[limbType]; }

			// Const data members (can only be set by the constructor, or be copied from another LimbPose)
			const LimbType limbType;
			const LimbIndex limbIndex;
			const LimbSign limbSign;
			const bool isLeft;
		};

		//! Base class of all leg pose classes
		class LegPose : public LimbPose
		{
		public:
			// Constructor/destructor
			explicit LegPose(LimbIndex limbIndex) : LimbPose(ensureLimbIndex(limbIndex), LT_LEG) {}
			virtual ~LegPose() = default;
		};

		//! Base class of all arm pose classes
		class ArmPose : public LimbPose
		{
		public:
			// Constructor/destructor
			explicit ArmPose(LimbIndex limbIndex) : LimbPose(ensureLimbIndex(limbIndex), LT_ARM) {}
			virtual ~ArmPose() = default;
		};

		//! Base class of all head pose classes
		class HeadPose : public LimbPose
		{
		public:
			// Constructor/destructor
			HeadPose() : LimbPose(NEUTRAL, LT_HEAD) {}
			virtual ~HeadPose() = default;
		};

		//
		// Joint limb pose base classes
		//

		//! Base class of all joint leg pose classes
		class JointLegPoseBase : public LegPose, public JointPosBase
		{
		public:
			// Constructor/destructor
			explicit JointLegPoseBase(LimbIndex limbIndex) : LegPose(limbIndex), JointPosBase() {}
			virtual ~JointLegPoseBase() = default;
		};

		//! Base class of all joint arm pose classes
		class JointArmPoseBase : public ArmPose, public JointPosBase
		{
		public:
			// Constructor/destructor
			explicit JointArmPoseBase(LimbIndex limbIndex) : ArmPose(limbIndex), JointPosBase() {}
			virtual ~JointArmPoseBase() = default;
		};

		//! Base class of all joint head pose classes
		class JointHeadPoseBase : public HeadPose, public JointPosBase
		{
		public:
			// Constructor/destructor
			JointHeadPoseBase() : HeadPose(), JointPosBase() {}
			virtual ~JointHeadPoseBase() = default;
		};

		//
		// Abstract limb pose base classes
		//

		//! Base class of all abstract leg pose classes
		class AbsLegPoseBase : public LegPose, public AbsPosBase
		{
		public:
			// Constructor/destructor
			explicit AbsLegPoseBase(LimbIndex limbIndex) : LegPose(limbIndex), AbsPosBase() {}
			virtual ~AbsLegPoseBase() = default;
		};

		//! Base class of all abstract arm pose classes
		class AbsArmPoseBase : public ArmPose, public AbsPosBase
		{
		public:
			// Constructor/destructor
			explicit AbsArmPoseBase(LimbIndex limbIndex) : ArmPose(limbIndex), AbsPosBase() {}
			virtual ~AbsArmPoseBase() = default;
		};

		//! Base class of all abstract head pose classes
		class AbsHeadPoseBase : public HeadPose, public AbsPosBase
		{
		public:
			// Constructor/destructor
			AbsHeadPoseBase() : HeadPose(), AbsPosBase() {}
			virtual ~AbsHeadPoseBase() = default;
		};

		//
		// Inverse limb pose base classes
		//

		//! Base class of all inverse leg pose classes
		class InvLegPoseBase : public LegPose, public InvPosBase
		{
		public:
			// Constructor/destructor
			explicit InvLegPoseBase(LimbIndex limbIndex) : LegPose(limbIndex), InvPosBase() {}
			virtual ~InvLegPoseBase() = default;
		};

		//! Base class of all inverse arm pose classes
		class InvArmPoseBase : public ArmPose, public InvPosBase
		{
		public:
			// Constructor/destructor
			explicit InvArmPoseBase(LimbIndex limbIndex) : ArmPose(limbIndex), InvPosBase() {}
			virtual ~InvArmPoseBase() = default;
		};

		//! Base class of all inverse head pose classes
		class InvHeadPoseBase : public HeadPose, public InvPosBase
		{
		public:
			// Constructor/destructor
			InvHeadPoseBase() : HeadPose(), InvPosBase() {}
			virtual ~InvHeadPoseBase() = default;
		};

		//
		// Limb tip pose base classes
		//

		// Limb tip point types
		typedef rot_conv::Vec3 LegTipPoint; //!< Leg tip point class
		typedef rot_conv::Vec3 ArmTipPoint; //!< Arm tip point class
		typedef rot_conv::Vec3 HeadTipPoint; //!< Head tip point class

		//! Base class of all leg tip pose classes
		class LegTipPoseBase : public LegPose, public TipPosBase
		{
		public:
			// Constructor/destructor
			explicit LegTipPoseBase(LimbIndex limbIndex) : LegPose(limbIndex), TipPosBase() {}
			LegTipPoseBase(LimbIndex limbIndex, const LegTipPoint& pos, const rot_conv::Quat& rot) : LegPose(limbIndex), TipPosBase(), pos(pos), rot(rot) {} 
			virtual ~LegTipPoseBase() = default;

			// Data fields
			LegTipPoint pos;
			rot_conv::Quat rot;

			// Get functions
			virtual rot_conv::Vec3 tipPos() const override { return pos; }
			virtual rot_conv::Quat tipRot() const override { return rot; }

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) override { pos += shift; }
			virtual void rotateTip(const rot_conv::Quat& q) override { rot = q * rot; }
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) override { rot = q * rot; pos = origin + rot_conv::QuatRotVec(q, pos - origin); }
		};

		//! Base class of all arm tip pose classes
		class ArmTipPoseBase : public ArmPose, public TipPosBase
		{
		public:
			// Constructor/destructor
			explicit ArmTipPoseBase(LimbIndex limbIndex) : ArmPose(limbIndex), TipPosBase() {}
			ArmTipPoseBase(LimbIndex limbIndex, const ArmTipPoint& pos, const rot_conv::Quat& rot) : ArmPose(limbIndex), TipPosBase(), pos(pos), rot(rot) {} 
			virtual ~ArmTipPoseBase() = default;

			// Data fields
			ArmTipPoint pos;
			rot_conv::Quat rot;

			// Get functions
			virtual rot_conv::Vec3 tipPos() const override { return pos; }
			virtual rot_conv::Quat tipRot() const override { return rot; }

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) override { pos += shift; }
			virtual void rotateTip(const rot_conv::Quat& q) override { rot = q * rot; }
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) override { rot = q * rot; pos = origin + rot_conv::QuatRotVec(q, pos - origin); }
		};

		//! Base class of all head tip pose classes
		class HeadTipPoseBase : public HeadPose, public TipPosBase
		{
		public:
			// Constructor/destructor
			HeadTipPoseBase() : HeadPose(), TipPosBase() {}
			HeadTipPoseBase(const HeadTipPoint& pos, const rot_conv::Quat& rot) : HeadPose(), TipPosBase(), pos(pos), rot(rot) {} 
			virtual ~HeadTipPoseBase() = default;

			// Data fields
			HeadTipPoint pos;
			rot_conv::Quat rot;

			// Get functions
			virtual rot_conv::Vec3 tipPos() const override { return pos; }
			virtual rot_conv::Quat tipRot() const override { return rot; }

			// Transformation functions
			virtual void shiftBy(const rot_conv::Vec3& shift) override { pos += shift; }
			virtual void rotateTip(const rot_conv::Quat& q) override { rot = q * rot; }
			virtual void rotateAround(const rot_conv::Vec3& origin, const rot_conv::Quat& q) override { rot = q * rot; pos = origin + rot_conv::QuatRotVec(q, pos - origin); }
		};

		//
		// Joint limb pose velocity base classes
		//

		//! Base class of all joint leg pose velocity classes
		class JointLegPoseVelBase : public LegPose, public JointVelBase
		{
		public:
			// Constructor/destructor
			explicit JointLegPoseVelBase(LimbIndex limbIndex) : LegPose(limbIndex), JointVelBase() {}
			virtual ~JointLegPoseVelBase() = default;
		};

		//! Base class of all joint arm pose velocity classes
		class JointArmPoseVelBase : public ArmPose, public JointVelBase
		{
		public:
			// Constructor/destructor
			explicit JointArmPoseVelBase(LimbIndex limbIndex) : ArmPose(limbIndex), JointVelBase() {}
			virtual ~JointArmPoseVelBase() = default;
		};

		//! Base class of all joint head pose velocity classes
		class JointHeadPoseVelBase : public HeadPose, public JointVelBase
		{
		public:
			// Constructor/destructor
			JointHeadPoseVelBase() : HeadPose(), JointVelBase() {}
			virtual ~JointHeadPoseVelBase() = default;
		};

		//
		// Abstract limb pose velocity base classes
		//

		//! Base class of all abstract leg pose velocity classes
		class AbsLegPoseVelBase : public LegPose, public AbsVelBase
		{
		public:
			// Constructor/destructor
			explicit AbsLegPoseVelBase(LimbIndex limbIndex) : LegPose(limbIndex), AbsVelBase() {}
			virtual ~AbsLegPoseVelBase() = default;
		};

		//! Base class of all abstract arm pose velocity classes
		class AbsArmPoseVelBase : public ArmPose, public AbsVelBase
		{
		public:
			// Constructor/destructor
			explicit AbsArmPoseVelBase(LimbIndex limbIndex) : ArmPose(limbIndex), AbsVelBase() {}
			virtual ~AbsArmPoseVelBase() = default;
		};

		//! Base class of all abstract head pose velocity classes
		class AbsHeadPoseVelBase : public HeadPose, public AbsVelBase
		{
		public:
			// Constructor/destructor
			AbsHeadPoseVelBase() : HeadPose(), AbsVelBase() {}
			virtual ~AbsHeadPoseVelBase() = default;
		};

		//
		// Inverse limb pose velocity base classes
		//

		//! Base class of all inverse leg pose velocity classes
		class InvLegPoseVelBase : public LegPose, public InvVelBase
		{
		public:
			// Constructor/destructor
			explicit InvLegPoseVelBase(LimbIndex limbIndex) : LegPose(limbIndex), InvVelBase() {}
			virtual ~InvLegPoseVelBase() = default;
		};

		//! Base class of all inverse arm pose velocity classes
		class InvArmPoseVelBase : public ArmPose, public InvVelBase
		{
		public:
			// Constructor/destructor
			explicit InvArmPoseVelBase(LimbIndex limbIndex) : ArmPose(limbIndex), InvVelBase() {}
			virtual ~InvArmPoseVelBase() = default;
		};

		//! Base class of all inverse head pose velocity classes
		class InvHeadPoseVelBase : public HeadPose, public InvVelBase
		{
		public:
			// Constructor/destructor
			InvHeadPoseVelBase() : HeadPose(), InvVelBase() {}
			virtual ~InvHeadPoseVelBase() = default;
		};

		//
		// Limb tip pose velocity base classes
		//

		// Limb tip point velocity types
		typedef rot_conv::Vec3 LegTipPointVel; //!< Leg tip point velocity class
		typedef rot_conv::Vec3 ArmTipPointVel; //!< Arm tip point velocity class
		typedef rot_conv::Vec3 HeadTipPointVel; //!< Head tip point velocity class

		//! Base class of all leg tip pose velocity classes
		class LegTipPoseVelBase : public LegPose, public TipVelBase
		{
		public:
			// Constructor/destructor
			explicit LegTipPoseVelBase(LimbIndex limbIndex) : LegPose(limbIndex), TipVelBase() {}
			LegTipPoseVelBase(LimbIndex limbIndex, const LegTipPointVel& vel, const rot_conv::AngVel& angVel) : LegPose(limbIndex), TipVelBase(), vel(vel), angVel(angVel) {} 
			virtual ~LegTipPoseVelBase() = default;

			// Data fields
			LegTipPointVel vel;
			rot_conv::AngVel angVel;

			// Get functions
			virtual rot_conv::Vec3 tipVel() const override { return vel; }
			virtual rot_conv::AngVel tipAngVel() const override { return angVel; }
		};

		//! Base class of all arm tip pose velocity classes
		class ArmTipPoseVelBase : public ArmPose, public TipVelBase
		{
		public:
			// Constructor/destructor
			explicit ArmTipPoseVelBase(LimbIndex limbIndex) : ArmPose(limbIndex), TipVelBase() {}
			ArmTipPoseVelBase(LimbIndex limbIndex, const ArmTipPointVel& vel, const rot_conv::AngVel& angVel) : ArmPose(limbIndex), TipVelBase(), vel(vel), angVel(angVel) {} 
			virtual ~ArmTipPoseVelBase() = default;

			// Data fields
			ArmTipPointVel vel;
			rot_conv::AngVel angVel;

			// Get functions
			virtual rot_conv::Vec3 tipVel() const override { return vel; }
			virtual rot_conv::AngVel tipAngVel() const override { return angVel; }
		};

		//! Base class of all head tip pose velocity classes
		class HeadTipPoseVelBase : public HeadPose, public TipVelBase
		{
		public:
			// Constructor/destructor
			HeadTipPoseVelBase() : HeadPose(), TipVelBase() {}
			HeadTipPoseVelBase(const HeadTipPointVel& vel, const rot_conv::AngVel& angVel) : HeadPose(), TipVelBase(), vel(vel), angVel(angVel) {} 
			virtual ~HeadTipPoseVelBase() = default;

			// Data fields
			HeadTipPointVel vel;
			rot_conv::AngVel angVel;

			// Get functions
			virtual rot_conv::Vec3 tipVel() const override { return vel; }
			virtual rot_conv::AngVel tipAngVel() const override { return angVel; }
		};

		//
		// Robot pose base classes
		//

		//! Base class of all robot pose classes
		class RobotPose : public Pose
		{
		public:
			// Constructor/destructor
			RobotPose() = default;
			virtual ~RobotPose() = default;

		protected:
			// Helper function for derived appendFieldNames() implementations to process child limb poses
			template<typename Type, typename... Types> static void appendFieldNamesHelper(std::vector<std::string>& names, const Type& arg, Types&&... args)
			{
				// Append all pose field names in sequence
				static_assert(std::is_base_of<LimbPose, Type>::value, "All arguments to appendFieldNamesHelper() must derive from LimbPose!");
				std::vector<std::string> tempNames;
				appendFieldNamesHelper(tempNames, names, arg, std::forward<Types>(args)...);
			}

			// Helper function for derived appendFieldNamesShort() implementations to process child limb poses
			template<typename Type, typename... Types> static void appendFieldNamesShortHelper(std::vector<std::string>& names, const Type& arg, Types&&... args)
			{
				// Append all short pose field names in sequence
				static_assert(std::is_base_of<LimbPose, Type>::value, "All arguments to appendFieldNamesShortHelper() must derive from LimbPose!");
				std::vector<std::string> tempNames;
				appendFieldNamesShortHelper(tempNames, names, arg, std::forward<Types>(args)...);
			}

			// Helper function for derived toArray() implementations to process child poses
			template<typename Type, typename... Types> static double* toArrayHelper(double* array, const Type& arg, Types&&... args)
			{
				// Recursively transcribe all poses to the array in sequence
				static_assert(std::is_base_of<Pose, Type>::value, "All arguments to toArrayHelper() must derive from Pose!");
				arg.toArray(array);
				return toArrayHelper(array + arg.numFields(), std::forward<Types>(args)...);
			}

			// Helper function for derived fromArray() implementations to process child poses
			template<typename Type, typename... Types> static const double* fromArrayHelper(const double* array, Type& arg, Types&&... args)
			{
				// Recursively transcribe all poses from the array in sequence
				static_assert(std::is_base_of<Pose, Type>::value, "All arguments to fromArrayHelper() must derive from Pose!");
				arg.fromArray(array);
				return fromArrayHelper(array + arg.numFields(), std::forward<Types>(args)...);
			}

		private:
			// Internal helper function for derived appendFieldNames() implementations to process child limb poses
			static void appendFieldNamesHelper(std::vector<std::string>& tempNames, std::vector<std::string>& names) {}
			template<typename Type, typename... Types> static void appendFieldNamesHelper(std::vector<std::string>& tempNames, std::vector<std::string>& names, const Type& arg, Types&&... args)
			{
				// Recursively append all pose field names in sequence
				static_assert(std::is_base_of<LimbPose, Type>::value, "All arguments to appendFieldNamesHelper() must derive from LimbPose!");
				const char* prefix = LimbIndexName[arg.limbIndex];
				arg.getFieldNames(tempNames);
				for(const std::string& temp : tempNames)
					names.push_back(prefix + temp);
				appendFieldNamesHelper(tempNames, names, std::forward<Types>(args)...);
			}

			// Internal helper function for derived appendFieldNamesShort() implementations to process child limb poses
			static void appendFieldNamesShortHelper(std::vector<std::string>& tempNames, std::vector<std::string>& names) {}
			template<typename Type, typename... Types> static void appendFieldNamesShortHelper(std::vector<std::string>& tempNames, std::vector<std::string>& names, const Type& arg, Types&&... args)
			{
				// Recursively append all short pose field names in sequence
				static_assert(std::is_base_of<LimbPose, Type>::value, "All arguments to appendFieldNamesShortHelper() must derive from LimbPose!");
				char prefix = LimbIndexChar[arg.limbIndex];
				arg.getFieldNamesShort(tempNames);
				for(const std::string& temp : tempNames)
					names.push_back(prefix + temp);
				appendFieldNamesShortHelper(tempNames, names, std::forward<Types>(args)...);
			}

			// Internal helper function for derived toArray() implementations to process child poses
			static double* toArrayHelper(double* array) { return array; }

			// Internal helper function for derived fromArray() implementations to process child poses
			static const double* fromArrayHelper(const double* array) { return array; }
		};

		//
		// Robot pose position base classes
		//

		//! Base class of all robot joint pose classes
		class JointPoseBase : public RobotPose
		{
		public:
			// Constructor/destructor
			JointPoseBase() = default;
			virtual ~JointPoseBase() = default;

			// Joint information functions
			virtual void appendJointInfo(std::vector<JointInfo>& info) const = 0;
			void getJointInfo(std::vector<JointInfo>& info) const { info.clear(); appendJointInfo(info); }

		protected:
			// Helper function for derived appendJointInfo() implementations to process child joint poses
			template<typename Type, typename... Types> static void appendJointInfoHelper(std::vector<JointInfo>& info, const Type& arg, Types&&... args)
			{
				// Recursively append information about all joints in sequence
				static_assert(std::is_base_of<JointPosBase, Type>::value, "All arguments to appendJointInfoHelper() must derive from JointPosBase!");
				arg.appendJointInfo(info);
				appendJointInfoHelper(info, std::forward<Types>(args)...);
			}

		private:
			// Internal helper function for derived appendJointInfo() implementations to process child joint poses
			static void appendJointInfoHelper(std::vector<JointInfo>& info) {}
		};

		//! Base class of all robot abstract pose classes
		class AbsPoseBase : public RobotPose
		{
		public:
			// Constructor/destructor
			AbsPoseBase() = default;
			virtual ~AbsPoseBase() = default;
		};

		//! Base class of all robot inverse pose classes
		class InvPoseBase : public RobotPose
		{
		public:
			// Constructor/destructor
			InvPoseBase() = default;
			virtual ~InvPoseBase() = default;
		};

		//! Base class of all robot tip pose classes
		class TipPoseBase : public RobotPose
		{
		public:
			// Constructor/destructor
			TipPoseBase() = default;
			virtual ~TipPoseBase() = default;
		};

		//
		// Robot pose velocity base classes
		//

		//! Base class of all robot joint pose velocity classes
		class JointPoseVelBase : public RobotPose
		{
		public:
			// Constructor/destructor
			JointPoseVelBase() = default;
			virtual ~JointPoseVelBase() = default;
		};

		//! Base class of all robot abstract pose velocity classes
		class AbsPoseVelBase : public RobotPose
		{
		public:
			// Constructor/destructor
			AbsPoseVelBase() = default;
			virtual ~AbsPoseVelBase() = default;
		};

		//! Base class of all robot inverse pose velocity classes
		class InvPoseVelBase : public RobotPose
		{
		public:
			// Constructor/destructor
			InvPoseVelBase() = default;
			virtual ~InvPoseVelBase() = default;
		};

		//! Base class of all robot tip pose velocity classes
		class TipPoseVelBase : public RobotPose
		{
		public:
			// Constructor/destructor
			TipPoseVelBase() = default;
			virtual ~TipPoseVelBase() = default;
		};
	}
}

#endif
// EOF