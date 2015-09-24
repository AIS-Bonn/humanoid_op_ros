// URDF to RBDL parser
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

#ifndef RBDL_PARSER_H
#define RBDL_PARSER_H

#include <string>

#include <rbdl/Model.h>

// URDF class forward declarations
namespace urdf { class Model; class Joint; class Link; }

// RBDL parser namespace
namespace rbdl_parser
{

// URDF to RBDL model parser class
class URDF_RBDL_Model : public RigidBodyDynamics::Model
{
public:
	// Constructor/destructor
	URDF_RBDL_Model();
	virtual ~URDF_RBDL_Model();

	/**
	 * Parse an URDF model into a RBDL tree.
	 *
	 * This method also calls Init(), so you don't need to do that.
	 *
	 * @param root Decide which URDF link should be the root of the returned
	 *    RBDL tree. This can also be an URDF leaf if you want to reorder
	 *    your tree.
	 **/
	virtual bool initFrom(const urdf::Model& model, const std::string& root = "");

	// Joint information functions
	unsigned int jointIndex(const std::string& name);
	std::string jointName(unsigned int index);

	// Like jointIndex(), but return -1 if not found
	int findJointIndex(const std::string& name);

	// Wrap the gravity parameter to allow code to be more understandable
	inline void resetRBDLGravity() { gravity.setZero(); }
	inline void setRBDLGravity(const RigidBodyDynamics::Math::Vector3d& gravInRBDLRoot) { gravity = gravInRBDLRoot; }
	inline RigidBodyDynamics::Math::Vector3d getRBDLGravity() const { return gravity; }

	// Get functions for the used URDF root
	inline const std::string& URDFRootName() const { return m_nameURDFRoot; }
	inline unsigned int URDFRootIndex() const { return m_indexURDFRoot; }

protected:
	/**
	 * @brief Hook to get joint information from URDF.
	 *
	 * The default implementation does nothing. It might be interesting to
	 * override if you want to create a mapping between joint indexes and
	 * URDF joint names.
	 **/
	virtual void setupJoint(unsigned int index, const urdf::Joint& urdf, bool reverse);

private:
	// Process functions
	void process(const urdf::Link& link, unsigned int parent, const RigidBodyDynamics::Math::SpatialTransform& parentJointFrame);
	void processReverse(const urdf::Link& link, unsigned int parent, const urdf::Link* parentLink, const RigidBodyDynamics::Math::SpatialTransform& parentJointFrame);

	// Joint details
	typedef std::map<std::string, unsigned int> JointMap;
	JointMap m_jointMap;
	std::vector<std::string> m_jointNames;
	
	// URDF root link
	std::string m_nameURDFRoot; // String name of the URDF root link
	unsigned int m_indexURDFRoot; // Index of the URDF root link in this single support RBDL model
};

}

#endif
