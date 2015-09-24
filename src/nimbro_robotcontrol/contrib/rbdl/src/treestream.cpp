// Dump RBDL tree to ostream
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rbdl/treestream.h>

std::ostream& operator<<(std::ostream& stream, indent& ind)
{
	if(ind._branch)
	{
		for(int i = 0; i < ind.level-1; ++i)
		{
			stream << "| ";
		}
		stream << "|-";
		ind._branch = false;
	}
	else
	{
		for(int i = 0; i < ind.level; ++i)
			stream << "| ";
	}

	return stream;
}

std::ostream& operator<<(std::ostream& stream, const RigidBodyDynamics::Math::SpatialVector& vec)
{
	stream << "(";

	for(int i = 0; i < vec.rows(); ++i)
	{
		if(fabs(vec(i)) < 0.01)
			stream << color::reset();
		else
			stream << color::green();

		stream << std::fixed << vec(i) << color::reset();

		if(i != vec.rows()-1)
			stream << ", ";
	}

	stream << ")";
	return stream;
}

std::ostream& operator<<(std::ostream& stream, const RigidBodyDynamics::Joint& joint)
{
	if(joint.mJointType == RigidBodyDynamics::JointTypeRevolute)
	{
		stream << "revolute with axis " << joint.mJointAxes[0];
	}
	else
		stream << "unknown";

	return stream;
}

std::ostream& operator<<(std::ostream& stream, const RigidBodyDynamics::Math::Vector3d& vec)
{
	stream << "(";

	for(int i = 0; i < vec.rows(); ++i)
	{
		if(fabs(vec(i)) < 0.01)
			stream << color::reset();
		else
			stream << color::green();

		stream << std::fixed << vec(i) << color::reset();

		if(i != vec.rows()-1)
			stream << ", ";
	}

	stream << ")";
	return stream;
}

void dumpTf(std::ostream& stream, const RigidBodyDynamics::Math::SpatialTransform& tf)
{
	stream << "[" << tf.r << " rot " << tf.E.transpose().eulerAngles(0, 1, 2).transpose() << "]";
}



TreeStream::TreeStream(std::ostream* stream)
 : m_stream(stream)
{
}

void TreeStream::dumpElement(const RigidBodyDynamics::Model& model, int index, indent ind)
{
	const RigidBodyDynamics::Body& body = model.mBodies[index];

	(*m_stream) << ind << "Joint at ";
	dumpTf(*m_stream, model.X_T[index]);
	(*m_stream) << ": " << model.mJoints[index] << "\n";

	(*m_stream) << ind << "connected to Body '" << color::brown() << model.GetBodyName(index) << color::reset() << "'\n";
	(*m_stream) << ind << "Inertial: " << std::fixed << body.mMass << "kg at " << body.mCenterOfMass << "\n";

	for(size_t i = 0; i < model.mu[index].size(); ++i)
	{
		dumpElement(model, model.mu[index][i], ind.branch());
	}

	for(size_t i = 0; i < model.mFixedBodies.size(); ++i)
	{
		const RigidBodyDynamics::FixedBody& body = model.mFixedBodies[i];
		if((int)body.mMovableParent == index)
			dumpFixedElement(model, i, ind.branch());
	}
}

void TreeStream::dumpFixedElement(const RigidBodyDynamics::Model& model, int index, indent ind)
{
	const RigidBodyDynamics::FixedBody& body = model.mFixedBodies[index];

	(*m_stream) << ind << "Fixed joint at ";
	dumpTf(*m_stream, body.mParentTransform);
	(*m_stream) << "\n";

	(*m_stream) << ind << "connected to Body '" << color::brown() << model.GetBodyName(model.fixed_body_discriminator + index) << color::reset() << "'\n";
}


TreeStream& TreeStream::operator<<(const RigidBodyDynamics::Model& model)
{
	(*m_stream) << color::cyan();
	(*m_stream) << "============================= TREE DUMP =============================\n";
	(*m_stream) << color::reset();

	dumpElement(model, 0, indent());

	(*m_stream) << color::cyan();
	(*m_stream) << "=====================================================================\n";
	(*m_stream) << color::reset() << std::flush;
	return *this;
}
