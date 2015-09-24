// Dump RBDL stream to ostream
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TREESTREAM_H
#define TREESTREAM_H

#include <ostream>

#include <rbdl/Model.h>

namespace color
{
	struct color
	{
		int code;

		color(int _code) : code(_code) {}
	};

	inline color black()   { return color(30); }
	inline color red()     { return color(31); }
	inline color green()   { return color(32); }
	inline color brown()   { return color(33); }
	inline color blue()    { return color(34); }
	inline color magenta() { return color(35); }
	inline color cyan()    { return color(36); }
	inline color white()   { return color(37); }

	inline color reset()   { return color(0); }

	inline std::ostream& operator<<(std::ostream& stream, const color& c)
	{
		stream << "\x1B[" << c.code << "m";
		return stream;
	}
}

struct indent
{
	int level;
	bool _branch;
	indent()
	 : level(0)
	 , _branch(false)
	{
	}

	inline indent branch()
	{
		indent ret;
		ret.level = level+1;
		ret._branch = true;

		return ret;
	}
};

class TreeStream
{
public:
	TreeStream(std::ostream* stream);

	TreeStream& operator<<(const RigidBodyDynamics::Model& model);
private:
	void dumpElement(const RigidBodyDynamics::Model& model, int index, indent ind);
	void dumpFixedElement(const RigidBodyDynamics::Model& model, int index, indent ind);

	std::ostream* m_stream;
};

#endif
