// Utilities for unit testing
// File: test_utilities.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <sstream>
#include <test_utilities/test_utilities.h>

// Namespaces
using namespace testutilities;

//
// Formatting functions
//

// ANSI escape characters do not work in general in the Win32 console
#if !defined(TEST_UTIL_FORCE_COLOURS) && defined(_MSC_VER)

// Define dud implementations of the required functions
std::string testutilities::setFormat(Attr::Colour foreground, Attr::Colour background, Attr::Attribute attribute) { return ""; }
std::string testutilities::setColour(Attr::Colour foreground, Attr::Colour background) { return ""; }
std::string testutilities::setAttribute(Attr::Attribute attribute) { return ""; }
std::string testutilities::resetColour() { return ""; };
std::string testutilities::resetFormat() { return ""; };

#else

// Set a particular foreground colour, background colour and attribute
std::string testutilities::setFormat(Attr::Colour foreground, Attr::Colour background, Attr::Attribute attribute)
{
	// Work out which inputs we have
	bool haveF = (foreground != Attr::NONE  );
	bool haveB = (background != Attr::NONE  );
	bool haveA = (attribute  != Attr::NOATTR);

	// Construct the required ANSI escape code
	std::stringstream s;
	s << "\033[";
	if(haveF || haveB || haveA)
	{
		if(haveF) s << 30 + foreground;
		if(haveB)
		{
			if(haveF) s << ";";
			s << 40 + background;
		}
		if(haveA)
		{
			if(haveF || haveB) s << ";";
			s << attribute;
		}
	}
	else
	{
		s << Attr::RESET;
	}
	s << "m";

	// Return the required ANSI escape code
	return s.str();
}

// Set a particular foreground and background colour
std::string testutilities::setColour(Attr::Colour foreground, Attr::Colour background)
{
	// Return the required ANSI escape code
	return setFormat(foreground,background);
}

// Set a particular attribute
std::string testutilities::setAttribute(Attr::Attribute attribute)
{
	// Return the required ANSI escape code (unless the input is NOATTR, in which case do nothing)
	if(attribute == Attr::NOATTR)
		return "";
	else
	{
		std::stringstream s;
		s << "\033[" << attribute << "m";
		return s.str();
	}
}

// Reset all text attributes
std::string testutilities::resetColour()
{
	// Return the required ANSI escape code
	return setAttribute(Attr::RESET);
}

// Reset all text attributes
std::string testutilities::resetFormat()
{
	// Return the required ANSI escape code
	return setAttribute(Attr::RESET);
}
#endif
// EOF