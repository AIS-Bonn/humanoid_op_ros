// Utilities for unit testing
// File: test_utilities.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
// Refer to http://en.wikipedia.org/wiki/ANSI_escape_code for information on the escape codes being used

// Ensure header is only included once
#ifndef TEST_UTILITIES_H
#define TEST_UTILITIES_H

// Includes
#include <iostream>
#include <string>

// Macros
#ifndef DISPLAY_NEWLINE
#ifdef VERBOSE_TEST
#define DISPLAY(cmd)        std::cout << "    "; cmd // Note: Don't use inside a single-line if/for/while/etc as this expands into two lines of code
#define DISPLAY_NO_PAD(cmd) cmd
#define DISPLAY_NEWLINE     std::cout << endl;
#else
#define DISPLAY(cmd)
#define DISPLAY_NO_PAD(cmd)
#define DISPLAY_NEWLINE
#endif /* VERBOSE_TEST */
#endif /* DISPLAY_NEWLINE */

// Test utilities namespace
namespace testutilities
{
	// Attributes namespace - enumerations are 'hidden' so as to avoid possible naming conflicts, and are not in the FText class as FText(FText::MAGENTA) looks awkward.
	namespace Attr
	{
		// Enumerations
		enum Attribute
		{
			RESET = 0,
			BOLD,
			ITALIC,
			UNDERLINE,
			BLINK,
			INVERT = 7,
			CONCEAL,
			STRIKETHROUGH,
			BOLD_OFF = 22,
			ITALIC_OFF,
			UNDERLINE_OFF,
			BLINK_OFF,
			INVERT_OFF = 27,
			CONCEAL_OFF,
			STRIKETHROUGH_OFF,
			NOATTR = 110
		};
		enum Colour
		{
			BLACK = 0,
			RED,
			GREEN,
			YELLOW,
			BLUE,
			MAGENTA,
			CYAN,
			WHITE,
			DEFAULT = 9,
			NONE = 110
		};
	}

	// Formatting functions
	// These can be used with any output stream, not just cout
	// Use like this: cout << "The following word is " << setColour(Attr::MAGENTA) << "magenta" << resetColour() << "!" << endl;
	std::string setFormat(Attr::Colour foreground = Attr::NONE, Attr::Colour background = Attr::NONE, Attr::Attribute attribute = Attr::NOATTR);
	std::string setColour(Attr::Colour foreground = Attr::NONE, Attr::Colour background = Attr::NONE);
	std::string setAttribute(Attr::Attribute attribute);
	std::string resetColour(); // Both reset functions reset all colours/attributes...
	std::string resetFormat(); // The alternative naming is only included for symmetry reasons

	// Stream text formatting class
	// This class automatically writes the output in the given format to cout, then resets the formatting and appends std::endl
	// Use like this: FText(Attr::MAGENTA) << "This text is magenta!";
	class FText
	{
	public:
		// Constructors
		FText(Attr::Colour foreground = Attr::NONE, Attr::Colour background = Attr::NONE, Attr::Attribute attribute = Attr::NOATTR)
		{
			std::cout << setFormat(foreground,background,attribute);
		}
		~FText()
		{
			std::cout << resetFormat() << std::endl;
		}

		// Stream operators
		template <class T>
		FText& operator<<(const T &v) // Everything else...
		{
			std::cout << v;
			return *this;
		}
		FText& operator<<(std::ostream& (*pf)(std::ostream&)) // Type 1 manipulator
		{
			std::cout << pf;
			return *this;
		}
		FText& operator<<(std::ios& (*pf)(std::ios&)) // Type 2 manipulator
		{
			std::cout << pf;
			return *this;
		}
		FText& operator<<(std::ios_base& (*pf)(std::ios_base&)) // Type 3 manipulator
		{
			std::cout << pf;
			return *this;
		}
	};
}

#endif /* TEST_UTILITIES_H */
// EOF