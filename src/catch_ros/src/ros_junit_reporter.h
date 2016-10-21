// Customized JUnit XML reporter for use with the catkin testing framework
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

// Based on the JUnit reporter from the Catch framework, written by
// Phil Nash. The original source is licensed under the Boost software license.

#ifndef CATCH_ROS_ROS_JUNIT_REPORTER_H
#define CATCH_ROS_ROS_JUNIT_REPORTER_H


// Due to the single-header structure of Catch, this file has to be included
// in the main source file defining CATCH_CONFIG_MAIN/RUNNER.
// If you like auto-completion/static code checking/whatever, just define
// __IDE__ to include the internal Catch stuff here.
#ifdef __IDE__
# define CATCH_IMPL 1
#endif
#include <catch_ros/catch.hpp>

#include <assert.h>

#include "meta_info.h"

namespace catch_ros
{

class ROSReporter : public Catch::CumulativeReporterBase {
public:
	ROSReporter( Catch::ReporterConfig const& _config )
	 : CumulativeReporterBase( _config )
	 , xml( _config.stream() )
	{
		Catch::ReporterConfig consoleConfig(_config.fullConfig(), consoleOut);
		console = new Catch::ConsoleReporter(consoleConfig);
	}

	~ROSReporter()
	{
		std::cerr << consoleOut.str() << std::flush;
		delete console;
	}

	static std::string getDescription()
	{
		return "Reports test result in JUnit format tweaked for ROS";
	}

	virtual void noMatchingTestCases( std::string const& spec )
	{
		console->noMatchingTestCases(spec);
	}

	virtual Catch::ReporterPreferences getPreferences() const
	{
		Catch::ReporterPreferences prefs;
		prefs.shouldRedirectStdOut = true;
		return prefs;
	}

	virtual void testRunStarting( Catch::TestRunInfo const& runInfo )
	{
		Catch::CumulativeReporterBase::testRunStarting( runInfo );
		console->testRunStarting(runInfo);
		totalUnexpectedExceptions = 0;
	}

	virtual void testRunEnded( Catch::TestRunStats const& testRunStats )
	{
		Catch::CumulativeReporterBase::testRunEnded( testRunStats );

		// Hide totals if there were no errors. This is done because
		// catkin_tools reports all collected stderr, which produces a lot
		// of noise.
		if(testRunStats.totals.testCases.failed)
			console->testRunEnded(testRunStats);
	}

	virtual void testGroupStarting( Catch::GroupInfo const& groupInfo )
	{
		suiteTimer.start();
		stdOutForSuite.str("");
		stdErrForSuite.str("");
		unexpectedExceptions = 0;

		CumulativeReporterBase::testGroupStarting( groupInfo );
		console->testGroupStarting(groupInfo);
	}

	virtual void testGroupEnded( Catch::TestGroupStats const& testGroupStats )
	{
		double suiteTime = suiteTimer.getElapsedSeconds();

		CumulativeReporterBase::testGroupEnded( testGroupStats );
		console->testGroupEnded(testGroupStats);
	}

	virtual void sectionStarting( Catch::SectionInfo const& sectionInfo )
	{
		CumulativeReporterBase::sectionStarting(sectionInfo);
		console->sectionStarting(sectionInfo);
	}

	virtual void sectionEnded( Catch::SectionStats const& sectionStats )
	{
		CumulativeReporterBase::sectionEnded(sectionStats);
		console->sectionEnded(sectionStats);
	}

	virtual void assertionStarting( Catch::AssertionInfo const& assertionInfo )
	{
		CumulativeReporterBase::assertionStarting(assertionInfo);
		console->assertionStarting(assertionInfo);
	}

	virtual bool assertionEnded( Catch::AssertionStats const& assertionStats )
	{
		if( assertionStats.assertionResult.getResultType() == Catch::ResultWas::ThrewException )
		{
			unexpectedExceptions++;
			totalUnexpectedExceptions++;
		}

		console->assertionEnded(assertionStats);
		return CumulativeReporterBase::assertionEnded( assertionStats );
	}

	virtual void testCaseStarting( Catch::TestCaseInfo const& testInfo )
	{
		CumulativeReporterBase::testCaseStarting(testInfo);
		console->testCaseStarting(testInfo);
	}

	virtual void testCaseEnded( Catch::TestCaseStats const& testCaseStats )
	{
		stdOutForSuite << testCaseStats.stdOut;
		stdErrForSuite << testCaseStats.stdErr;

		CumulativeReporterBase::testCaseEnded( testCaseStats );
		console->testCaseEnded(testCaseStats);
	}

	virtual void skipTest( Catch::TestCaseInfo const& testInfo )
	{
		CumulativeReporterBase::skipTest(testInfo);
		console->skipTest(testInfo);
	}

	virtual void testRunEndedCumulative()
	{
		writeRun(*m_testRuns.back());
	}

	void writeRun( TestRunNode const& runNode )
	{
		Catch::XmlWriter::ScopedElement e = xml.scopedElement( "testsuites" );
		Catch::TestRunStats const& stats = runNode.value;

		unsigned int tests = 0;
		unsigned int failures = 0;
		unsigned int errors = 0;
		for( TestRunNode::ChildNodes::const_iterator
				it = runNode.children.begin(), itEnd = runNode.children.end();
				it != itEnd;
				++it )
		{
			failures += (*it)->value.totals.assertions.failed;
			tests += (*it)->value.totals.assertions.total();
		}

		xml.writeAttribute( "errors", totalUnexpectedExceptions );
		xml.writeAttribute( "failures", failures - totalUnexpectedExceptions );
		xml.writeAttribute( "tests", tests );

		for( TestRunNode::ChildNodes::const_iterator
				it = runNode.children.begin(), itEnd = runNode.children.end();
				it != itEnd;
				++it )
		{
			writeGroup(**it, 0);
		}
	}

	void writeGroup( TestGroupNode const& groupNode, double suiteTime )
	{
		Catch::XmlWriter::ScopedElement e = xml.scopedElement( "testsuite" );
		Catch::TestGroupStats const& stats = groupNode.value;
		xml.writeAttribute( "name", m_config->name() );
		xml.writeAttribute( "errors", unexpectedExceptions );
		xml.writeAttribute( "failures", stats.totals.assertions.failed-unexpectedExceptions );
		xml.writeAttribute( "tests", stats.totals.assertions.total() );
		xml.writeAttribute( "hostname", "tbd" ); // !TBD

		xml.writeAttribute( "package", catch_ros::meta::packageName() );

		if( m_config->showDurations() == Catch::ShowDurations::Never )
			xml.writeAttribute( "time", "" );
		else
			xml.writeAttribute( "time", suiteTime );
		xml.writeAttribute( "timestamp", "tbd" ); // !TBD

		// Write test cases
		for( TestGroupNode::ChildNodes::const_iterator
				it = groupNode.children.begin(), itEnd = groupNode.children.end();
				it != itEnd;
				++it )
			writeTestCase( **it );

		xml.scopedElement( "system-out" ).writeText( Catch::trim( stdOutForSuite.str() ), false );
		xml.scopedElement( "system-err" ).writeText( Catch::trim( stdErrForSuite.str() ), false );
	}

	void writeTestCase( TestCaseNode const& testCaseNode )
	{
		Catch::TestCaseStats const& stats = testCaseNode.value;

		// All test cases have exactly one section - which represents the
		// test case itself. That section may have 0-n nested sections
		assert( testCaseNode.children.size() == 1 );
		SectionNode const& rootSection = *testCaseNode.children.front();

		std::string className = stats.testInfo.className;

		if( className.empty() )
		{
			className = m_config->name();
		}

		if(className.empty())
			className = catch_ros::meta::packageName();
		else
			className = std::string(catch_ros::meta::packageName()) + "." + className;

		writeSection( className, "", rootSection );
	}

	void writeSection(  std::string const& className,
						std::string const& rootName,
						SectionNode const& sectionNode )
	{
		std::string name = Catch::trim( sectionNode.stats.sectionInfo.name );
		if( !rootName.empty() )
			name = rootName + "/" + name;

		if( !sectionNode.assertions.empty() ||
			!sectionNode.stdOut.empty() ||
			!sectionNode.stdErr.empty() )
		{
			Catch::XmlWriter::ScopedElement e = xml.scopedElement( "testcase" );
			if( className.empty() ) {
				xml.writeAttribute( "classname", name );
				xml.writeAttribute( "name", "root" );
			}
			else {
				xml.writeAttribute( "classname", className );
				xml.writeAttribute( "name", name );
			}
			xml.writeAttribute( "time", Catch::toString( sectionNode.stats.durationInSeconds ) );

			writeAssertions( sectionNode );

			if( !sectionNode.stdOut.empty() )
				xml.scopedElement( "system-out" ).writeText( Catch::trim( sectionNode.stdOut ), false );
			if( !sectionNode.stdErr.empty() )
				xml.scopedElement( "system-err" ).writeText( Catch::trim( sectionNode.stdErr ), false );
		}
		for( SectionNode::ChildSections::const_iterator
				it = sectionNode.childSections.begin(),
				itEnd = sectionNode.childSections.end();
				it != itEnd;
				++it )
		{
			if( className.empty() )
				writeSection( name, "", **it );
			else
				writeSection( className, name, **it );
		}
	}

	void writeAssertions( SectionNode const& sectionNode )
	{
		for( SectionNode::Assertions::const_iterator
				it = sectionNode.assertions.begin(), itEnd = sectionNode.assertions.end();
				it != itEnd;
				++it )
		{
			writeAssertion( *it );
		}
	}
	void writeAssertion( Catch::AssertionStats const& stats )
	{
		Catch::AssertionResult const& result = stats.assertionResult;
		if( !result.isOk() ) {
			std::string elementName;
			switch( result.getResultType() ) {
				case Catch::ResultWas::ThrewException:
				case Catch::ResultWas::FatalErrorCondition:
					elementName = "error";
					break;
				case Catch::ResultWas::ExplicitFailure:
					elementName = "failure";
					break;
				case Catch::ResultWas::ExpressionFailed:
					elementName = "failure";
					break;
				case Catch::ResultWas::DidntThrowException:
					elementName = "failure";
					break;

				// We should never see these here:
				case Catch::ResultWas::Info:
				case Catch::ResultWas::Warning:
				case Catch::ResultWas::Ok:
				case Catch::ResultWas::Unknown:
				case Catch::ResultWas::FailureBit:
				case Catch::ResultWas::Exception:
					elementName = "internalError";
					break;
			}

			Catch::XmlWriter::ScopedElement e = xml.scopedElement( elementName );

			xml.writeAttribute( "message", result.getExpandedExpression() );
			xml.writeAttribute( "type", result.getTestMacroName() );

			std::ostringstream oss;
			if( !result.getMessage().empty() )
				oss << result.getMessage() << "\n";
			for( std::vector<Catch::MessageInfo>::const_iterator
					it = stats.infoMessages.begin(),
					itEnd = stats.infoMessages.end();
						it != itEnd;
						++it )
			{
				if( it->type == Catch::ResultWas::Info )
					oss << it->message << "\n";
			}

			oss << "at " << result.getSourceInfo();
			xml.writeText( oss.str(), false );
		}
	}

	Catch::XmlWriter xml;
	Catch::Timer suiteTimer;
	std::ostringstream stdOutForSuite;
	std::ostringstream stdErrForSuite;
	unsigned int unexpectedExceptions;
	unsigned int totalUnexpectedExceptions;

	// Duplicate output to console
	std::stringstream consoleOut;
	Catch::ConsoleReporter* console;
};

INTERNAL_CATCH_REGISTER_REPORTER( "ros_junit", ROSReporter )

}

#endif
