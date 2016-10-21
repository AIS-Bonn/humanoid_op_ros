#!/usr/bin/python
# Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# Imports
import os
import optparse
import rospkg
from yamlUtils import *
from termcolor import colored
from collections import namedtuple

# Setup
rospack = rospkg.RosPack()

# Types
ConfigVar = namedtuple("ConfigVar", "path name value")
ConfigDiff = namedtuple("ConfigDiff", "path name srcValue dstValue diff")
class DiffResult:
	def __init__(self): pass
	Equal, SrcOnly, DstOnly, Changed = range(4)


# Main function
def main():

	# Default config directory
	defaultDir = os.path.join(rospack.get_path('launch'), 'config')

	# Parse the arguments
	parser = optparse.OptionParser()
	parser.add_option('-b', '--basepath', type = "string", dest = "basepath", help = "Base path", default = defaultDir)
	parser.add_option('-s', '--src', type = "string", dest = "src", help = "Source config file")
	parser.add_option('-d', '--dst', type = "string", dest = "dst", help = "Destination config file")
	parser.add_option('-e', '--element', type = "string", dest = "element", help = "Element in the config file to compare", default = "/")
	options, args = parser.parse_args()

	# Process the arguments
	if not options.src:
		parser.error("Source config file not specified!")
	if not options.dst:
		parser.error("Destination config file not specified!")
	basepath = options.basepath.rstrip('/')
	srcFilename = options.src
	dstFilename = options.dst
	srcPath = basepath + "/" + srcFilename
	dstPath = basepath + "/" + dstFilename
	element = options.element

	# Display which configs are being compared
	print
	print "COMPARE CONFIG FILES:"
	print colored("Src: " + srcPath, "yellow")
	print colored("Dst: " + dstPath, "yellow")

	# Check whether the files exist
	if not os.path.isfile(srcPath):
		print
		error("Source config file not found!", True)
	if not os.path.isfile(dstPath):
		print
		error("Destination config file not found!", True)

	# Load the source and destination config files
	srcData = readYAML(srcPath)
	dstData = readYAML(dstPath)

	# Retrieve the yaml nodes to compare
	elementComponents = pathToComponents(element)
	element = componentsToPath(elementComponents)
	elementPrefix = "/" + element if element else element
	if elementComponents:
		srcNode = getNodeByComponents(srcData, elementComponents)
		if (srcNode is None) or (not type(srcNode) is dict):
			error("Source config file does not contain the specified element to compare!", True)
		dstNode = getNodeByComponents(dstData, elementComponents)
		if (dstNode is None) or (not type(dstNode) is dict):
			error("Destination config file does not contain the specified element to compare!", True)
	else:
		srcNode = srcData
		dstNode = dstData

	# Display which element of the config files is being compared
	if element:
		print "Element: /" + element
	else:
		print "Element: <all>"
	print

	# Parse the source and destination node trees into lists and sort them (important!)
	srcConfigs = listOfLeafNodes(srcNode)
	srcConfigs.sort(key = lambda tmp: tmp.path)
	dstConfigs = listOfLeafNodes(dstNode)
	dstConfigs.sort(key = lambda tmp: tmp.path)

	# Merge and diff the source and destination configs
	configs = []
	srcIndex = 0
	dstIndex = 0
	while True:
		haveSrc = (srcIndex < len(srcConfigs))
		haveDst = (dstIndex < len(dstConfigs))
		srcConfig = srcConfigs[srcIndex] if haveSrc else None
		dstConfig = dstConfigs[dstIndex] if haveDst else None
		if not haveSrc and not haveDst:
			break
		elif haveSrc and not haveDst:
			configs.append(ConfigDiff(srcConfig.path, srcConfig.name, srcConfig.value, None, DiffResult.SrcOnly))
			srcIndex += 1
		elif haveDst and not haveSrc:
			configs.append(ConfigDiff(dstConfig.path, dstConfig.name, None, dstConfig.value, DiffResult.DstOnly))
			dstIndex += 1
		else:
			if srcConfig.path == dstConfig.path:
				diffResult = DiffResult.Equal if srcConfig.value == dstConfig.value else DiffResult.Changed
				configs.append(ConfigDiff(srcConfig.path, srcConfig.name, srcConfig.value, dstConfig.value, diffResult))
				srcIndex += 1
				dstIndex += 1
			elif srcConfig.path < dstConfig.path:
				configs.append(ConfigDiff(srcConfig.path, srcConfig.name, srcConfig.value, None, DiffResult.SrcOnly))
				srcIndex += 1
			else:
				configs.append(ConfigDiff(dstConfig.path, dstConfig.name, None, dstConfig.value, DiffResult.DstOnly))
				dstIndex += 1

	# Print a guide for interpreting the coming diff
	print "GUIDE:"
	print colored("Config in " + srcFilename + " only (source)", "green")
	print colored("Config in " + dstFilename + " only (destination)", "red")
	print colored("Config value different between " + srcFilename + " and " + dstFilename, "cyan")
	print

	# Print the calculated diff between source and destination
	identical = True
	printSeparator()
	print "DIFF RESULTS:   (" + colored(srcFilename, "yellow") + " ==> " + colored(dstFilename, "yellow") + ")"
	for config in configs:
		printConfigDiff(config, elementPrefix)
		if config.diff != DiffResult.Equal:
			identical = False
	if identical:
		print colored("Everything is equal!", "green")
		print
		print "Nothing more to do..."
		print
		return
	print

	# Give some options to the user to take action
	printSeparator()
	print "PERFORM AN ACTION:"
	print " (a) Add all " + colored("new", "green") + " configs to " + colored(dstFilename, "yellow")
	print " (r) Remove all " + colored("old", "red") + " configs from " + colored(dstFilename, "yellow")
	print " (u) Update all " + colored("new", "green") + "/" + colored("old", "red") + " configs in " + colored(dstFilename, "yellow")
	print " (m) Mirror all " + colored("new", "green") + "/" + colored("old", "red") + "/" + colored("changed", "cyan") + " configs to " + colored(dstFilename, "yellow")
	print " (c) Perform a custom merge"
	print " (q) Quit (default)"
	while True:

		# Query the user for a selection
		userChoice = raw_input("Choice? ")

		# Perform the selected action
		touchedSrc = False
		touchedDst = False
		if userChoice == "a":
			print
			print colored("Writing to " + dstFilename + ":", "yellow")
			performAdd(configs, dstNode, elementPrefix)
			touchedDst = True
			print
		elif userChoice == "r":
			print
			print colored("Writing to " + dstFilename + ":", "yellow")
			performRemove(configs, dstNode, elementPrefix)
			touchedDst = True
			print
		elif userChoice == "u":
			print
			print colored("Writing to " + dstFilename + ":", "yellow")
			performAdd(configs, dstNode, elementPrefix)
			performRemove(configs, dstNode, elementPrefix)
			touchedDst = True
			print
		elif userChoice == "m":
			print
			print colored("Writing to " + dstFilename + ":", "yellow")
			performAdd(configs, dstNode, elementPrefix)
			performRemove(configs, dstNode, elementPrefix)
			performChange(configs, dstNode, elementPrefix)
			touchedDst = True
			print
		elif userChoice == "c":
			print
			print colored("Writing to " + srcFilename + " and/or " + dstFilename + " as required:", "yellow")
			touchedSrc, touchedDst = performCustomMerge(configs, srcNode, dstNode, elementPrefix, srcFilename, dstFilename)
			print
		elif userChoice == "q" or not userChoice:
			print
		else:
			continue

		# Write the source and destination data to disk if they have been modified
		if touchedSrc:
			writeYAML(srcPath, srcData)
			print colored("Saved all changes to the source file " + srcFilename + "!", "yellow")
		else:
			print "The source file " + srcFilename + " was not modified!"
		if touchedDst:
			writeYAML(dstPath, dstData)
			print colored("Saved all changes to the destination file " + dstFilename + "!", "yellow")
		else:
			print "The destination file " + dstFilename + " was not modified!"
		print
		break


# Add all new configs from the source to the destination
def performAdd(configs, dstNode, elementPrefix):

	# Add the appropriate configs
	numAdded = 0
	for config in configs:
		if config.diff != DiffResult.SrcOnly:
			continue
		if writeNodeByPath(dstNode, config.path, config.srcValue):
			numAdded += 1
		else:
			warning("Failed to add " + elementPrefix + config.path + " with value " + config.srcValue + "!")
	if elementPrefix:
		print colored("Added " + str(numAdded) + " new config(s) to " + elementPrefix, "green")
	else:
		print colored("Added " + str(numAdded) + " new config(s)", "green")


# Remove all old configs in the destination that do not exist in the source
def performRemove(configs, dstNode, elementPrefix):

	# Remove the appropriate configs
	numRemoved = 0
	for config in configs:
		if config.diff != DiffResult.DstOnly:
			continue
		if removeNodeByPath(dstNode, config.path):
			numRemoved += 1
		else:
			warning("Failed to remove " + elementPrefix + config.path + "!")
	if elementPrefix:
		print colored("Removed " + str(numRemoved) + " old config(s) from " + elementPrefix, "red")
	else:
		print colored("Removed " + str(numRemoved) + " old config(s)", "red")


# Update all configs in the destination that have a different value than the source
def performChange(configs, dstNode, elementPrefix):

	# Change the appropriate configs
	numChanged = 0
	for config in configs:
		if config.diff != DiffResult.Changed:
			continue
		if writeNodeByPath(dstNode, config.path, config.srcValue):
			numChanged += 1
		else:
			warning("Failed to change " + elementPrefix + config.path + " to value " + config.srcValue + "!")
	if elementPrefix:
		print colored("Changed " + str(numChanged) + " config(s) in " + elementPrefix, "cyan")
	else:
		print colored("Changed " + str(numChanged) + " config(s)", "cyan")


# Perform a custom merge based on the calculated diff
def performCustomMerge(configs, srcNode, dstNode, elementPrefix, srcFilename, dstFilename):

	# Initialise how many actions of each type were performed
	numAddedSrc = 0
	numAddedDst = 0
	numRemovedSrc = 0
	numRemovedDst = 0
	numUpdatedSrc = 0
	numUpdatedDst = 0
	numNothing = 0

	# Print a guide for the response options for each diff
	print "For each diff item you will have, where applicable, the following options:"
	print " (A) Add an " + colored("old", "red") + " config back into the source file " + colored(srcFilename, "yellow")
	print " (R) Remove a " + colored("new", "green") + " config from the source file " + colored(srcFilename, "yellow")
	print " (U) Update a " + colored("changed", "cyan") + " config in the source file " + colored(srcFilename, "yellow")
	print " (a) Add a " + colored("new", "green") + " config to the destination file " + colored(dstFilename, "yellow")
	print " (r) Remove an " + colored("old", "red") + " config from the destination file " + colored(dstFilename, "yellow")
	print " (u) Update a " + colored("changed", "cyan") + " config in the destination file " + colored(dstFilename, "yellow")
	print " (n) Do nothing (default)"
	print " (q) Quit"
	print

	# Print that the custom merge is starting
	printSeparator()

	# Count how many diff items there are
	diffCount = 0
	for config in configs:
		if config.diff != DiffResult.SrcOnly and config.diff != DiffResult.DstOnly and config.diff != DiffResult.Changed:
			continue
		diffCount += 1

	# Go through each diff item and prompt the user for an action
	index = 0
	for config in configs:

		# Ignore diff items that are equal
		if config.diff != DiffResult.SrcOnly and config.diff != DiffResult.DstOnly and config.diff != DiffResult.Changed:
			continue
		index += 1

		# Print the diff item
		print "Diff item (" + str(index) + " / " + str(diffCount) + ")"
		printConfigDiff(config, elementPrefix)
		fullPath = elementPrefix + "/" + config.path

		# Query and perform the required action
		shouldQuit = False
		if config.diff == DiffResult.SrcOnly:
			while not shouldQuit:
				userChoice = raw_input("Action (R / a / n)? ")
				if userChoice == "a":
					if writeNodeByPath(dstNode, config.path, config.srcValue):
						print colored("Added " + fullPath + " with value " + toString(config.srcValue) + " to " + dstFilename + "!", "green")
						numAddedDst += 1
					else:
						warning("Failed to add " + fullPath + " with value " + toString(config.srcValue) + " to " + dstFilename + "!")
					break
				elif userChoice == "R":
					if removeNodeByPath(srcNode, config.path):
						print colored("Removed " + fullPath + " from " + srcFilename + "!", "red")
						numRemovedSrc += 1
					else:
						warning("Failed to remove " + fullPath + " from " + srcFilename + "!")
					break
				elif userChoice == "n" or not userChoice:
					numNothing += 1
					break
				elif userChoice == "q":
					shouldQuit = True
		elif config.diff == DiffResult.DstOnly:
			while not shouldQuit:
				userChoice = raw_input("Action (A / r / n)? ")
				if userChoice == "A":
					if writeNodeByPath(srcNode, config.path, config.dstValue):
						print colored("Added " + fullPath + " with value " + toString(config.dstValue) + " to " + srcFilename + "!", "green")
						numAddedSrc += 1
					else:
						warning("Failed to add " + fullPath + " with value " + toString(config.dstValue) + " to " + srcFilename + "!")
					break
				elif userChoice == "r":
					if removeNodeByPath(dstNode, config.path):
						print colored("Removed " + fullPath + " from " + dstFilename + "!", "red")
						numRemovedDst += 1
					else:
						warning("Failed to remove " + fullPath + " from " + dstFilename + "!")
					break
				elif userChoice == "n" or not userChoice:
					numNothing += 1
					break
				elif userChoice == "q":
					shouldQuit = True
		elif config.diff == DiffResult.Changed:
			while not shouldQuit:
				userChoice = raw_input("Action (U / u / n)? ")
				if userChoice == "U":
					if writeNodeByPath(srcNode, config.path, config.dstValue):
						print colored("Updated " + fullPath + " to value " + toString(config.dstValue) + " in " + srcFilename + "!", "cyan")
						numUpdatedSrc += 1
					else:
						warning("Failed to update " + fullPath + " to value " + toString(config.dstValue) + " in " + srcFilename + "!")
					break
				elif userChoice == "u":
					if writeNodeByPath(dstNode, config.path, config.srcValue):
						print colored("Updated " + fullPath + " to value " + toString(config.srcValue) + " in " + dstFilename + "!", "cyan")
						numUpdatedDst += 1
					else:
						warning("Failed to update " + fullPath + " to value " + toString(config.srcValue) + " in " + dstFilename + "!")
					break
				elif userChoice == "n" or not userChoice:
					numNothing += 1
					break
				elif userChoice == "q":
					shouldQuit = True
		print

		# Quit if required
		if shouldQuit:
			print "Quit: Did not modify anything at all..."
			print
			sys.exit()

	# Print stats about how configs were added/removed/changed
	printSeparator()
	if elementPrefix:
		print "SUMMARY OF ACTIONS:   (in " + elementPrefix + ")"
	else:
		print "SUMMARY OF ACTIONS:"
	print colored("Added " + str(numAddedSrc) + " old config(s) back into the source file " + srcFilename, "green")
	print colored("Removed " + str(numRemovedSrc) + " new config(s) from the source file " + srcFilename, "red")
	print colored("Updated " + str(numUpdatedSrc) + " config(s) in the source file " + srcFilename, "cyan")
	print colored("Added " + str(numAddedDst) + " new config(s) to the destination file " + dstFilename, "green")
	print colored("Removed " + str(numRemovedDst) + " old config(s) to the destination file " + dstFilename, "red")
	print colored("Updated " + str(numUpdatedDst) + " config(s) in the destination file " + dstFilename, "cyan")
	print "Left " + str(numNothing) + " config(s) as they were"

	# Return which data, if any, was touched
	return (numAddedSrc != 0 or numRemovedSrc != 0 or numUpdatedSrc != 0), (numAddedDst != 0 or numRemovedDst != 0 or numUpdatedDst != 0)


# Print a config diff item
def printConfigDiff(config, elementPrefix):

	# Print the required diff item
	prefix = elementPrefix + "/" + config.path + ":   "
	if config.diff == DiffResult.SrcOnly:
		print colored(prefix + toString(config.srcValue) + " ==> ???", "green")
	elif config.diff == DiffResult.DstOnly:
		print colored(prefix + "??? ==> " + toString(config.dstValue), "red")
	elif config.diff == DiffResult.Changed:
		print colored(prefix + toString(config.srcValue) + " ==> " + toString(config.dstValue), "cyan")


# Convert a dictionary tree into a list of leaf nodes
def listOfLeafNodes(node, path = ""):

	# Return a list of leaf nodes
	leafList = []
	for name, value in node.iteritems():
		itemPath = path + name
		if type(value) is dict:
			leafList.extend(listOfLeafNodes(value, itemPath + "/"))
		else:
			leafList.append(ConfigVar(itemPath, name, value))
	return leafList


# Print horizontal line
def printSeparator():

	# Print the required console separator
	print "-" * 79
	print


# Run the main
if __name__ == '__main__':
	main()
# EOF