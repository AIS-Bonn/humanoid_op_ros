#!/usr/bin/python
# Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# Imports
import optparse
import os.path
import rospkg
import shutil
from yamlUtils import *
from termcolor import colored

# Setup
rospack = rospkg.RosPack()


# Main function
def main():

	# Default config directory
	defaultDir = os.path.join(rospack.get_path('launch'), 'config')

	# Parse the arguments
	parser = optparse.OptionParser()
	parser.add_option('-b', '--basepath', type = "string", dest = "basepath", help = "Base path", default = defaultDir)
	parser.add_option('-s', '--src', type = "string", dest = "src", help = "Source config file")
	parser.add_option('-d', '--dst', type = "string", dest = "dst", help = "Destination config file")
	parser.add_option('-e', '--element', type = "string", dest = "element", help = "Element in the source config file to copy", default = "/")
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
	srcDstString = srcFilename + " to " + dstFilename
	element = options.element

	# Handle case where entire config file should be copied
	if element == "all" and os.path.isfile(srcPath):
		shutil.copyfile(srcPath, dstPath)
		print colored("All configs copied from " + srcDstString, "green")
		sys.exit()

	# Error handling if files not found
	if not os.path.isfile(srcPath) or not os.path.isfile(dstPath):
		print colored("Please specify correct source and destination config files in the command line arguments!", "red")
		print "Src: " + srcPath
		print "Dst: " + dstPath
		sys.exit()

	# Get the components of the required config path
	elementComponents = pathToComponents(element)
	element = "/" + componentsToPath(elementComponents)
	if not elementComponents:
		error(srcDstString + ": Config parameter path is empty => Use 'all' if you want to copy everything!", True)

	# Load the source yaml file
	sData = readYAML(srcPath)

	# Get the required node of the source yaml file
	srcData = getNodeByComponents(sData, elementComponents)
	if srcData is None:
		error(srcDstString + ": Config parameter path '" + element + "' does not exist in the source file!", True)

	# Load the destination yaml file
	dData = readYAML(dstPath)

	# Write the source value into the destination data
	if not writeNodeByComponents(dData, elementComponents, srcData):
		error(srcDstString + ": Failed to write the source value of '" + element + "' into the destination YAML tree!", True)

	# Save the destination yaml file
	writeYAML(dstPath, dData)

	# Indicate that the copy was successful
	print colored("'" + element + "' configs copied from " + srcDstString, "green")


# Run the main
if __name__ == '__main__':
	main()
# EOF