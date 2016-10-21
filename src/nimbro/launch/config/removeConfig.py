#!/usr/bin/python
# Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# Imports
import optparse
import os.path
import rospkg
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
	parser.add_option('-e', '--element', type = "string", dest = "element", help = "Element in the source config file to remove", default = "/")
	options, args = parser.parse_args()

	# Process the arguments
	if not options.src:
		parser.error("Source config file not specified!")
	basepath = options.basepath.rstrip('/')
	srcFilename = options.src
	srcPath = basepath + "/" + srcFilename
	element = options.element

	# Handle case where entire config file should be deleted
	if element == "all" and os.path.isfile(srcPath):
		os.remove(srcPath)
		print colored("Entire config file deleted: " + srcPath, "green")
		sys.exit()

	# Error handling if files not found
	if not os.path.isfile(srcPath):
		print colored("Please specify a correct source config file in the command line arguments!", "red")
		print "Src: " + srcFilename
		sys.exit()

	# Get the components of the required config path
	elementComponents = pathToComponents(element)
	element = "/" + componentsToPath(elementComponents)
	if not elementComponents:
		error(srcFilename + ": Config parameter path is empty => Use 'all' if you want to remove everything!", True)

	# Load the source yaml file
	sData = readYAML(srcPath)

	# Remove the required element of the source file
	if not removeNodeByComponents(sData, elementComponents):
		error(srcFilename + ": Config parameter path '" + element + "' already does not exist in the source file!", True)

	# Save the yaml file back again
	writeYAML(srcPath, sData)

	# Indicate that the remove was successful
	print colored("'" + element + "' configs removed from " + srcFilename, "green")


# Run the main
if __name__ == '__main__':
	main()
# EOF