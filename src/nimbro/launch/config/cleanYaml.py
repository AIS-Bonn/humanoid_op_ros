#!/usr/bin/python
# Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# Imports
import os.path
from yamlUtils import *
from termcolor import colored


# Function to clean a YAML file
def clean(filepath):

	# Check if the yaml file exists
	if not os.path.isfile(filepath):
		error("Specified YAML file '" + filepath + "' not found!")
		return
    
	# Load the yaml file
	data = readYAML(filepath)

	# Save the yaml file back again
	writeYAML(filepath, data)

	# Indicate to the user that we have cleaned the yaml file
	print colored('Cleaned: ' + filepath, 'green')


# Main function
def main():

	# Complain if we have no YAML files to clean
	argc = len(sys.argv)
	if argc < 2:
		error('Please specify at least one YAML file to clean!', True)

	# Clean all the specified YAML files
	for yfile in sys.argv[1:]:
		clean(yfile)


# Run the main
if __name__ == '__main__':
	main()
# EOF