#!/usr/bin/python
# Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# Imports
import sys
import yaml
import os.path
from termcolor import colored

# Function to clean a YAML file
def clean(filename):
	
	# Check if yaml file exists
	if not os.path.isfile(filename):
		print colored("Specified YAML file '" + filename + "' not found!", "red")
		return
    
	# Load the yaml file
	yf = file(filename, 'r')
	data = yaml.load(yf)

	# Save the yaml file back again
	ofile = file(filename, 'w')
	yaml.dump(data, ofile, default_flow_style=False, width=5000)

	# Indicate to the user that we have cleaned the yaml file
	print colored('Cleaned: ' + filename, 'green')

# Main function
def main():
	
	# Complain if we have no YAML files to clean
	argc = len(sys.argv)
	if argc < 2:
		print colored('Please specify at least one YAML file to clean!', 'red')
		sys.exit()

	# Clean all the specified YAML files
	for yfile in sys.argv[1:]:
		clean(yfile)

# Run the main
if __name__ == '__main__':
	main()
# EOF