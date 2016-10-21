#!/usr/bin/python
# List the entries in a yaml file.
# Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
# Usage: listYaml.py path/to/file.yaml [depth]

# Imports
import os.path
from yamlUtils import *
from termcolor import colored


# List all entries in the array
def listEntries(data, path, depth):
	for key, value in data.iteritems():
		current = path + key
		print current
		if type(value) is dict:
			print current + '/'
			if depth != 1:
				listEntries(value, current + '/', depth - 1)


# Main function
def main(argv, depth):
	if not os.path.isfile(argv):
		sys.stderr.write(colored("Specified config file '" + argv + "'to list does not exist!\n", "red"))
		sys.exit()
	data = readYAML(argv)
	listEntries(data, '/', depth)


# Run the main
if __name__ == '__main__':
	argc = len(sys.argv)
	if argc < 2:
		error('Please specify a yaml file to list the variables of as the first command line argument.', True)
	if argc >= 3 and sys.argv[2]:
		reqDepth = int(sys.argv[2].strip())
	else:
		reqDepth = 0
	main(sys.argv[1], reqDepth)
# EOF