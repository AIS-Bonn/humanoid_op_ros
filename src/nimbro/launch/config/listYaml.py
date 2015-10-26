#!/usr/bin/python
# List the entries in a yaml file.
# Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
# Usage: listYaml.py path/to/file.yaml [depth]

# Imports
import sys
import yaml
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
	yf = file(argv, 'r')
	data = yaml.load(yf)
	listEntries(data, '/', depth)

# Run the main
if __name__ == '__main__':
	argc = len(sys.argv)
	if argc < 2:
		print colored('Please specify a yaml file to list the variables of as the first command line argument.', 'red')
		sys.exit()
	if argc >= 3:
		depth = int(sys.argv[2].strip())
	else:
		depth = 0
	main(sys.argv[1], depth)
# EOF