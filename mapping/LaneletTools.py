#!/usr/bin/python

def deleteLine():
	fn = 'll_columbia_university.osm'
	f = open(fn)
	output = []
	rolestr="role='center'"
	idstr="ref='-*'"

	for line in f:
		x = line.find(str)
		if x > 0:

			output.append(line)
	f.close()
	
	#f = open(fn, 'w')
	#f.writelines(output)
	#f.close()
	print(output)

deleteLine()






