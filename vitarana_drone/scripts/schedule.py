import csv
import math
import numpy as np
from itertools import permutations

def main():
	global delivery
	delivery = []
	global returns
	returns = []
	global visited
	visited = {}
	with open('manifest.csv') as manifest:
		csvread = csv.reader(manifest)
		for row in csvread:
			if(row[0] == "DELIVERY"):
				delivery.append([row[1],row[2].split(";")])
			else:
				try:
					i,_ = row[-1].split(" ")
				except:
					i = row[-1]
				returns.append([i,row[1].split(";")])
		for i in range(len(delivery)):
			delivery[i][1][0] = float(delivery[i][1][0])
			delivery[i][1][1] = float(delivery[i][1][1])
			delivery[i][1][2] = float(delivery[i][1][2])
			visited[delivery[i][0]] = False

		for i in range(len(returns)):
			returns[i][1][0] = float(returns[i][1][0])
			returns[i][1][1] = float(returns[i][1][1])
			returns[i][1][2] = float(returns[i][1][2])
			visited[returns[i][0]] = False

	global initial_location
	initial_location = [18.9998102845, 72.000142461, 16.757981]
	ans = 0
	order = []
	p = list(permutations(delivery+returns))
	for i in p:
		cost = 0
		dist = 0
		curr_location = []
		if(i[0][0] < 'D'):
			for j in i:
				dist += np.sqrt(((curr_location[0]-j[1][0])*0.000004517*2)**2 + ((curr_location[1]-j[1][1])*0.0000047487*2)**2)
				if(j[0][0] < 'D'):
					curr_location = j[1]
				else:
					curr_location = initial_location
			cost = 5*18 + 0.1*dist
			if(cost > ans):
				ans = cost
				order = i
	print(i)

if __name__ == '__main__':
	main()
	
