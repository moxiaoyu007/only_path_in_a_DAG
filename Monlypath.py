from collections import defaultdict
from heapq import *
import sys
import numpy
from numpy import *

def dijkstra_raw(edges, from_node, to_node):
    # edges :  ( from_node, to_node, weight of links (j,i))
    g = defaultdict(list) #build a dict whose values are lists as a default value.
    for l,r,c in edges:
        g[l].append((c,r))   # g = {l: (c1, r1), (c2, r2) }, g contains all the edges .
    q, seen = [(0,from_node,())], set()  # () is a set.
    while q:
        (cost,v1,path) = heappop(q) # heappop(q) :  delete the smallest one in heap q, and return it

        # v1 is the from_node now and path is the path now
        if v1 not in seen:
            seen.add(v1)
            path = (v1, path)
            if v1 == to_node:
                Only_path = True
                # if reach the to_node, return the cost and the path.
                #print (list(q))
                for wanted_path in list(q):   ###print (wanted_path)  ###print (wanted_path[1])
                    if wanted_path[1] == to_node:
                        Only_path = False
                        print ('there is more than one path from node %s to node %s\n' % (from_node, to_node))
                        #wanted_path[2] = (v1, wanted_path[2])
                        print ('one of other pathes is :', wanted_path[2])
                    else:
                        pass
                if Only_path:
                    print ('there is ONLY ONE PATH from node %s to node %s\n' % (from_node, to_node))
                return cost, path

            for c, v2 in g.get(v1, ()): # dict.get(key, defalut) : return the value of key, if key is not here , return default
                # get all the target_nodes of source_node now, v1.
                # c is the cost of path(v1->v2),
                if v2 not in seen:
                    # if v2 have not been seen ,in the seen set, push (cost+c, v2, path) to the heap.
                    # this loop push all target_nodes that have not been seen of source_node v1 to he heap q.
                    heappush(q, (cost+c, v2, path))
    print ('there is -  NO PATH  - from node %s to node %s !!!\n' % (from_node, to_node))
    return float("inf"),[] # if there is no path between from_node and to_node, the cost is infinite and past is null.


def dijkstra(edges, from_node, to_node):
    len_shortest_path = -1
    ret_path=[]
    length, path_queue = dijkstra_raw(edges, from_node, to_node)
    ###print (path_queue)
    if len(path_queue)>0:
        len_shortest_path = length      ## 1. Get the length firstly;
        ## 2. Decompose the path_queue, to get the passing nodes in the shortest path.
        left = path_queue[0]
        ret_path.append(left)       ## 2.1 Record the destination node firstly;
        right = path_queue[1]
        while len(right)>0:
            left = right[0]
            ret_path.append(left)   ## 2.2 Record other nodes, till the source-node.
            right = right[1]
        ret_path.reverse()  ## 3. Reverse the list finally, to make it be normal sequence.
    return len_shortest_path,ret_path

### ==================== Given a list of nodes in the topology shown in Fig. 1.
list_nodes_id = range(51);
### ==================== Given constants matrix of topology.

### M_topo is the 2-dimensional adjacent matrix used to represent a topology.
M_topo = mat(numpy.loadtxt('L1023.txt'))
M_topo[10, 1] = -1
#print (M_topo)
row, col = M_topo.shape
print (row, col)
### --- Read the topology, and generate all edges in the given topology.
edges = []
for i in range(row):
    for j in range(col):
        if i!=j and int(M_topo[i, j]) == -1 :
            edges.append((j, i, -int(M_topo[i, j])))### (i,j) is a link; M_topo[i][j] here is -1, the length of link (j,i).
###print ((-int(M_topo[1, 0]) == 1 ))

### === MAIN === ###
print ("=== Monlypath ===")
src_node = 3
target_node = 1022
print ("Let's figure out if there is A Only_path from node \033[1;35;47m%s\033[0m to node \033[1;35;47m%s\033[0m :\n" % (src_node, target_node))
length,Shortest_path = dijkstra(edges, src_node, target_node)
print ('length = ',length)
print ('The path is ',Shortest_path)
# Print with color
#print ("\033[1;35;47m%s\033[0m" % " RED")

#




















#***************************************
