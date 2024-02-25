from queue import PriorityQueue
import timeit
import collections

class InputMatching:
    def main(self):
        nodes ={} 

        # RUN_ONLY_TESTCASE 1
        # with open('./input.txt', 'r') as f:
        with open('./finaltesting/input6.txt', 'r') as f:
            # Read the algorithm name
            algo = f.readline().rstrip()

            # Read the energy-level
            energy = f.readline().rstrip()

            # Read the no. of nodes
            nodecount = f.readline().rstrip()
        
            for l in range(int(nodecount)):
                line = f.readline().rstrip()

                # Splitting the line into a dictionary where the key is the node name and the value is a tuple of [x,y,z] coordinates.
                parts = line.split()
                name = parts[0]
                coordinates= [[int(parts[1]), int(parts[2]), int(parts[3])],[]]
                nodes[name] = coordinates

            # Read the no. of connections
            connectioncount = f.readline().rstrip()

            lines = f.readlines()
            for linee in lines:
                part=linee.split()
                n1=part[0]
                n2=part[1]
                nodes[n1][1].append(n2)
                nodes[n2][1].append(n1)

        # Cost Function
        def costf(node1,node2):
            distance=0
            x1= nodes[node1][0][0]
            y1=nodes[node1][0][1]

            x2=nodes[node2][0][0]
            y2=nodes[node2][0][1]

            if algo == 'A*':
                z1=nodes[node1][0][2]
                z2=nodes[node2][0][2]
            else:
                z1=0
                z2=0
                
            distance = ((x2- x1) ** 2 + (y2-y1) ** 2 + (z2-z1) ** 2) ** 0.5

            return distance

        #  BFS Starts
        def BFS(energy):
            totalenergy = int(energy)
            momentum=0
            output= ['goal']

            # current node, parent node, gparent node, momentum, current_index, parent_index
            pqueue = collections.deque([('start', '', '', momentum)])
            
            explored = {}  # Keep track of explored nodes
            while len(pqueue) != 0:
                current,parent,gparent,momentum= pqueue.popleft()
                if current == 'goal':
                    explored[current]={parent : gparent}
                    # Backtracking 
                    while parent != '':
                        if(parent in explored[current]):
                            output.append(parent)
                            temp=parent
                            parent=explored[current][parent]
                            current=temp

                    output.reverse()

                    # If goal is found return the path to the goal
                    with open('./output.txt', 'w') as f:
                        f.write(" ".join(output)) 
                        break 

                # If queue is not empty and current node is not Goal 
                if current not in explored:
                    explored[current] ={}

                if(parent in explored[current]):
                    continue

                explored[current][parent] = gparent
                
                # # When current is not goal 

                for neighbor in nodes[current][1]:
                    fmomentum=0

                    z1=nodes[current][0][-1]
                    z2=nodes[neighbor][0][-1]
                    
                    totalenergy = int(energy) + momentum

                    #If uphill and total energy is not greater than altitude difference
                    if (z2>z1 and (totalenergy < z2-z1)):
                        continue
               
                    # downhill OR uphill with total energy> altitude difference
                    # downhill case calculate fmomentum, same level z1==z2, in uphill case fmomentum remains 0
                    if z2<=z1:
                        fmomentum= z1-z2
                    
                    if neighbor not in explored:
                        pqueue.append((neighbor,current,parent,fmomentum))

                    else:
                        if(current not in explored[neighbor]):
                            pqueue.append((neighbor,current,parent,fmomentum))

            # If goal is not found/ queue becomes empty
            if(len(pqueue) == 0):
                with open('./output.txt', 'w') as f:
                    f.write("FAIL")
    
        # UCS Starts
        def UCS(energy):
            pathcost=0
            momentum=0
            totalenergy=int(energy)
            output = ['goal']
            
            open_queue = PriorityQueue()
            # pathcost, current node, parent node, gparent node, momentum, cindex, pindex
            open_queue.put((0,'start','','',0))

            closed={}
            
            while True:
                # If queue is empty
                if open_queue.empty():
                    with open('./output.txt', 'w') as f:
                        f.write("FAIL")
                        break
                
                pathcost, currnode, parnode, gparnode, momentum= open_queue.get()

                # If current node is Goal
                if currnode== 'goal':
                    closed[currnode]={parnode : gparnode}

                    #Backtracking
                    while parnode != '':
                        if(parnode in closed[currnode]):
                            output.append(parnode)
                            temp=parnode
                            parnode= closed[currnode][parnode]
                            currnode=temp
                            
                    output.reverse()

                    # Returning the path to the goal
                    with open('./output.txt', 'w') as f:
                        f.write(" ".join(output)) 
                        break 

                # If queue is not empty and current node is not Goal 
                    
                # When currnode is not goal
                if currnode not in closed:
                    closed[currnode]={}

                if(parnode in closed[currnode]):
                    continue

                closed[currnode][parnode] = gparnode

                for child in nodes[currnode][1]:

                    fmomentum = 0  

                    # momentum
                    z1=nodes[currnode][0][-1]
                    z2=nodes[child][0][-1]
                    
                    # uphill and total energy < altitude difference
                    totalenergy = energy + momentum
                    if(z2>z1 and (totalenergy <z2-z1)):
                        continue

                    # downhill OR uphill with total energy> altitude difference
                    # downhill case calculate fmomentum, same level z1==z2, in uphill case fmomentum remains 0
                    if(z2<=z1):
                        fmomentum=z1-z2
                
                    if child not in closed:
                        cost = pathcost + costf(currnode,child)
                        open_queue.put((cost,child,currnode,parnode,fmomentum))
                    
                    else:
                        if(currnode not in closed[child]):
                            cost = pathcost + costf(currnode,child)
                            open_queue.put((cost,child,currnode,parnode,fmomentum))
                        
        # AStar Starts
        def AStar(energy):
            totalcost=0
            momentum=0
            totalenergy=int(energy)
            output = ['goal']
            goal = 'goal'

            op_queue= PriorityQueue()
            # totalcost, current node, parent node, grandparent node, momentum
            op_queue.put((0,'start','','',0))

            close ={}

            while True:
                #if queue is empty
                if op_queue.empty():
                    with open('./output.txt', 'w') as f:
                        f.write("FAIL")
                        break
                
                totalcost, cnode, pnode, gpnode, momentum = op_queue.get()

                # If current node is Goal
                if cnode == 'goal':
                    close[cnode] = {pnode: gpnode}

                    #Backtracking
                    while pnode != '':
                        if(pnode in close[cnode]):
                            output.append(pnode)
                            temp=pnode
                            pnode=close[cnode][pnode]
                            cnode=temp

                    output.reverse()

                    # Returning the path to the goal
                    with open('./output.txt', 'w') as f:
                        f.write(" ".join(output)) 
                        break 

                # If queue is not empty and current node is not Goal

                # When cnode is not goal
                if cnode not in close:
                    close[cnode] = {}

                if (pnode in close[cnode]):
                    continue

                close[cnode][pnode] = gpnode

                for child in nodes[cnode][1]:
                    fmomentum = 0

                    #momentum
                    z1=nodes[cnode][0][-1]
                    z2=nodes[child][0][-1]

                    # uphill and total energy < altitude difference
                    totalenergy = energy + momentum
                    if(z2>z1 and (totalenergy <z2-z1)):
                        continue

                    # downhill OR uphill with total energy > altitude difference
                    # downhill case calculate fmomentum same level z2=z1, in uphill case fmomentum remains 0
                    if(z2<=z1):
                        fmomentum=z1-z2
                
                    if child not in close:
                        tcost = (totalcost-costf(cnode,goal)) + costf(cnode,child) + costf(child,goal)
                        op_queue.put((tcost,child,cnode,pnode,fmomentum,))
                    
                    else:
                        if(cnode not in close[child]):
                            tcost = (totalcost-costf(cnode,goal)) + costf(cnode,child) + costf(child,goal)
                            op_queue.put((tcost,child,cnode,pnode,fmomentum))

        answer= []
        if algo == "BFS":
            answer = BFS(int(energy))
        elif algo == "UCS":
            answer = UCS(int(energy))
        elif algo == "A*":
            answer =  AStar(int(energy))
    
start = timeit.default_timer()
object = InputMatching()
object.main()
print("--- %s seconds ---" % (timeit.default_timer() - start))