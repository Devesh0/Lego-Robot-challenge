from collections import deque

def BFS(x,y,Map, end):
    queue = deque([(x,y,None)])
    while len(queue)> 0:
        node = queue.popleft()
        #input()
        #print(node)
        x = node[0]              
        y = node[1]
        #print(x,y)
        if Map[x][y]==end:
            return GetPath(node)

        elif Map[x][y] != "A":
            continue
        
        Map[x][y] = "S"
        
        for i in [[x-1,y],[x+1,y],[x,y-1],[x,y+1]]: #check the neighbor
            #print(i)
            queue.append((i[0],i[1],node))
            #print(i[0],i[1])
    return []

def GetPath(node):
    path=[]  
    while (node != None):
        path.append((node[0],node[1]))
        node = node[2]         
        #print(path)
    return path
    



      
















