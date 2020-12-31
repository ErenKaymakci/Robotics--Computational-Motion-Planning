import numpy as np
import matplotlib.pyplot as plt
import time 

class AStarGrid():

    def definitions(self,input_map,start_coords,dest_coords,drawMap):

        self.nrows,self.ncols = input_map.shape 

        self.start_coords = start_coords
        self.dest_coords = dest_coords

        self.map = np.ones((self.nrows,self.ncols)) 
        
        self.g = np.full([self.nrows,self.ncols],np.inf) # matrix of g values
        self.f = np.full([self.nrows,self.ncols],np.inf) # matrix of f values
        self.parent = np.full([self.nrows,self.ncols],np.inf) 

        # defining X,Y axis 
        y = np.linspace(1, self.nrows, self.ncols)
        x = np.linspace(1, self.nrows, self.ncols) 
        self.X,self.Y = np.meshgrid(x,y)
        
        
        xd = dest_coords[1]+1 
        yd = dest_coords[0]+1
        
        self.H = abs(self.X-xd) + abs(self.Y-yd) # Heuristic function 
        self.implement(input_map,drawMap)

    def sub2ind(self,matrix_shape,row,col):
        # It is similar to MATLAB sub2ind function
        return row*matrix_shape[1] + col
    
    def ind2sub(self,array_shape, ind):
        # It is similar to MATLAB ind2sub function
        rows = (np.int32(ind) // array_shape[1])
        cols = (np.int32(ind) % array_shape[1])
        return (rows, cols)


    def setIndexes(self,row,col,i,j):
        self.g[row,col] = self.g[i,j] + 1
        self.f[row,col] = self.g[row,col] + self.H[row,col] # main function ==> f(x) = g(x) + h(x)
        self.parent[row,col] = self.sub2ind(self.map.shape,i,j)
        self.map[row,col]  = 4

    def implement(self,input_map,drawMap):
        self.parent[self.start_coords[0],self.start_coords[1]] = 0
        self.f[self.start_coords[0],self.start_coords[1]] = 0
        self.g[self.start_coords[0],self.start_coords[1]] = 0

        self.map[self.start_coords[0],self.start_coords[1]] = 5
        self.map[self.dest_coords[0],self.dest_coords[1]] = 6

        coords = np.where(input_map == 1) # finding coords of obstacles in input_map

        for i in range(0,len(coords[0])):
            self.map[coords[0][i],coords[1][i]] = 2 # marking obstacles with 2 
        
        numExpanded = 0
        
        while(True):

            if(drawMap):
                self.drawMapEveryTime()
        
            current = np.unravel_index(np.argmin(self.f,axis=None),self.f.shape) #finding min value index of f matrix
            min_f = self.f.min()  # getting the value of minimum 
            i,j = current # i = row , j = column

            if ((current[0] == self.dest_coords[0] and current[1] == self.dest_coords[1]) or min_f == np.inf):
                break
            
            self.f[i,j] = np.inf # remove this node from further consideration
             
            if(i-1>=0 and(self.map[i-1,j]==1 or self.map[i-1,j]==6)):
                if (self.g[i-1,j] > self.g[i,j] +1):
                    self.setIndexes(i-1,j ,i,j)
            
            if (i+1<=self.nrows and (self.map[i+1,j]==1 or self.map[i+1,j]==6)):
                if(self.g[i+1,j]> self.g[i,j]+1):
                    self.setIndexes(i+1,j, i,j)
                 
            if(j-1>=0 and (self.map[i,j-1]==1 or self.map[i,j-1]==6)):
                if(self.g[i,j-1]>self.g[i,j]+1):
                    self.setIndexes(i,j-1, i,j)
            if(j+1<=self.ncols and (self.map[i,j+1]==1 or self.map[i,j+1]==6)):
                if(self.g[i,j+1]> self.g[i,j]+1):
                    self.setIndexes(i,j+1, i,j)
            numExpanded +=1

        print(numExpanded)
        self.drawRoute()


    def drawMapEveryTime(self):
        plt.imshow(self.map)
        plt.pause(0.000000000000000001) 
        plt.show(block=False)

    def drawRoute(self):
            count = 0
            route = [self.parent[self.dest_coords[0],self.dest_coords[1]]] 
            adim = 1 

            while(adim != 0):
            
                adim = self.parent[self.ind2sub(self.map.shape,route[count])]
                route.append(adim)
                count +=1
                
            print (route[::-1])
            for i in range(0,len(route)-2):
                self.map[(self.ind2sub(self.map.shape,route[i]))] = 1.5
                plt.imshow(self.map)
                plt.pause(0.0000000000001)
                plt.show(block=False)
            plt.show()
 




