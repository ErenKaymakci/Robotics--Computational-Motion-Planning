import numpy as np
import  matplotlib.pyplot as plt


class DijsktraGrid():

    def definitions(self,input_map,start_coords,dest_coords,draw):

        self.nrows,self.ncols = input_map.shape
        
        self.start_coords = start_coords
        self.dest_coords = dest_coords
        
        self.map = np.ones((self.nrows,self.ncols))
        self.parent = np.zeros((self.nrows,self.ncols))
        
        self.draw = draw

        self.implement(input_map)
        
    def ind2sub(self,array_shape, ind):
        # It is similar to MATLAB ind2sub function
        rows = (np.int32(ind) // array_shape[1])
        cols = (np.int32(ind) % array_shape[1])
        return (rows, cols)


    def sub2ind(self,matrix_shape,rows,cols):
        # It is similar to MATLAB sub2ind function
        return rows*matrix_shape[1] + cols


    def implement(self,input_map):
        
        coords = np.where(input_map == 1) 
        for i in range(0,len(coords[0])):   
            self.map[coords[0][i],coords[1][i]] = 2 # mark obstacles with 2 in map

        self.map[self.start_coords[0]][self.start_coords[1]] = 5 # mark start coordinate with 5
        self.map[self.dest_coords[0]][self.dest_coords[1]] = 6 # mark destination coordinate with 6

        distanceFromStart = np.full([self.nrows,self.ncols],np.inf)  

        distanceFromStart[self.start_coords[0],self.start_coords[1]] = 0
         
        while(True):
            
            if (self.draw):
                self.drawMapEveryTime() 
         
            self.map[self.start_coords[0]][self.start_coords[1]] = 5 
            self.map[self.dest_coords[0]][self.dest_coords[1]] = 6

            current = np.unravel_index(np.argmin(distanceFromStart,axis=None),distanceFromStart.shape) # finding minimum value index of the distanceFromStart matrix
            min_dist = distanceFromStart.min()   # getting the value of minimum    
            
            i,j = current # i = row , j = column
            
            if ((current[0] == self.dest_coords[0] and current[1] == self.dest_coords[1]) or min_dist == np.inf):
                break
            self.map[i,j] = 3 # mark current node as visited
            distanceFromStart[i,j] = np.inf # remove this node from further consideration
            
            if (i-1>=0 and  (self.map[i-1,j] != 2 and self.map[i-1,j] != 3 and self.map[i-1,j] != 5 )):
                self.map[i-1,j] = 4 
                distanceFromStart[i-1,j] = min_dist +1
                self.parent[i-1,j] = self.sub2ind(input_map.shape,i,j)

            if (i+1 <=self.nrows-1 and(self.map[i+1,j] != 2 and self.map[i+1,j] != 3 and self.map[i+1,j] != 5)):
                self.map[i+1,j] = 4
                distanceFromStart[i+1,j] = min_dist +1
                self.parent[i+1,j] = self.sub2ind(input_map.shape,i,j) 

            if (j-1 >= 0 and (self.map[i,j-1] != 2 and self.map[i,j-1]!= 3 and self.map[i,j-1] != 5)):
                self.map[i,j-1] = 4
                distanceFromStart[i,j-1] = min_dist +1 
                self.parent[i,j-1] = self.sub2ind(input_map.shape,i,j) 

            if (j+1 <= self.ncols-1 and(self.map[i,j+1] != 2 and self.map[i,j+1] != 3 and self.map[i,j+1] != 5)):
                self.map[i,j+1] = 4
                distanceFromStart[i,j+1] = min_dist+1
                self.parent[i,j+1] = self.sub2ind(input_map.shape,i,j)
                           
        self.drawRoute()
    
    def drawMapEveryTime(self):
        plt.imshow(self.map)
        plt.pause(0.000000000000000001) 
        plt.show(block=False)
    

    def drawRoute(self):
        count = 0
        route = [self.parent[self.dest_coords[0],self.dest_coords[1]]] 
        step = 1 
        while(step != 0):
            step = self.parent[self.ind2sub(self.map.shape,route[count])]
            route.append(step)
            count +=1

        print(f"route = {route[::-1]}")
  
        for i in range(0,len(route)-2):
            self.map[(self.ind2sub(self.map.shape,route[i]))] = 1.5
            plt.imshow(self.map)
            plt.pause(0.0000000000001)
            plt.show(block=False)
                
        plt.show()




   

       
