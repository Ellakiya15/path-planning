def RRT():
    nodes = []
    path = []
    add_node(0, *self.start_pose)
    sn = self.nodes[0]
    #1: add the start poseas a node
    self.gn = None
    cnt = 0
    while(self.gn is None):
        cnt = cnt+1
        #2 sample a random node with goal bias
        if(random.random() < goal_bias):
            q_rand = goal_pose
        else:
            q_rand = sample_Xrand()

        #3 expand
        expand(q_rand[0],q_rand[1])

        #4 keep checking if self.gn (Goal Node) is still None or not, this is updated when we reach the goal
        if(self.gn is not None):
             #5 construct the path
            construct_path(sn,gn)
            break
def expand(x, y):
    #1: Find the nearest node to x,y
    #2 step towards it and during step see if its reaching the goal, this is already implemented in step but do read the code and understand it
    pos =[x, y]
    q_near = nearest(pos)
    step = step(q_near, pos,step_size=15)

def sample_Xrand(self):
      x = float(random.uniform(0, mapx))
      y = float(random.uniform(0, mapy))
      return x, y
