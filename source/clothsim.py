from arcsim_expert_python import *
from camera_transform import CameraTransform
import sys,time
from util import *



def modify_expert_cam(expert):
    R = [ 1.1787705693369732e-02, 9.9990175212015187e-01,
       7.5852555353195794e-03, 7.1516001000053597e-01,
       -3.1286601584672447e-03, -6.9895376927350883e-01,
       -6.9886136686078104e-01, 1.3663732749963631e-02,
       -7.1512662677096450e-01 ]
    t = [ 6.2519535255385096e-04, -1.3248296586455230e-01,
       1.0407258265155006e+00 ]
    R = np.array(R).reshape((3,3))
    t = np.array(t).reshape((3,1))
    cam = CameraTransform(R,t)
    expert.cx = cam.camera_position[0]
    expert.cy = cam.camera_position[1]
    expert.cz = cam.camera_position[2]
    
    prop = 0
    expert.fx = expert.cx + cam.camera_orientation[0]
    expert.fy = expert.cy + cam.camera_orientation[1]
    expert.fz = expert.cz + cam.camera_orientation[2]
    
    expert.w=640
    expert.h=480
    expert.fovy=53.13
    expert.zNear=1.0
    expert.zFar=1000.0
    
    expert.lights=[LightSource([4,0,2],'color',[1,1,1])]
    #expert.lights=expert.lights+[LightSource([5,0,2],'color',[1,1,1])]
    return expert

import numpy as np
class Handles:
    def __init__(self, cloth_x, cloth_y, handles, nx=[0.1,0.1,0.1], ext=0.1):
        self.cloth_x = cloth_x
        self.cloth_y = cloth_y
        self.init_handles = np.array(handles)
        self.ext = ext
        self.nx = nx
        pass
        
    def noise3(self):
        from random import uniform
        ret = np.zeros(3)
        for i in range(3):
            ret[i]=uniform(-self.nx[i],self.nx[i])
        
        return ret

    def dist3d(self,p1,p2):
        return np.linalg.norm(np.subtract(p1,p2))
    
    def valid(self, handles):
        valid = 1
        valid &= self.dist3d(handles[0],handles[1])<self.cloth_x + self.ext
        valid &= self.dist3d(handles[2],handles[3])<self.cloth_x + self.ext
        valid &= handles[0][1]<handles[1][1]
        valid &= handles[2][1]<handles[3][1]
        #valid &= self.dist3d(handles[1],handles[3])<self.cloth_y + self.ext
        return valid==1
        
    def random_handles(self):
        while True:
            handles = self.init_handles.copy()
            #for i in range(handles.shape[0]):
            n1 = self.noise3() 
            n2 = self.noise3()
            n3 = self.noise3()
            n4 = self.noise3()
            handles[1,:] += n1
            handles[0,:] += n1
            handles[2,:] += n4
            handles[3,:] += n4

            #print(self.dist3d(handles[0],handles[2]),valid)
            if self.valid(handles):
                break
        return handles
    
    def random_robot(self):
        while True:
            handles = self.init_handles.copy()
            for i in range(2,4):
                handles[i,:] += self.noise3()
            if self.valid(handles):
                break
        return handles
    
    def random_hands(self):
        while True:
            handles = self.init_handles.copy()
            for i in range(0,2):
                handles[i,:] += self.noise3() 
            if self.valid(handles):
                break
        return handles
    
    def series(self,pos_num):
        tt_handles = np.zeros((pos_num,4*3))
        tt_handles[0,:] = np.array(handles).reshape(-1,)
        print(pos_num)
        for i in range(0,pos_num):
            #print(tt_pos)
            res = self.random_robot()
            tt_handles[i,:] = res.reshape(-1,)
        return tt_handles


def add_bg(im,bg):
    mask = (im==0).astype('bool')
    #res = cv2.bitwise_and(bg,bg,mask=mask)
    return bg*mask+im
    
if __name__== "__main__":
    cloth_x = 0.3
    cloth_y = 0.35

    
    rr = [0.48943184,0.1678617,0.47914139]
    rl = [0.4918203,-0.11984081,0.47457296]
    
    hl = [0.54506982,-0.20150409, 0.09595238]
    hr = [0.52955174, 0.13494965, 0.09382752]
    robot_pos = np.load("robot_pos.npy")
    rl = robot_pos[0,-3:]
    rr = robot_pos[0,:3]
    
    #hl = rl+[0,0,-cloth_y]
    #hr = rr+[0,0,-cloth_y]

    hl =  [0.65536936, -0.10399476, 0.27866456]
    hr =  [0.60926911, 0.16645341, 0.30914943]
    
    hl =  [0.65536936, -0.15, 0.27866456]
    hr =  [0.60926911, 0.15, 0.30914943]
    rr = [0.48943184,0.15,0.47914139]
    rl = [0.4918203,-0.15,0.47457296]
    robot_pos=[rl,rr]
    hands_pos=[hl,hr]
    
    handles = np.array(hands_pos + robot_pos)    
    bias = [0,-0.,0]
    
    handles = np.array(hands_pos + robot_pos)
    for i in range(handles.shape[0]):
        handles[i]+=bias
    

    Handles = Handles(cloth_x=cloth_x, cloth_y=cloth_y, handles=handles)
    print("init handles", handles)
    path = sys.argv[1]
    delta = 0.02
    image_fag = True
    depth_fag = True
    vtk_fag = False

    expert = arcsim_expert()
    expert_func = expert.expert_flat
    expert_func = expert.expert_twist
    # expert_func = expert.expert_arc
    expert_func = expert.expert_flat
    # change the camera configuration by real world parameters
    expert = modify_expert_cam(expert)
    
    
    if not os.path.exists('./python_sheet'):
        obs =   \
        [ 
            1,  #this is a box, desk
            10.0,10.0,0.3,  #x,y,z extent
            0,  #this is a capsule
            0.1,0.5     #y length, radius
        ]
        expert.create_sheet(x=cloth_x,resX=10,y=cloth_y,resY=10,
                            path = "./python_sheet",
                            g = [0,0,-9.81],
                            w = [0,0,0], wden = 0, wdrag = 0,
                            obs=obs, nrObs=0,
                            oneside=False,remesh=True)
    
    motions=[]
    expert.setup("./python_sheet/sheet.json", motions)    
    if os.path.exists(path):
        shutil.rmtree(path)
    os.mkdir(path)
    img_count=0
    
    handles = handles.tolist()
    
    tt_handles = np.array([])
    tt_expert = np.array([])
    tt_depth = np.array([])
    target_size = (64,64)
    bg = cv2.imread('bg.png',cv2.IMREAD_ANYDEPTH)
    

    expert.set_handle(handles)
    expert.advance()
    for i in range(1):#tt_handles.shape[0]):        
        rand_handles = Handles.random_handles()
	rand_handles = handles
        hands = rand_handles[:2]
        robot = rand_handles[-2:]
        for j in range(0,50):
            expert_pos = expert_func(handles,cloth_x,cloth_y)
            handles = expert.apply_hand(handles,hands,delta)
            handles = expert.apply_expert(handles,expert_pos,delta)
            expert.set_handle(handles)
            expert.advance()
            expert.save_frame(path,img_count,image_fag,depth_fag,vtk_fag)
            fn = path+"/depth%04i.png"%img_count
            d = cv2.imread(fn,cv2.IMREAD_ANYDEPTH)
            d = add_bg(d,bg)
            d = cv2.resize(d, target_size)
            d = d.reshape(-1,)
            tt_depth = stack_vector(tt_depth,d)
            
            img_count=img_count+1
            tt_handles = stack_vector(tt_handles, np.array(handles).reshape(-1,))
            tt_expert = stack_vector(tt_expert, np.array(expert_pos).reshape(-1,))
            np.savez(os.path.join(path,"data"), handles = tt_handles, expert  =tt_expert, depth=tt_depth)
