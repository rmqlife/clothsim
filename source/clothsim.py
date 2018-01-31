from arcsim_expert_python import *
from camera_transform import CameraTransform
import sys,time
from util import *

def modify_expert_cam(expert):
    R = [ -2.4512346789053452e-02, 9.9968955285579986e-01,
    4.4657323994911069e-03, 6.8043109798842805e-01,
    1.9956487329708672e-02, -7.3254027841731739e-01,
    -7.3240198371191967e-01, -1.4917658141606671e-02,
    -6.8070904043535008e-01 ]
    t = [ 3.4787262116370329e-02, -1.1033486540439220e-01,
    1.0847564970952182e+00 ]
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

class Handles:
    def __init__(self,cloth_x, cloth_y, handles, ext=0.03):
        self.cloth_x = cloth_x
        self.cloth_y = cloth_y
        self.init_handles = np.array(handles)
        self.ext = ext
        
    def noisen(self,x,n):
        from random import uniform
        ret = np.zeros(n)
        for i in range(n):
            ret[i]=uniform(-x,x)
        return ret

    def dist3d(self,p1,p2):
        return np.linalg.norm(np.subtract(p1,p2))

    def random_handles(self):
        while True:
            handles = self.init_handles.copy()
            for i in range(handles.shape[0]):
                handles[i,:] += self.noisen(x=0.1, n=3) 
            valid = 1
            valid &= self.dist3d(handles[0],handles[1])<self.cloth_x + self.ext
            valid &= self.dist3d(handles[2],handles[3])<self.cloth_x + self.ext
            valid &= self.dist3d(handles[0],handles[2])<self.cloth_y + self.ext
            valid &= self.dist3d(handles[1],handles[3])<self.cloth_y + self.ext
            #print(self.dist3d(handles[0],handles[2]),valid)
            if valid==1:
                break
        return handles
    
    def random_hands(self):
        while True:
            handles = self.init_handles.copy()
            for i in range(handles.shape[0]):
                handles[i,:] += self.noisen(x=0.1, n=3) 
            valid = 1
            valid &= self.dist3d(handles[0],handles[1])<self.cloth_x + self.ext
            if valid==1:
                break
        return handles[:2]
    
if __name__== "__main__":
    cloth_x = 0.3
    cloth_y = 0.35
    rr = [0.48943184,0.1678617,0.47914139]
    rl = [0.4918203,-0.11984081,0.47457296]

    hl = [0.55830802,-0.17057873,0.06070259]
    hr = [0.54702463,0.16470575,0.01352225]
    robot_pos=[rl,rr]
    hands_pos=[hl,hr]
    handles = np.array(hands_pos + robot_pos)
    Handles = Handles(cloth_x=cloth_x, cloth_y=cloth_y, handles=handles)
    
    path = sys.argv[1]
    delta = 0.01
    image_fag = True
    depth_fag = True
    vtk_fag = False

    expert = arcsim_expert()
    expert_func = expert.expert_flat_close
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
    expert.set_handle(handles)
    expert.advance()
    for i in range(5):#tt_handles.shape[0]):        
        hands = Handles.random_handles()[:2]
        expert.save_frame(path,img_count,image_fag,depth_fag,vtk_fag)
        for j in range(0,30):
            expert_pos = expert_func(handles,cloth_x,cloth_y)
            handles = expert.apply_hand(handles,hands,delta)
            handles = expert.apply_expert(handles,expert_pos,delta)
            expert.set_handle(handles)
            expert.advance()
            expert.save_frame(path,img_count,image_fag,depth_fag,vtk_fag)
            img_count=img_count+1
            tt_handles = stack_vector(tt_handles, np.array(handles).reshape(-1,))
            tt_expert = stack_vector(tt_expert, np.array(expert_pos).reshape(-1,))
            np.savez(os.path.join(path,"data"),handles = tt_handles, expert  =tt_expert)
            

