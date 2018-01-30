from arcsim_expert_python import *
from camera_transform import CameraTransform
import sys,time

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

def init_handles(hands_pos,robot_pos,cloth_x,cloth_y):
    #robot_pos
    #[ 0.41399902  0.13384837  0.39804319]
    #[ 0.42313311 -0.12751294  0.37715151]
    #hands_pos
    #[ 0.66680812 -0.08721922  0.25165576]
    #[ 0.64415079  0.09218274  0.23095848]

    #robot_pos=[[0.64415079,0.09218274,0.23095848], [0.66680812,-0.08721922,0.25165576]]
    #hands_pos=[[0.41399902,0.13384837,0.39804319], [0.42313311,-0.12751294,0.37715151]]
    
    
    rr = [0.48943184,0.1678617,0.47914139]
    rl = [0.4918203,-0.11984081,0.47457296]
    
    hl = [0.55830802,-0.17057873,0.06070259]
    hr = [0.54702463,0.16470575,0.01352225]
    robot_pos=[hl,hr]
    hands_pos=[rl,rr]

    cloth_x = 0.3
    cloth_y = 0.35
    handles = hands_pos + robot_pos
    return cloth_x, cloth_y, handles, hands_pos, robot_pos
    
def init_handles_default():
    cloth_x = 0.3
    cloth_y = 0.3
    hands_pos = [[0,0,0],[cloth_x,0,0]]
    robot_pos = [[0,cloth_y,0],[cloth_x,cloth_y,0]]
    handles = hands_pos + robot_pos
    return cloth_x, cloth_y, handles, hands_pos, robot_pos
    
def random_hands(cloth_x,cloth_y,hands):
    noise=np.array([[0,0,0],[0,0,0]])
    x = cloth_x/4
    y = cloth_y
    from random import uniform
    while True:
        
        noise[0] = [uniform(-x,x),uniform(-x,x),uniform(-x,x)]
        noise[1] = [uniform(-x,x),uniform(-x,x),uniform(-x,x)]
        hands[0] += noise[0]
        hands[1] += noise[1]
        
        if np.linalg.norm(np.subtract(hands[0],hands[1])) < x:
            break
    return hands

if __name__== "__main__":
    
    robot_all_pos = np.load('robot_pos.npy')
    print(robot_all_pos.shape)
    
    #cloth_x, cloth_y,handles, hands_pos, robot_pos= init_handles_default()
    #cloth_x, cloth_y,handles, hands_pos, robot_pos= init_handles(robot_all_pos[0,:])  
    cloth_x = 0.3
    cloth_y = 0.35
    hl = [0.55830802,-0.17057873,0.06070259]
    hr = [0.54702463,0.16470575,0.01352225]
    robot_pos=[robot_all_pos[0,-3:],robot_all_pos[0,:3]]
    hands_pos=[hl,hr]
    handles = hands_pos + robot_pos
    
    print(handles)
    path = sys.argv[1]
    delta = 1
    nr_frame = 100
    nr_pass = 2

    image_fag = True
    depth_fag = True
    vtk_fag = False
    
    obs =   \
    [ 
        1,  #this is a box, desk
        10.0,10.0,0.3,  #x,y,z extent
        0,  #this is a capsule
        0.1,0.5     #y length, radius
    ]
    expert = arcsim_expert()
    expert_func = expert.expert_flat
    # change the camera configuration by real world parameters
    expert = modify_expert_cam(expert)
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
    expert.set_handle(handles)
    for i in range(0,robot_all_pos.shape[0],5):
        #pick hand location
        #hands_pos = random_hands(cloth_x,cloth_y, hands_pos)
        print("hands_pos",hands_pos)
        robot_pos=[robot_all_pos[i,-3:],robot_all_pos[i,:3]]
        print("robot_pos",robot_pos)
        #test
        expert.advance()
        #handles = expert.apply_hand(handles,hands_pos,delta)
        #expert_pos = expert_func(handles,cloth_x,cloth_y)
        #handles = expert.apply_expert(handles,robot_pos,delta)
        handles = hands_pos + robot_pos
        expert.set_handle(handles)
        expert.save_frame(path,i,image_fag,depth_fag,vtk_fag)
        img_count=img_count+1
