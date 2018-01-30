from arcsim_expert_python import *
import numpy as np

def random_hands(x):
    hands = np.zeros((2,3))
    while True:
        hands[0] = [random.uniform(-x/4,x/4),random.uniform(-x/4,x/4),random.uniform(-x/4,x/4)]
        hands[1] = [random.uniform(-x/4,x/4)+x,random.uniform(-x/4,x/4),random.uniform(-x/4,x/4)]
        if np.linalg.norm(np.subtract(hands[0],hands[1])) < x:
            break
    return hands

if __name__=='__main__':
    import sys,time
    dataset_path = sys.argv[1]
    
    remesh_fag = True
    render_image = True
    render_depth = True 
    MOVEMENT_THRESH = 1e-5
    BETA = 0.8
    cloth_x = 1.0 #width of the cloth 
    cloth_y = 2.0 #length of the cloth
    delta = 0.01 # controller gain, multiplier
    nr_frame = 100 # number of frames each setup consists
    obs =   \
    [ 
        1,  #this is a box, desk
        10.0,10.0,0.3,  #x,y,z extent
        0,  #this is a capsule
        0.1,0.5     #y length, radius
    ]
    expert = arcsim_expert()
    expert.create_sheet(1,5,2,10,   \
                        "./python_sheet",[0,0,-9.81],[0,0,0],0,0,   \
                        obs,0,   \
                        True,True)
    motion0=    \
    [
        0 , #this is frame 0 at 0sec
        0,1,-1, #translation
        0,0,0,  #rotation
        
        10, #this is frame 1 at 10sec
        0,1,-2, #translation
        0,0,0,  #rotation
    ]
    motion1=    \
    [
        0 , #this is frame 0 at 0sec
        1,1,-0.5, #translation
        0,0,0,  #rotation

        10, #this is frame 1 at 10sec
        1,1,-0.5, #translation
        0,0,0,  #rotation
    ]
    motions=[motion0,motion1]
    expert.setup("./python_sheet/sheet.json", motions)
    expert_controller = expert.expert_flat #expert type
    
    hands = random_hands(x=cloth_x)
    print hands
    handles = np.vstack((hands,[[0,cloth_y,0],[cloth_x,cloth_y,0]]))
    print handles
    pos = np.zeros((2,3))    
    for i in range(nr_frame):
        # handles
        handles_prev = handles
        handles = expert.apply_expert(handles, pos.reshape(2,3), delta)
        expert.advance()
        expert.set_handle(handles)
        # save images
        expert.save_frame(dataset_path+"/%04i"%(i), render_image, render_depth)
        expert_pos = np.array(expert_controller(handles,cloth_x,cloth_y))
        # apply the robot controllers positions
        pos = expert_pos        
        
