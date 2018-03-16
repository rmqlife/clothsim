from arcsim_expert_python import *
import sys,time
from util import *

if __name__== "__main__":
    cloth_x = 0.3
    cloth_y = 0.3

    path = sys.argv[1]
    expert = arcsim_expert()
    expert_func = expert.expert_flat
    expert_func = expert.expert_twist
    #expert_func = expert.expert_arc
    
    func_dict = { "flat":expert.expert_flat, "arc":expert.expert_arc,"twist": expert.expert_twist}
    
    

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
    
    for func_name in func_dict:
        expert_func = func_dict[func_name]
        tt_handles = np.load(path)
        tt_expert = np.array([])
        for i in range(tt_handles.shape[0]):#tt_handles.shape[0]):        
            handles = tt_handles[i,:].reshape(4,3).tolist()
            expert_pos = expert_func(handles,cloth_x,cloth_y)
            expert_pos = np.array(expert_pos).reshape(-1,)
            #print("expert", expert_pos)
            tt_expert = stack_vector(mat = tt_expert, vec=expert_pos)
        filename  =  path[:-4]+"-"+func_name+".npy"
        print(filename, tt_expert.shape)
        np.save(path[:-4]+"-"+func_name+".npy", tt_expert)
