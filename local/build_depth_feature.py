from merge_dataset import *
import cv2

def build_depth_feature(home,filename,target_size = (64,64)):
    tt_depth = np.array([])
    pics = get_filelist(home,['png'])
    for i in range(len(pics)):
        d = cv2.imread(pics[i],cv2.IMREAD_ANYDEPTH)
        d = cv2.resize(d, target_size)
        d = d.reshape(-1,)
        tt_depth = stack_vector(tt_depth,d)
        print(pics[i],tt_depth.shape)
    np.save(filename,tt_depth)
    print(tt_depth.shape)

if __name__=='__main__':
    import sys
    home = sys.argv[1]
    fn = sys.argv[2]
    build_depth_feature(home,fn)
