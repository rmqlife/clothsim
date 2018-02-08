from merge_dataset import *
from util import *
import matplotlib.pyplot as plt
from wrinkle2 import *

def build_feature(home,filename):
    tt_depth = np.array([])
    pics = get_filelist(home,['png'])
    for i in range(0,len(pics)):
        d = cv2.imread(pics[i],cv2.IMREAD_ANYDEPTH)
        hist = howhist(d)
        #plt.imshow(d)
        #plt.show()
        #print(hist)
        tt_depth = stack_vector(tt_depth,hist)
        print(pics[i],tt_depth.shape)
    np.save(filename,tt_depth)
    print(tt_depth.shape)

if __name__=='__main__':
    import sys
    build_feature(sys.argv[1], sys.argv[2])
