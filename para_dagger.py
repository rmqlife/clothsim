from shutil import copyfile,rmtree
import subprocess
import argparse
import datetime
import time
import os, shutil
import shutil
class para_dagger:
    def __init__(self):
        import time
        path = "./output/"+time.strftime("%y%m%d-%H%M%S")
        if os.path.exists(path):
            shutil.rmtree(path)
        os.mkdir(path)
        self.path = path
        pass

    def create_script(self,jobName):
        import os
        job_path=os.path.join(self.path, jobName)
        cwd=os.getcwd()
        shutil.copytree(os.path.join(cwd,'source'),job_path)
        os.chdir(job_path)    
        
        script_path = 'script.sh'
        script=open(script_path,'w')
        script.write('#!/bin/bash\n')
        script.write('#SBATCH --job-name='+jobName+'\n')
        script.write('source activate cloth\n')
        script.write('python clothsim.py '+'data'+'\n')
        script.close()
        #execute script
        ret=subprocess.check_output(['sbatch',script_path])
        
        os.chdir(cwd)
        #find jobid
        c=''
        for i in xrange(len(ret)):
            if ret[i].isdigit():
                c=c+ret[i]
        return c
    
    def frame_path(self):
        self.frame_id = self.frame_id + 1
        return self.path+"/%04i"%(self.frame_id-1)

if __name__=='__main__':
    dagger = para_dagger();
    for i in range(5):
        print dagger.create_script('task'+str(i))
