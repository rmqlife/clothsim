from shutil import copyfile,rmtree
import subprocess
import argparse
import datetime
import time
import os, shutil
import shutil
import sys

class para_dagger:
    def __init__(self,result_dir):
        import time
        path = "./output/"+time.strftime("%y%m%d-%H%M%S")
        if not os.path.exists(path):
            os.mkdir(path)
        
        #os.mkidr("~/clothsim/result/"+time.strftime("%y%m%d-%H%M%S"))
        self.path = path
        self.result_dir = result_dir
        if not os.path.exists(self.result_dir):
            os.mkdir(self.result_dir)
        shutil.copyfile('source/clothsim.py', os.path.join(result_dir,'clothsim.py'))
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
        script.write('#SBATCH --job-name={jobName} --time={time}\n'.format(jobName=jobName, time=24*60))
        script.write('source activate cloth\n')
        script.write('python clothsim.py '+os.path.join(self.result_dir, jobName)+'\n')
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


if __name__=='__main__':
    dagger = para_dagger(sys.argv[1]);
    for i in range(50):
        print(dagger.create_script(jobName=str(i)))
