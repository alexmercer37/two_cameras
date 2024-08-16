#!/usr/bin/python3

import os
import time
import subprocess
import psutil 
def is_process_running(process_name_or_pid):  
    for proc in psutil.process_iter():  
        try:  
            if process_name_or_pid == proc.name() or str(process_name_or_pid) == str(proc.pid):  
                return True  
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):  
            pass  
    return False  
    
def start_camera():  

    try:  
        os.system('./camera')  
        print("camera success")  
    except OSError as e:  
        print("camera error")  
    
    

if __name__ == "__main__":
    os.chdir('/dev')
    os.system('sudo chmod +666 ttyUSB0') 
    os.chdir('/home/nf/Downloads/twocamera/final/workspace') 

    while True:  
        if not is_process_running('camera'):    
            start_camera()  
        time.sleep(5)  
