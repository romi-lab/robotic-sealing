
import os, signal                                                               
from multiprocessing import Pool                                                
import time                                                         
                                
def run_process(process):                                                             
    os.system('python {}'.format(process))                                       

def init_worker():
    signal.signal(signal.SIGINT, signal.SIG_IGN)

def run_worker():
    time.sleep(15)                                        
                                                                                
def main():
    pool = Pool(processes=4, initializer=init_worker)                                                
    processes = ('./gesture_peace.py', './gesture_ok.py', './gesture_wave.py', './gesture_closed_hand.py')                                    
    pool.map(run_process, processes)    
    try:
        time.sleep(1)
    except KeyboardInterrupt:
        pool.terminate()
        pool.join()


if __name__ == '__main__':
    main()
      
