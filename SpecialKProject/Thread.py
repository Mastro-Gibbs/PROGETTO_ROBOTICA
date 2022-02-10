import threading
import time
import inspect
import ctypes
 



class GATTO:
    

    def __init__(self) -> None:
        self.GATTOVAR = 0

    def getGATTOVAR(self):
        return self.GATTOVAR 

    def _async_raise(self, tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    
    def stop_thread(self, thread):
        self._async_raise(thread.ident, SystemExit)
    
    
    def test(self):
        print("THREAD STARTED!")
        while True:
            time.sleep(0.01)
            self.GATTOVAR += 1
 
 
if __name__ == "__main__":
    g = GATTO()
    print("MAIN THREAD: ", g.getGATTOVAR())
    t = threading.Thread(target=g.test)
    t.start()

    try:
        while True:
            time.sleep(1)
            print("MAIN THREAD: ", g.getGATTOVAR())

    except KeyboardInterrupt:
        g.stop_thread(t)




