from threading import Thread
import inspect
import ctypes


class RobotThread(Thread):
    
    def __tid(self):
        if self.is_alive():
            return ctypes.c_long(self.ident)

        print(f'Thread {self.name} is not alive')
        return None

    def bury(self):
        tid = self.__tid()
        if tid != None:
            exctype = SystemExit
            
            if not inspect.isclass(exctype):
                exctype = type(exctype)  

            res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))

            if res == 1:
                print(f'Thread {self.name} burried')
            elif res == 0:
                raise ValueError("invalid thread id")
            elif res != 1:
                ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
                raise SystemError("PyThreadState_SetAsyncExc failed")
            
            