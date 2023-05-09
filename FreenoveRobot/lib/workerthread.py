from threading import Thread
import inspect
import ctypes


class RobotThread(Thread):
    
    def __tid(self):
        if self.is_alive():
            return ctypes.c_long(self.ident)

        return None

    def bury(self) -> str:
        tid = self.__tid()
        if tid is not None:
            exctype = SystemExit
            
            if not inspect.isclass(exctype):
                exctype = type(exctype)  

            res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))

            if res == 1:
                return f'Thread {self.name} burried'
            elif res == 0:
                raise ValueError("invalid thread id")
            elif res != 1:
                ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
                raise SystemError("PyThreadState_SetAsyncExc failed")

        else:
            return f'Thread {self.name} is not alive'
            