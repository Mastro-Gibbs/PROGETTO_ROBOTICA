from controller import Controller
import hashlib
import builtins
from tools.utility import Logger, CFG

c = Controller()

hash_ = None
hash_old = hash_

GOAL_REACHED = False

logger = Logger(class_name="Agent", color="yellow")
logger.set_logfile(CFG.logger_data()["ALOGFILE"])


def run():
    global hash_
    global hash_old
    global GOAL_REACHED

    logger.log("AGENT LAUNCHED", "green", italic=True)

    while not GOAL_REACHED:
        GOAL_REACHED = c.algorithm()

        with builtins.open("../resources/data/config.conf", "rb") as f:
            hash_ = hashlib.md5(f.read()).hexdigest()

        if hash_old != hash_:
            logger.log("Config file changed", "yellow", italic=True)
            hash_old = hash_
            c.update_cfg()


def stop():
    global GOAL_REACHED
    GOAL_REACHED = True

    c.virtual_destructor()
    logger.log("AGENT STOPPED", "yellow", italic=True)
