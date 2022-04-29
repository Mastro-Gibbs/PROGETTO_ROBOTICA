from controller import Controller
import hashlib

c = Controller()

hash_ = None
hash_old = hash_


def run():
    global hash_
    global hash_old

    while not c.goal_reached():
        c.algorithm()

        with open("../resources/data/config.conf", "rb") as f:
            hash_ = hashlib.md5(f.read()).hexdigest()

        if hash_old != hash_:
            hash_old = hash_
            c.update_cfg()



