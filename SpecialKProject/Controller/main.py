import time

from controller import Controller
import redis

REDIS_HOST = "localhost"
REDIS_PORT = 6379
redis = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

controller = Controller(redis=redis)

EXIT = 1

while EXIT:
    try:
        a, b, c = controller.test_polling()
        print("LEFT: {:.2f}".format(a), end=', FRONT: ')
        print("{:.2f}".format(b), end=', RIGHT: ')
        print("{:.2f}".format(c))
        time.sleep(0.1)
    except  TypeError:
        pass
    except KeyboardInterrupt:
        controller.stop_thread()
        EXIT = 0

