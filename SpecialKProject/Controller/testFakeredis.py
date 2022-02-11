import redis
from controller_enums import Key, Topic
from random import randint
from time import sleep

REDIS_HOST = "localhost"
REDIS_PORT = 6379

val = 0.0

if __name__ == "__main__":
    redis = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

    while True:
        val = val + 0.001
        round(val, 2)
        redis.set(Key.USONIC, str(val) + '-' + str(val) + '-' + str(val))
        redis.publish(Topic.BODY, Key.USONIC)
        sleep(0.01)



