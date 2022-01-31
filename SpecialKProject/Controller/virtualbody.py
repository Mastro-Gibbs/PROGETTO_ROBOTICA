from controller_enums import Key, Topic
import redis


class VirtualBody:
    def __init__(self, redis_instance: redis.Redis):
        self.redis = redis_instance
        self.pubsub = self.redis.pubsub()
        self.pubsub.subscribe(Topic.BODY)

        self.usonic = [-1.0, -1.0, -1.0]  # left, front, right
        self.ired = [False, False, False]  # left, mid, right
        self.orientation = None

    def get_perceptions(self):
        value = None
        key = None
        msg = self.pubsub.get_message()
        try:
            if msg["type"] == 'message':
                key = msg["data"]
                value = self.redis.get(key)

                if key == Key.USONIC:
                    pass
                elif key == Key.IRED:
                    pass
                elif key == Key.POS:
                    pass
        except TypeError:
            pass

        return key

    def get_usonic(self) -> tuple:
        return self.usonic[0], self.usonic[1], self.usonic[2]

    def get_ired(self) -> tuple:
        return self.ired[0], self.ired[1], self.ired[2]

    def get_orientation(self) -> int:
        return self.orientation

    def send_command(self, k, v):
        self.redis.set(k, v)
        self.redis.publish(Topic.CONTROLLER, k)
