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
        key = None
        msg = self.pubsub.get_message()
        try:
            if msg["type"] == 'message':
                key = msg["data"]
                value = self.redis.get(key)

                if key == Key.USONIC:
                    a, b, c = value.split('-')
                    self.usonic[0] = float(a)
                    self.usonic[1] = float(b)
                    self.usonic[2] = float(c)
                elif key == Key.IRED:
                    a, b, c = value.split('-')
                    self.ired[0] = bool(a)
                    self.ired[1] = bool(b)
                    self.ired[2] = bool(c)
                elif key == Key.POS:
                    self.orientation = float(value)
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
