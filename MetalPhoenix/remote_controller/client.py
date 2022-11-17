import random
from redis import Redis

from paho.mqtt import client as mqtt_client


class NodeMCU_MQTT(mqtt_client.Client):

    BROKER_IP   = '192.168.4.1'
    BROKER_PORT = 1883
    topic       = "CMD"
    client_id   = f'python-mqtt-{random.randint(0, 100)}'

    itself      = None

    redis       = None

    def __init__(self) -> None:
        super().__init__()

        self.itself = mqtt_client.Client(self.client_id)
        self.redis  = Redis('localhost', 6379, decode_responses=True)

    def NodeMCU_connect(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print(f"Connected to MQTT {self.BROKER_IP}!")
            else:
                print(f"Failed to connect, return code {rc}")
                exit(-1)

        self.itself.on_connect = on_connect

        try:
            self.itself.connect(self.BROKER_IP, self.BROKER_PORT)
        except TimeoutError as te:
            print("Failed to connect, no route to server")
            exit(-1)

    def NodeMCU_subscribe(self):
        assert self.redis is not None

        def on_message(client, userdata, msg):
            #self.redis.publish(msg.topic, msg.payload.decode())
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

        self.itself.subscribe(self.topic)
        self.itself.on_message = on_message

    def attach(self):
        self.itself.loop_forever()

    def detach(self):
        assert self.itself is not None
        self.itself.unsubscribe(self.topic)
        self.itself.disconnect()



if __name__ == '__main__':
    client = NodeMCU_MQTT()
    client.NodeMCU_connect()
    client.NodeMCU_subscribe()

    try:
        client.attach()
    except KeyboardInterrupt as ki:
        client.detach()

