import paho.mqtt.client as mqtt
import json
import threading
import time
from multi_slam import MultiAgentSLAM

class Server:
    def __init__(self, broker_ip, broker_port):
        self.broker_ip = broker_ip
        self.broker_port = broker_port
        self.client = mqtt.Client()

        self.slam = MultiAgentSLAM(20, 30)

        self.client.on_connect = self._mqtt_on_connect

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        pass

    def _mqtt_on_message(self, client, userdata, message):
        topic_parts = message.topic.split('/')
        if len(topic_parts) != 3: return
        robot_id = topic_parts[1]
        if topic_parts[2] == "request":
            response_data = None
            client.publish(f"robot/{robot_id}/response", json.dumps(response_data))
        elif topic_parts[2] == 'lidar':
            lidar_data = json.loads(message.payload.decode())
            self.slam.update_robot(robot_id, lidar_data['distances'], lidar_data['angles'], lidar_data['step_pose_delta'])

    def _mqtt_on_disconnect(self, client, userdata, rc):
        if rc != 0:
            while True:
                try:
                    client.reconnect()
                    break
                except:
                    time.sleep(1)
                
    def register_robot(self, robot_id):
        self.client.subscribe(f"robot/{robot_id}/request")
        self.client.subscribe(f"robot/{robot_id}/lidar")
        self.slam.register_robot(robot_id)

    def send_command(self, robot_id, command):
        self.client.publish(f"robot/{robot_id}/command", json.dumps(command))

    def start(self):
        self.client.connect(self.broker_ip, self.broker_port)
        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()

    def command_move_robot(self, robot_id, type, val):
        self.client.publish(f"robot/{robot_id}/move", json.dumps({
            "type": type,
            "val": val
        }))
