from stepper_motor import StepperMotor
import paho.mqtt.client as mqtt
import time
import json
import threading
from rplidar import RPLidar
from queue import Queue

class Robot:
    MIN_SAMPLES = 200

    STEPS_PER_METER = 4545.45
    STEPS_PER_DEGREE = 1.0

    def __init__(self, robot_id, motor_pin_left, motor_pin_right, lidar_port, broker_ip, broker_port, lidar_sync_interval):
        self.robot_id = robot_id

        l1, l2, l3 = motor_pin_left
        r1, r2, r3 = motor_pin_right
        self.motor_left = StepperMotor(l1, l2, l3)
        self.motor_right = StepperMotor(r1, r2, r3)

        self.lidar_port = lidar_port
        self.lidar = RPLidar(self.lidar_port, baudrate=115200, timeout=5)
        
        self.broker_ip = broker_ip
        self.broker_port = broker_port
        self.client = mqtt.Client()
        self.client.on_connect = self._mqtt_on_connect
        self.client.on_message = self._mqtt_on_message
        self.client.on_disconnect = self._mqtt_on_disconnect
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)

        self.lidar_sync_interval = lidar_sync_interval
        self.lidar_thread = None
        self.lidar_thread_stop_event = threading.Event()
        self.iterator = None

        self.pos = [0, 0]
        self.last_send_pos = [0, 0]
        self.previous_distances = None
        self.previous_angles = None

        self.publish_queue = Queue()
        self.publish_thread = None
        self.publish_thread_stop_event = threading.Event()
        
    def _mqtt_on_connect(self, client, userdata, flags, rc):
        self.client.subscribe(f"robot/{self.robot_id}/response")
        self.client.subscribe(f"robot/{self.robot_id}/move")

    def _mqtt_on_message(self, client, userdata, message):
        if message.topic.endswith("/response"):
            message_data = json.loads(message.payload.decode())
        elif message.topic.endswith("/move"):
            message_data = json.loads(message.payload.decode())
            print(f"message_data: {message_data}")
            move_type = message_data['type']
            val = message_data['val']
            if move_type == "s":
                self.move_straight(val)
            elif move_type == "r":
                self.move_rotate(val)

    def _mqtt_on_disconnect(self, client, userdata, rc):
        if rc != 0:
            while True:
                try:
                    client.reconnect()
                    break
                except:
                    time.sleep(1)

    def _async_publisher(self):
        while not self.publish_thread_stop_event.is_set():
            try:
                topic, payload = self.publish_queue.get(timeout=0.1)
                self.client.publish(topic, payload)
                self.publish_queue.task_done()
            except:
                time.sleep(0.1)
                continue

    def _publish_lidar_data(self):
        tick = 0
        while not self.lidar_thread_stop_event.is_set():
            try:
                items = [item for item in next(self.iterator)]
                
                valid_items = [(item[1], item[2]) for item in items 
                                if 150 <= item[2] <= 8000 and item[0] > 10]
                
                if tick != (self.lidar_sync_interval * 50): continue
                tick = 0
                
                if len(valid_items) < self.MIN_SAMPLES:
                    if self.previous_distances is not None:
                        pass
                            
                step = max(1, len(valid_items) // 300)
                sampled_items = valid_items[::step]
                
                angles = [item[0] for item in sampled_items]
                distances = [item[1] for item in sampled_items]

                step_pose_delta = [self.pos[0] - self.last_send_pos[0],
                                self.pos[1] - self.last_send_pos[1],
                                self.pos[2] - self.last_send_pos[2]]

                lidar_data = {
                    "angles": angles,
                    "distances": distances,
                    "step_pose_delta": step_pose_delta
                }

                self.last_send_pos = self.pos

                topic = f"robot/{self.robot_id}/lidar"
                payload = json.dumps(lidar_data)

                if self.publish_queue.qsize() > 5:
                    try:
                        self.publish_queue.get_nowait()
                        self.publish_queue.task_done()
                    except:
                        pass

                self.client.publish()
                self.publish_queue.put((topic, payload))
            except Exception as e:
                pass
            time.sleep(0.02)

    def send_request(self, type, data):
        self.client.publish(f"robot/{self.robot_id}/request", json.dumps({
            "type": type,
            "data": data
        }))

    def start(self):
        # lidar connect
        self.lidar.start_motor()
        time.sleep(3)
        self.iterator = self.lidar.iter_scans()

        # MQTT publish thread start
        self.publish_thread = threading.Thread(target=self._async_publisher, daemon=True)
        self.publish_thread.start()

        # start lidar thread
        self.lidar_thread = threading.Thread(target=self._publish_lidar_data, daemon=True)
        self.lidar_thread.start()

        # MQTT connect
        self.client.connect(self.broker_ip, self.broker_port)
        self.client.loop_start()

    def stop(self):
        # MQTT publish thread stop
        self.publish_thread_stop_event.set()
        if self.publish_thread and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)
            
        # lidar thread stop
        self.lidar_thread_stop_event.set()
        if self.lidar_thread.is_alive():
            self.lidar_thread.join(timeout=1.0)

        # MQTT disconnect
        self.client.loop_stop()
        self.client.disconnect()

        # lidar disconnect
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def move_straight(self, distance):
        left_step = abs(distance) * self.STEPS_PER_DEGREE
        right_step = abs(distance) * self.STEPS_PER_DEGREE

        if distance > 0:
            self.motor_left.start_step_rotation(left_step, StepperMotor.DIR_COUNTERCLOCKWISE)
            self.motor_right.start_step_rotation(right_step, StepperMotor.DIR_CLOCKWISE)
        elif distance < 0:
            self.motor_left.start_step_rotation(left_step, StepperMotor.DIR_CLOCKWISE)
            self.motor_right.start_step_rotation(right_step, StepperMotor.DIR_COUNTERCLOCKWISE)

    def move_rotate(self, rotation):
        left_step = abs(rotation) * self.STEPS_PER_METER
        right_step = abs(rotation) * self.STEPS_PER_METER

        if rotation > 0:
            self.motor_left.start_step_rotation(left_step, StepperMotor.DIR_COUNTERCLOCKWISE)
            self.motor_right.start_step_rotation(right_step, StepperMotor.DIR_COUNTERCLOCKWISE)
        elif rotation < 0:
            self.motor_left.start_step_rotation(left_step, StepperMotor.DIR_CLOCKWISE)
            self.motor_right.start_step_rotation(right_step, StepperMotor.DIR_CLOCKWISE)
    
    def move_to(self, target_pos):
        pass
