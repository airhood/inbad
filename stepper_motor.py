import RPi.GPIO as GPIO
import time
import threading

class StepperMotor:
    DIR_CLOCKWISE = 0
    DIR_COUNTERCLOCKWISE = 1

    DEFAULT_DELAY = 0.003

    def __init__(self, enable_pin, step_pin, dir_pin):
        self.enable_pin = enable_pin
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        
        self._thread = None
        self._stop_event = threading.Event()
        
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        GPIO.output(self.enable_pin, GPIO.LOW)
        GPIO.output(self.step_pin, GPIO.LOW)
        GPIO.output(self.dir_pin, GPIO.LOW)

    def set_direction(self, dir):
        if dir == self.DIR_CLOCKWISE:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        elif dir == self.DIR_COUNTERCLOCKWISE:
            GPIO.output(self.dir_pin, GPIO.LOW)

    def step_once(self, delay):
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(delay / 2)
        GPIO.output(self.step_pin, GPIO.LOW)
        time.sleep(delay / 2)

    def _step_worker(self, steps, direction, delay, continuous=False):
        self.set_direction(direction)
        step_count = 0
        
        try:
            while not self._stop_event.is_set():
                self.step_once(delay)
                step_count += 1
                
                if not continuous and step_count >= steps:
                    break
                    
        except Exception as e:
            print(f"스테핑 오류: {e}")
        finally:
            GPIO.output(self.step_pin, GPIO.LOW)

    def start_continuous_rotation(self, direction, delay=DEFAULT_DELAY):
        self.stop()
        
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._step_worker,
            args=(0, direction, delay, True)
        )
        self._thread.daemon = True
        self._thread.start()

    def start_step_rotation(self, steps, direction, delay=DEFAULT_DELAY):
        self.stop()
        
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._step_worker,
            args=(steps, direction, delay, False)
        )
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        if self._thread and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join(timeout=1.0)

    def is_running(self):
        return self._thread and self._thread.is_alive()

    def wait_completion(self):
        if self._thread:
            self._thread.join()
