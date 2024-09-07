from math import floor
from threading import Lock, Thread
from time import sleep
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

BATTERY_HIGH = 0.90
BATTERY_LOW = 0.30
BATTERY_CRITICAL = 0.10

class BatteryMonitor(Node):
    def __init__(self, lock):
        super().__init__('battery_monitor')
        self.lock = lock
        self.battery_state_subscriber = self.create_subscription(
            BatteryState, 
            'battery_state', 
            self.battery_state_callback, 
            qos_profile_sensor_data
        )
        self.battery_percent = None

    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()

def main(args=None):
    rclpy.init(args=args)
    navigator = TurtleBot4Navigator()
    goal_poses = navigator.createPath()
    lock = Lock()
    battery_monitor = BatteryMonitor(lock)
    battery_percent = None
    forward = True  

    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    if not navigator.getDockedStatus():
        navigator.info('Acoplando antes de inicializar la pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    navigator.undock()

    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if battery_percent is not None:
            navigator.info(f'La batería está al {(battery_percent * 100):.2f}% de carga')

            if battery_percent < BATTERY_CRITICAL:
                navigator.error('Batería críticamente baja. Cargar o apagar.')
                break
            elif battery_percent < BATTERY_LOW:
                navigator.info('Acoplando para cargar...')
                navigator.startToPose(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('El robot no logró acoplarse')
                    break
                navigator.info('Cargando...')
                battery_percent_prev = 0 
                while battery_percent < BATTERY_HIGH:
                    sleep(15)
                    battery_percent_prev = floor(battery_percent * 100) / 100
                    with lock:
                        battery_percent = battery_monitor.battery_percent
                    
                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(f'La batería está al {(battery_percent * 100):.2f}% de carga')

                navigator.undock()
            else:
                if forward:
                    navigator.startToPose(goal_poses[0])
                    navigator.startToPose(goal_poses[-1])
                else:
                    navigator.startToPose(goal_poses[-1])
                    navigator.startToPose(goal_poses[0])
                forward = not forward  

                
                sleep(2)

    battery_monitor.destroy_node()
    rclpy.shutdown()
