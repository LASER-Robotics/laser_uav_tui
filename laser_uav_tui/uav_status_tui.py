#!/usr/bin/env python3
import os
import time
import math
import psutil
import curses

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from laser_msgs.msg import UavControlDiagnostics, ApiPx4Diagnostics
from std_srvs.srv import Trigger

UAV_NAME = os.getenv('UAV_NAME', 'uav1')

def quaternion_to_euler(x, y, z, w):
    #Converts Quaternion to Euler (Radians).
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class TopicMonitor:
    #Monitors the frequency (Hz) of a topic.
    def __init__(self, node, topic_name, msg_type):
        self.count = 0
        self.last_count = 0
        self.hz = 0.0
        self.last_time = time.time()
        self.sub = node.create_subscription(msg_type, topic_name, self.callback, 10)

    def callback(self, msg):
        self.count += 1

    def update_hz(self):
        now = time.time()
        dt = now - self.last_time
        if dt >= 1.0:
            self.hz = (self.count - self.last_count) / dt
            self.last_count = self.count
            self.last_time = now
        return self.hz

class LaserUavTUI(Node):
    def __init__(self, stdscr):
        super().__init__('uav_status_tui')
        self.stdscr = stdscr

        # Configures to not block execution waiting for a key
        self.stdscr.nodelay(True)
        
        # 1. Odometry
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self.odom_topic = f'/{UAV_NAME}/estimation_manager/estimation'
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.odom_monitor = TopicMonitor(self, self.odom_topic, Odometry)

        # 2. Control
        self.ctrl_diag = None
        self.ctrl_topic = f'/{UAV_NAME}/control_manager/diagnostics'
        self.create_subscription(UavControlDiagnostics, self.ctrl_topic, self.ctrl_diag_callback, 10)

        # 3. Px4
        self.api_diag = None
        self.api_topic = f'/{UAV_NAME}/px4_api/diagnostics'
        self.create_subscription(ApiPx4Diagnostics, self.api_topic, self.api_diag_callback, 10)

        # 4. System
        self.sys_info = {'cpu': 0.0, 'ram_percent': 0.0, 'ram_used': 0.0}

        # Timers
        self.create_timer(0.066, self.draw_screen)     
        self.create_timer(1.0, self.update_system_stats) 

        # Drone command
        self.arm_client = self.create_client(Trigger, f'/{UAV_NAME}/px4_api/arm')
        self.disarm_client = self.create_client(Trigger, f'/{UAV_NAME}/px4_api/disarm')
        self.takeoff_client= self.create_client(Trigger, f'/{UAV_NAME}/control_manager/takeoff')
        self.land_client = self.create_client(Trigger, f'/{UAV_NAME}/control_manager/land')

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pos['x'], self.pos['y'], self.pos['z'] = p.x, p.y, p.z
        self.pos['roll'], self.pos['pitch'], self.pos['yaw'] = quaternion_to_euler(o.x, o.y, o.z, o.w)

    def ctrl_diag_callback(self, msg):
        self.ctrl_diag = msg

    def api_diag_callback(self, msg):
        self.api_diag = msg

    # Services functions
    def call_service_arm(self):
        if not self.arm_client.service_is_ready():
            self.get_logger().error("Arm service unavailable")
            return
        self.get_logger().info("Send Arm...")
        req = Trigger.Request()
        self.arm_client.call_async(req)

    def call_service_disarm(self):
        if not self.disarm_client.service_is_ready():
            self.get_logger().error("Disarm service unavailable")
            return
        self.get_logger().info("Send Disarm..")
        req = Trigger.Request()
        self.disarm_client.call_async(req) 

    def call_service_takeoff(self):
        if not self.takeoff_client.service_is_ready():
            self.get_logger().error("Takeoff service unavailable")
            return
        self.get_logger().info("Send Takeoff...")
        req = Trigger.Request()
        self.takeoff_client.call_async(req)

    def call_service_land(self):
        if not self.land_client.service_is_ready():
            self.get_logger().error("LAND service unavailable")
            return
        self.get_logger().info("Send Land...")
        req = Trigger.Request()
        self.land_client.call_async(req)
    
    # Menu logic
    def open_menu(self):
        self.stdscr.nodelay(False)
        
        options = ["Arm", "Takeoff", "Land", "Disarm", "Back"]
        idx = 0
        
        while True:
            # 1. Draw menu box
            h, w = self.stdscr.getmaxyx()
            box_h, box_w = 12, 40
            start_y, start_x = (h // 2) - (box_h // 2), (w // 2) - (box_w // 2)
            
            # Clears central area or draws bo
            self.draw_box(start_y, start_x, box_h, box_w, "COMMAND MENU", [], curses.color_pair(3), curses.color_pair(2))

            # 2. Draw options
            for i, option in enumerate(options):
                x_pos = start_x + 2
                y_pos = start_y + 3 + i
                if i == idx:
                    self.stdscr.attron(curses.A_REVERSE)
                    self.stdscr.addstr(y_pos, x_pos, f"> {option}")
                    self.stdscr.attroff(curses.A_REVERSE)
                else:
                    self.stdscr.addstr(y_pos, x_pos, f"  {option}")

            self.stdscr.refresh()

            # 3. Capture key
            key = self.stdscr.getch()

            if key == curses.KEY_UP:
                idx = max(0, idx - 1)
            elif key == curses.KEY_DOWN:
                idx = min(len(options) - 1, idx + 1)
            elif key == 10:
                if idx == 0:
                    self.call_service_arm()
                elif idx == 1:
                    self.call_service_takeoff()
                elif idx == 2:
                    self.call_service_land()
                elif idx == 3:
                    self.call_service_disarm()
                break
            elif key == 27 or key == ord('q'): 
                break
        
        self.stdscr.nodelay(True)


    def update_system_stats(self):
        self.sys_info['cpu'] = psutil.cpu_percent()
        mem = psutil.virtual_memory()
        self.sys_info['ram_percent'] = mem.percent
        self.sys_info['ram_used'] = (mem.total - mem.available) / (1024 ** 3)
        self.odom_monitor.update_hz()

    def draw_box(self, y, x, h, w, title, content, color_border, color_text):
        try:
            self.stdscr.attron(color_border)
            self.stdscr.addstr(y, x, '┌' + '─' * (w - 2) + '┐')
            for i in range(1, h - 1):
                self.stdscr.addstr(y + i, x, '│')
                self.stdscr.addstr(y + i, x + w - 1, '│')
            self.stdscr.addstr(y + h - 1, x, '└' + '─' * (w - 2) + '┘')
            self.stdscr.addstr(y, x + 2, f" {title} ", curses.A_BOLD | color_border)
            self.stdscr.attroff(color_border)

            self.stdscr.attron(color_text)
            for i, line in enumerate(content):
                if i < h - 2:
                    self.stdscr.addstr(y + 1 + i, x + 2, line[:w-4])
            self.stdscr.attroff(color_text)
        except curses.error: pass

    def draw_screen(self):
        try:
            key = self.stdscr.getch()
            if key == ord('m') or key == ord('M'):
                self.open_menu()
                return 
        except: pass

        self.stdscr.erase()
        max_y, max_x = self.stdscr.getmaxyx()
        
        # Columns
        c1 = 1 
        c2 = 34 
        c3 = 66

        # Header
        header = f" LASER UAV SYSTEM | {UAV_NAME} "
        try: self.stdscr.addstr(0, max(0, (max_x//2)-(len(header)//2)), header, curses.A_REVERSE)
        except: pass

        # Box 1: Odometry
        odom_txt = [
            f"X: {self.pos['x']:6.2f}",
            f"Y: {self.pos['y']:6.2f}",
            f"Z: {self.pos['z']:6.2f}",
            f"Heading: {self.pos['yaw']:5.3f} rad"
        ]
        self.draw_box(2, c1, 10, 31, f"Est [{self.odom_monitor.hz:.1f} Hz]", odom_txt, curses.color_pair(2), curses.color_pair(1))

        # Box 2: System
        sys_txt = [
            f"CPU: {self.sys_info['cpu']:5.1f}%",
            f"RAM: {self.sys_info['ram_percent']:5.1f}%",
            f"Mem: {self.sys_info['ram_used']:5.2f} GB"
        ]
        self.draw_box(2, c2, 10, 30, "System", sys_txt, curses.color_pair(2), curses.color_pair(4))

        # Box 3: Diagnostics
        diag_txt=[]

        if self.api_diag:
            armed = getattr(self.api_diag, 'armed', False)
            diag_txt.append(f"Armed: {'YES' if armed else 'NO'}")

        if self.ctrl_diag:
            fly = getattr(self.ctrl_diag, 'is_fly', False)
            goal = getattr(self.ctrl_diag, 'have_goal', False)
            spd = getattr(self.ctrl_diag, 'current_norm_speed', 0.0)
            diag_txt.append(f"Flying: {'YES' if fly else 'NO'}")
            diag_txt.append(f"Have_goal: {'YES' if goal else 'NO'}")
            diag_txt.append( f"Speed: {spd:5.2f} m/s")
            
        else:
            diag_txt.append("Control: Waiting...")
        self.draw_box(2, c3, 10, 35, "Control", diag_txt, curses.color_pair(2), curses.color_pair(3))

        try: self.stdscr.addstr(max_y - 1, 1, " [M] Menu | Ctrl+C to exit ", curses.color_pair(2))
        except: pass
        self.stdscr.refresh()

def main(args=None):
    stdscr = curses.initscr()
    curses.noecho(); curses.cbreak(); stdscr.keypad(True); curses.curs_set(0)
    curses.start_color(); curses.use_default_colors()
    
    # Color Pairs: 1=Green, 2=White, 3=Cyan, 4=Magenta
    for i, c in enumerate([curses.COLOR_GREEN, curses.COLOR_WHITE, curses.COLOR_CYAN, curses.COLOR_MAGENTA], 1):
        curses.init_pair(i, c, -1)

    rclpy.init(args=args)
    try:
        rclpy.spin(LaserUavTUI(stdscr))
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        rclpy.shutdown()
        curses.nocbreak(); stdscr.keypad(False); curses.echo(); curses.curs_set(1); curses.endwin()

if __name__ == '__main__':
    main()