#!/usr/bin/env python3

import os
import sys
import time
import math
import psutil
import curses
import re
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from laser_msgs.msg import UavControlDiagnostics, ApiPx4Diagnostics, PoseWithHeading
from std_srvs.srv import Trigger
from rosidl_runtime_py.utilities import get_message

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class TopicMonitor:
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

class UavData:
    def __init__(self, node, name, config_topics):
        self.node = node
        self.name = name
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self.ctrl_diag = None
        self.api_diag = None
        self.vins_feat = None
        self.had_goal = False
        self.autostart_active = False
        self.odom_monitor = TopicMonitor(node, f'/{name}/estimation_manager/estimation', Odometry)
        node.create_subscription(Odometry, f'/{name}/estimation_manager/estimation', self.odom_cb, 10)
        node.create_subscription(UavControlDiagnostics, f'/{name}/control_manager/diagnostics', self.ctrl_cb, 10)
        node.create_subscription(ApiPx4Diagnostics, f'/{name}/px4_api/diagnostics', self.api_cb, 10)
        node.create_subscription(Float64, f'/{name}/ov_msckf/num_points_slam', self.vins_feat_cb, 10)
        self.arm_cl = node.create_client(Trigger, f'/{name}/px4_api/arm')
        self.disarm_cl = node.create_client(Trigger, f'/{name}/px4_api/disarm')
        self.takeoff_cl = node.create_client(Trigger, f'/{name}/control_manager/takeoff')
        self.land_cl = node.create_client(Trigger, f'/{name}/control_manager/land')
        self.goto_pub = node.create_publisher(PoseWithHeading, f'/{name}/control_manager/goto', 10)
        self.goto_relative_pub = node.create_publisher(PoseWithHeading, f'/{name}/control_manager/goto_relative', 10)
        self.target_topics = config_topics
        self.extra_monitors = {} 

    def check_and_subscribe_extras(self, available_topics_map):
        for topic in self.target_topics:
            if topic in self.extra_monitors:
                continue
            if self.name not in topic:
                continue
            if topic in available_topics_map:
                try:
                    type_str = available_topics_map[topic][0]
                    msg_class = get_message(type_str)
                    self.extra_monitors[topic] = TopicMonitor(self.node, topic, msg_class)
                except Exception:
                    pass

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pos['x'], self.pos['y'], self.pos['z'] = p.x, p.y, p.z
        self.pos['roll'], self.pos['pitch'], self.pos['yaw'] = quaternion_to_euler(o.x, o.y, o.z, o.w)

    def ctrl_cb(self, msg): 
        self.ctrl_diag = msg
        if msg.have_goal:
            self.had_goal = True

    def api_cb(self, msg): 
        self.api_diag = msg

    def vins_feat_cb(self, msg): 
        self.vins_feat = msg

class LaserUavTUI(Node):
    def __init__(self, stdscr):
        super().__init__('laser_uav_tui')
        self.stdscr = stdscr
        self.stdscr.nodelay(True)
        self.stdscr.keypad(True)
        self.uavs = {}
        self.selected_idx = 0
        self.in_menu = False
        self.in_goto_menu = False
        self.menu_idx = 0
        self.goto_idx = 0
        self.menu_options = ["Arm", "Takeoff", "Land", "Disarm", "GoTo"]
        self.goto_options = ["Mode", "X", "Y", "Z", "Heading", "SEND"]
        self.goto_vals = ["World", 0.0, 0.0, 1.5, 0.0]
        self.sys_info = {'cpu': 0.0, 'ram_percent': 0.0, 'ram_used': 0.0}
        self.blink_state = True
        self.micro_agent_active = False 
        self.config_topics = []
        self.load_config_from_args()

        self.create_timer(2.0, self.discover_uavs)
        self.create_timer(0.05, self.update_loop)
        self.create_timer(1.0, self.update_system_stats)
        self.create_timer(0.5, self.toggle_blink)

    def load_config_from_args(self):
        if '--config' in sys.argv:
            try:
                idx = sys.argv.index('--config')
                if idx + 1 < len(sys.argv):
                    config_path = sys.argv[idx + 1]
                    if os.path.exists(config_path):
                        with open(config_path, 'r') as f:
                            data = yaml.safe_load(f)
                            if data and 'topics' in data:
                                self.config_topics = data['topics']
            except Exception:
                pass

    def toggle_blink(self):
        self.blink_state = not self.blink_state

    def discover_uavs(self):
        names_types = self.get_topic_names_and_types()
        topics_map = {name: types for name, types in names_types}
        for name, _ in names_types:
            match = re.match(r'/(uav\d+)/estimation_manager/estimation', name)
            if match:
                uav_name = match.group(1)
                if uav_name not in self.uavs:
                    if len(self.uavs) < 3:
                        self.uavs[uav_name] = UavData(self, uav_name, self.config_topics)
        
        for uav in self.uavs.values():
            uav.check_and_subscribe_extras(topics_map)

    def update_system_stats(self):
        self.sys_info['cpu'] = psutil.cpu_percent()
        mem = psutil.virtual_memory()
        self.sys_info['ram_percent'] = mem.percent
        self.sys_info['ram_used'] = (mem.total - mem.available) / (1024 ** 3)

        agent_running = False
        for p in psutil.process_iter(['name', 'cmdline']):
            try:
                name = p.info.get('name', '')
                cmd = p.info.get('cmdline', [])
                if (name and 'MicroXRCEAgent' in name) or (cmd and any('MicroXRCEAgent' in c for c in cmd)):
                    agent_running = True
                    break
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        self.micro_agent_active = agent_running

        active_nodes = self.get_node_names_and_namespaces()
        for name, uav in self.uavs.items():
            uav.odom_monitor.update_hz()
            for mon in uav.extra_monitors.values():
                mon.update_hz()
            target_ns = f"/{name.lower().strip('/')}"
            found = False
            for node_name, node_ns in active_nodes:
                if node_name == 'autostart' and node_ns == target_ns:
                    found = True
                    break
            uav.autostart_active = found

    def handle_input(self, key):
        if key == curses.KEY_RESIZE:
            curses.update_lines_cols()
            return
        u_list = sorted(self.uavs.keys())
        if not u_list: return

        if self.in_goto_menu:
            if key == curses.KEY_UP: self.goto_idx = (self.goto_idx - 1) % len(self.goto_options)
            elif key == curses.KEY_DOWN: self.goto_idx = (self.goto_idx + 1) % len(self.goto_options)
            elif key == 10:
                if self.goto_idx == 0:
                    self.goto_vals[0] = "Relative" if self.goto_vals[0] == "World" else "World"
                elif self.goto_idx == 5:
                    self.publish_goto(u_list[self.selected_idx])
                else:
                    self.edit_goto_field()
            elif key == 27: self.in_goto_menu = False
        elif self.in_menu:
            if key == curses.KEY_UP: self.menu_idx = (self.menu_idx - 1) % len(self.menu_options)
            elif key == curses.KEY_DOWN: self.menu_idx = (self.menu_idx + 1) % len(self.menu_options)
            elif key == 10: self.execute_action(u_list[self.selected_idx])
            elif key == 27: self.in_menu = False
        else:
            if key == curses.KEY_UP: self.selected_idx = (self.selected_idx - 1) % len(u_list)
            elif key == curses.KEY_DOWN: self.selected_idx = (self.selected_idx + 1) % len(u_list)
            elif key in [ord('m'), ord('M')]: self.in_menu = True

    def edit_goto_field(self):
        h, w = self.stdscr.getmaxyx()
        box_w_screen = (w - 4) // 3
        sys_center_x = 2 + box_w_screen + (box_w_screen // 2)
        menu_w = 18
        sx = max(2, sys_center_x - (menu_w // 2))
        sy = 2 + (self.selected_idx * 10)
        gx, gy = sx + 25, sy + 2
        fy, fx = gy + 1 + self.goto_idx, gx + 2
        curses.echo()
        curses.curs_set(1)
        self.stdscr.attron(curses.color_pair(3) | curses.A_BOLD)
        new_val_str = ""
        while True:
            self.draw_goto_menu(gy, gx) 
            self.stdscr.move(fy, fx)
            self.stdscr.addstr(" " * 17)
            self.stdscr.addstr(fy, fx + (17 - len(new_val_str))//2, new_val_str)
            self.stdscr.refresh()
            ch = self.stdscr.getch()
            if ch == 10: break
            elif ch == 27: 
                curses.noecho(); curses.curs_set(0); return
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                if len(new_val_str) > 0: new_val_str = new_val_str[:-1]
            elif 32 <= ch <= 126:
                if len(new_val_str) < 10: new_val_str += chr(ch)
        try:
            if new_val_str: self.goto_vals[self.goto_idx] = float(new_val_str)
        except: pass
        self.stdscr.attroff(curses.color_pair(3) | curses.A_BOLD)
        curses.noecho(); curses.curs_set(0)

    def publish_goto(self, name):
        u = self.uavs[name]
        msg = PoseWithHeading()
        msg.position.x = float(self.goto_vals[1])
        msg.position.y = float(self.goto_vals[2])
        msg.position.z = float(self.goto_vals[3])
        msg.heading = float(self.goto_vals[4])
        
        if self.goto_vals[0] == "World":
            u.goto_pub.publish(msg)
        else:
            u.goto_relative_pub.publish(msg)
            
        self.in_goto_menu = False
        self.in_menu = False

    def execute_action(self, name):
        u = self.uavs[name]
        if self.menu_idx == 0: self.call_srv(u.arm_cl); self.in_menu = False
        elif self.menu_idx == 1: self.call_srv(u.takeoff_cl); self.in_menu = False
        elif self.menu_idx == 2: self.call_srv(u.land_cl); self.in_menu = False
        elif self.menu_idx == 3: self.call_srv(u.disarm_cl); self.in_menu = False
        elif self.menu_idx == 4: self.in_goto_menu = True; self.goto_idx = 0

    def call_srv(self, cl):
        if cl.service_is_ready(): cl.call_async(Trigger.Request())

    def draw_watermark(self):
        h, w = self.stdscr.getmaxyx()
        art = ["  _        _     ____  _____ ____  ",
               " | |      / \   / ___|| ____|  _ \ ",
               " | |     / _ \  \___ \|  _| | |_) |",
               " | |___ / ___ \  ___) | |___|  _ < ",
               " |_____/_/   \_\|____/|_____|_| \_\\"]
        self.stdscr.attron(curses.color_pair(2) | curses.A_DIM | curses.A_BOLD)
        for i, line in enumerate(art):
            try: self.stdscr.addstr(h - 7 + i, w - len(line) - 2, line)
            except: pass
        self.stdscr.attroff(curses.color_pair(2) | curses.A_DIM | curses.A_BOLD)

    def draw_box(self, y, x, h, w, title, content, col_b, col_t):
        max_y, max_x = self.stdscr.getmaxyx()
        if y + h > max_y or x + w > max_x: return
        try:
            for i in range(h): self.stdscr.addstr(y + i, x, " " * w)
            self.stdscr.attron(col_b | curses.A_BOLD)
            self.stdscr.addstr(y, x, '╔' + '═'*(w-2) + '╗')
            for i in range(1, h-1):
                self.stdscr.addstr(y+i, x, '║'); self.stdscr.addstr(y+i, x+w-1, '║')
            self.stdscr.addstr(y+h-1, x, '╚' + '═'*(w-2) + '╝')
            if title: self.stdscr.addstr(y, x + (w - len(title) - 2)//2, f" {title} ")
            self.stdscr.attroff(col_b | curses.A_BOLD)
            self.stdscr.attron(col_t | curses.A_BOLD)
            for i, line in enumerate(content):
                if i < h-2: self.stdscr.addstr(y+1+i, x+2, line[:w-4])
            self.stdscr.attroff(col_t | curses.A_BOLD)
        except: pass

    def update_loop(self):
        k = self.stdscr.getch()
        if k != -1: self.handle_input(k)
        self.stdscr.erase()
        self.draw_watermark()
        self.draw_screen()
        if self.in_menu: self.draw_menu()
        self.stdscr.refresh()

    def draw_menu(self):
        h, w = self.stdscr.getmaxyx()
        u_list = sorted(self.uavs.keys())
        if not u_list: return
        box_w_screen = (w - 4) // 3
        sys_center_x = 2 + box_w_screen + (box_w_screen // 2)
        menu_w = 18
        current_y_offset = 2
        for i in range(self.selected_idx):
             u_loop = self.uavs[u_list[i]]
             has_extra = len(u_loop.extra_monitors) > 0
             current_y_offset += 10 + (8 if has_extra else 0)

        sy, sx = current_y_offset, max(2, sys_center_x - (menu_w // 2))
        u_name = u_list[self.selected_idx].upper()
        self.draw_box(sy, sx, 9, menu_w, f"{u_name} MENU", [], curses.color_pair(3), curses.color_pair(2))
        for i, opt in enumerate(self.menu_options):
            attr = (curses.A_REVERSE | curses.A_BOLD) if i == self.menu_idx and not self.in_goto_menu else (curses.A_NORMAL | curses.A_BOLD)
            self.stdscr.addstr(sy+2+i, sx+4, f" {opt} ", attr)
        if self.in_goto_menu:
            self.stdscr.attron(curses.color_pair(3) | curses.A_BOLD)
            self.stdscr.addstr(sy+6, sx+menu_w-2, " --> ")
            self.stdscr.attroff(curses.color_pair(3) | curses.A_BOLD)
            self.draw_goto_menu(sy+2, sx+21)

    def draw_goto_menu(self, y, x):
        box_w = 21
        self.draw_box(y, x, 8, box_w, "GOTO", [], curses.color_pair(3), curses.color_pair(2))
        for i, opt in enumerate(self.goto_options):
            attr = (curses.A_REVERSE | curses.A_BOLD) if i == self.goto_idx else (curses.A_NORMAL | curses.A_BOLD)
            if i == 0:
                content = f"{opt}: {self.goto_vals[i]}"
            elif i < 5:
                content = f"{opt}: {self.goto_vals[i]:.2f}"
            else:
                content = opt
            self.stdscr.addstr(y+1+i, x+2, content.center(box_w - 4), attr)

    def draw_screen(self):
        max_y, max_x = self.stdscr.getmaxyx()
        header = f" LASER UAV SYSTEM | CPU: {self.sys_info['cpu']}% | RAM: {self.sys_info['ram_percent']}% "
        try: self.stdscr.addstr(0, max(0, (max_x-len(header))//2), header, curses.color_pair(2) | curses.A_BOLD)
        except: pass
        u_list = sorted(self.uavs.keys())
        box_h = 6
        current_y = 2
        for i, name in enumerate(u_list):
            if current_y + box_h + 1 > max_y: break
            u = self.uavs[name]
            has_extras = len(u.extra_monitors) > 0
            block_height = 9 + ((len(u.extra_monitors.items()) + 2) if has_extras else 0)
            if current_y + block_height > max_y: break

            checks_passed = getattr(u.api_diag, 'preflight_checks_passed', True)
            frame_attr = curses.color_pair(1) | curses.A_BOLD if checks_passed else (curses.color_pair(5) | curses.A_BOLD if self.blink_state else curses.A_NORMAL)
            self.stdscr.attron(frame_attr)
            try:
                self.stdscr.addstr(current_y, 0, '┌' + '─'*(max_x-2) + '┐')
                agent_active = getattr(self, 'micro_agent_active', False)
                if agent_active:
                    self.stdscr.addstr(current_y, 2, " uXRCE: True ")
                else:
                    self.stdscr.addstr(current_y, 2, " uXRCE: False ", curses.color_pair(5) | curses.A_BOLD)
                total_block_h = block_height - 2
                for row in range(1, total_block_h):
                    self.stdscr.addstr(current_y+row, 0, '│'); self.stdscr.addstr(current_y+row, max_x-1, '│')
                self.stdscr.addstr(current_y + total_block_h, 0, '└' + '─'*(max_x-2) + '┘')
                self.stdscr.addstr(current_y, (max_x - len(name) - 2) // 2, f" {name.upper()} ")
                if not u.had_goal:
                    as_status = 'True' if u.autostart_active else 'False'
                    as_txt = f" AutoStart: {as_status} "
                    self.stdscr.addstr(current_y, max_x - len(as_txt) - 4, as_txt)
            except: pass
            self.stdscr.attroff(frame_attr)
            
            border_col = curses.color_pair(3) if i == self.selected_idx else curses.color_pair(2)
            box_w = (max_x - 4) // 3
            self.draw_box(current_y+1, 2, box_h, box_w - 3, f"Odom [{u.odom_monitor.hz:.1f}Hz]", [f"X: {u.pos['x']:3.2f}", f"Y: {u.pos['y']:3.2f}", f"Z: {u.pos['z']:3.2f}", f"Hdg: {u.pos['yaw']:3.2f}"], border_col, curses.color_pair(1))
            
            midtxt = []
            if u.api_diag:
                qty_sat = getattr(u.api_diag, 'qty_satellites', 'N/A')
                rf_jam = getattr(u.api_diag, 'rf_jamming', 'N/A')
                midtxt = [f"Satellites: {qty_sat}", f"RF Jamming: {rf_jam}"]
            else:
                midtxt = ["Satellites: N/A", "RF Jamming: N/A"]
            if u.vins_feat:
                midtxt += [f"OV Features: {u.vins_feat.data:.0f}"]
            self.draw_box(current_y+1, 2 + box_w - 3, box_h, box_w - 2, "", midtxt, border_col, curses.color_pair(4))
            
            dtxt = []
            if u.api_diag: dtxt += [f"Armed: {'YES' if u.api_diag.armed else 'NO'} | Offb: {'YES' if u.api_diag.offboard_mode else 'NO'}"]
            if u.ctrl_diag: dtxt += [f"Fly: {'YES' if u.ctrl_diag.is_fly else 'NO'} | Goal: {'YES' if u.ctrl_diag.have_goal else 'NO'}", f"Speed: {u.ctrl_diag.current_norm_speed:3.2f} m/s"]
            if u.ctrl_diag: dtxt += [f"RMSE: { f'{u.ctrl_diag.metrics.rmse:.2f}' if u.ctrl_diag.metrics.rmse >= 0 else ' '} | STD: { f'{u.ctrl_diag.metrics.std:.2f}' if u.ctrl_diag.metrics.std >= 0 else ' '}"]
            if u.ctrl_diag: self.draw_box(current_y+1, (2 * box_w) - 1 - 2, box_h, max_x - (2 * box_w) - 4 + 3 + 2, f"Control [{u.ctrl_diag.control_iteration_duration_ms:.1f}ms]", dtxt, border_col, curses.color_pair(3))
            
            if has_extras:
                mon_content = []
                for t_name, t_mon in u.extra_monitors.items():
                    display_name = t_name.replace(f"/{name}/", "").strip('/')
                    mon_content.append(f"{display_name:<35} {t_mon.hz:6.1f} Hz")
                self.draw_box(current_y + box_h + 1, 2, len(u.extra_monitors.items()) + 2, max_x - 4, "Topic Monitor", mon_content, border_col, curses.color_pair(2))
            current_y += (block_height - 2) + 2

        try: self.stdscr.addstr(max_y-1, 1, " [UP/DOWN] Select UAV | [M] Menu | Ctrl+C Exit ", curses.color_pair(2) | curses.A_BOLD)
        except: pass

def main():
    rclpy.init(args=sys.argv)
    
    def run(stdscr):
        curses.curs_set(0); curses.start_color(); curses.use_default_colors()
        for i, c in enumerate([curses.COLOR_GREEN, curses.COLOR_WHITE, curses.COLOR_CYAN, curses.COLOR_MAGENTA, curses.COLOR_YELLOW], 1): 
            curses.init_pair(i, c, -1)
        node = LaserUavTUI(stdscr)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    try: 
        curses.wrapper(run)
    except: 
        pass
    finally: 
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
