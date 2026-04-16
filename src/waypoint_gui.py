#!/usr/bin/env python3

import yaml
import os
import math
import random
import functools
import threading
import itertools
import tkinter as tk
from tkinter import ttk, messagebox

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# --- ROS 2 Imports ---
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped

# Sử dụng NavigateToPose (Point-to-Point)
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ================= ROS 2 NODE ACTION CLIENT =================
class Nav2WaypointClient(Node):
    def __init__(self, gui_app=None):
        super().__init__('waypoint_gui_client')
        self.gui_app = gui_app
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.path_history = []  
        
        self.waypoint_queue = []
        self.current_wp_index = 0
        self.is_navigating = False
        
        # Tốc độ kiểm tra vị trí 5Hz
        self.create_timer(0.2, self.update_robot_pose)

    def update_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            self.current_x = trans.transform.translation.x
            self.current_y = trans.transform.translation.y
            
            # --- Lưu vết đường đi ---
            if not self.path_history:
                self.path_history.append({'x': self.current_x, 'y': self.current_y})
            else:
                last_p = self.path_history[-1]
                dist = math.hypot(self.current_x - last_p['x'], self.current_y - last_p['y'])
                if dist > 0.05:
                    self.path_history.append({'x': self.current_x, 'y': self.current_y})
                    if len(self.path_history) > 1000:
                        self.path_history.pop(0)

            # --- KIỂM TRA BÁN KÍNH MỤC TIÊU (r = 0.2m) ---
            if self.is_navigating and self.current_wp_index < len(self.waypoint_queue):
                target_wp = self.waypoint_queue[self.current_wp_index]
                target_x = float(target_wp['position']['x'])
                target_y = float(target_wp['position']['y'])
                
                # Tính khoảng cách Euclidean từ vị trí hiện tại đến target
                dist_to_target = math.hypot(self.current_x - target_x, self.current_y - target_y)
                
                if dist_to_target <= 0.2:
                    self.get_logger().info(f"Đã lướt qua mục tiêu {self.current_wp_index + 1} (cách {dist_to_target:.2f}m). Gửi điểm tiếp theo!")
                    self.current_wp_index += 1
                    self.send_next_point()
                    
        except Exception:
            pass

    def start_point_by_point(self, waypoints_data):
        self.get_logger().info('Bắt đầu quy trình di chuyển Point-to-Point...')
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            return False, "Không tìm thấy NavigateToPose Action Server!"
            
        self.waypoint_queue = waypoints_data
        self.current_wp_index = 0
        self.is_navigating = True
        self.send_next_point()
        return True, "Robot bắt đầu di chuyển..."

    def send_next_point(self):
        if not self.is_navigating: return
            
        if self.current_wp_index >= len(self.waypoint_queue):
            self.get_logger().info("🎉 ĐÃ HOÀN THÀNH TẤT CẢ CÁC ĐIỂM!")
            self.is_navigating = False
            if self.gui_app:
                self.gui_app.update_status("Trạng thái: Đã hoàn thành mọi mục tiêu!", "green")
            return

        wp = self.waypoint_queue[self.current_wp_index]
        if self.gui_app:
            self.gui_app.update_status(f"Trạng thái: Đang tiến tới Target {self.current_wp_index + 1}", "blue")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['position']['x'])
        goal_msg.pose.pose.position.y = float(wp['position']['y'])
        
        if 'orientation' in wp:
            goal_msg.pose.pose.orientation.x = float(wp['orientation']['x'])
            goal_msg.pose.pose.orientation.y = float(wp['orientation']['y'])
            goal_msg.pose.pose.orientation.z = float(wp['orientation']['z'])
            goal_msg.pose.pose.orientation.w = float(wp['orientation']['w'])
        else:
            goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(functools.partial(self.goal_response_callback, wp_index=self.current_wp_index))

    def goal_response_callback(self, future, wp_index):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.is_navigating and self.current_wp_index == wp_index:
                self.get_logger().warn("Mục tiêu bị từ chối! Thử lại sau 4 giây...")
                threading.Timer(4.0, self._retry_current_point).start()
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(functools.partial(self.get_result_callback, wp_index=wp_index))

    def get_result_callback(self, future, wp_index):
        status = future.result().status
        
        # Nếu đã chuyển sang mục tiêu khác (nhờ bán kính 0.2m), bỏ qua callback cũ
        if not self.is_navigating or self.current_wp_index != wp_index: return

        if status == 4: # SUCCEEDED (Trong trường hợp robot đến nơi trước khi update_robot_pose kịp bắt)
            self.get_logger().info(f"Hoàn thành mục tiêu {self.current_wp_index + 1}!")
            self.current_wp_index += 1
            self.send_next_point()
        else:
            self.get_logger().error(f"Thất bại tại điểm {self.current_wp_index + 1} (Status: {status}). Bắt buộc thử lại!")
            if self.gui_app:
                self.gui_app.update_status(f"Lỗi tại Target {self.current_wp_index + 1}! Đang tìm cách thử lại...", "red")
            threading.Timer(3.0, self._retry_current_point).start()
    def _retry_current_point(self):
        if self.is_navigating: self.send_next_point()

# ================= GIAO DIỆN TKINTER & THUẬT TOÁN =================
class WaypointGUI:
    def __init__(self, master, file_path):
        self.master = master
        self.master.title("Nav2 Map Planner - Optimal Routing (Fast)")
        self.master.geometry("1300x800") 
        self.master.minsize(900, 600)
        
        self.file_path = file_path
        self.ros_node = None 
        
        self.all_waypoints = []      
        self.ga_waypoints = []       
        self.temp_point = None       
        self.planned_lines = [] 
        
        self.map_img = None
        self.map_photo = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0, 0.0]
        self.map_orig_w = 0
        self.map_orig_h = 0
        
        self.last_scale = 0.0 
        self.scale = 30.0
        self.origin_x = 0; self.origin_y = 0
        self.pan_x = 0; self.pan_y = 0
        self._drag_data = {"x": 0, "y": 0, "moved": False}
        
        self.setup_ui()
        self.load_waypoints()
        
        map_yaml_path = '/home/ubuntu24-04/ros2_ws/src/robot_omni/map/my_map.yaml'
        self.load_map_from_yaml(map_yaml_path)
        self.update_realtime_canvas()

    def set_ros_node(self, node):
        self.ros_node = node
        self.ros_node.gui_app = self

    def load_map_from_yaml(self, yaml_path):
        if not HAS_PIL or not os.path.exists(yaml_path): return
        try:
            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
            self.map_resolution = float(map_data.get('resolution', 0.05))
            self.map_origin = map_data.get('origin', [0.0, 0.0, 0.0])
            img_file = map_data.get('image', 'my_map.pgm')
            img_path = img_file if os.path.isabs(img_file) else os.path.join(os.path.dirname(yaml_path), img_file)
            if not os.path.exists(img_path): return
                
            raw_img = Image.open(img_path)
            self.map_orig_w = raw_img.width
            self.map_orig_h = raw_img.height
            self.map_img = raw_img.transpose(Image.ROTATE_90)
            
            self.lbl_map_status.config(text=f"Map: Đã tải ({self.map_orig_w}x{self.map_orig_h})", fg="green")
            self.last_scale = 0.0
        except Exception as e:
            pass

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam') 

        main_pane = ttk.PanedWindow(self.master, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_frame = ttk.Frame(main_pane, width=400)
        main_pane.add(left_frame, weight=0) 

        visual_frame = ttk.LabelFrame(left_frame, text=" 👁️ VISUAL & BẢN ĐỒ ")
        visual_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))

        btn_frame1 = ttk.Frame(visual_frame)
        btn_frame1.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(btn_frame1, text="🔍 Zoom (+)", command=self.zoom_in).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(btn_frame1, text="🔍 Zoom (-)", command=self.zoom_out).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(btn_frame1, text="🔄 Reset Map", command=self.reset_map).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        self.lbl_map_status = tk.Label(visual_frame, text="Map: Đang tải...", font=("Arial", 9, "bold"))
        self.lbl_map_status.pack(anchor=tk.W, padx=5)

        self.lb_all = tk.Listbox(visual_frame, font=("Consolas", 10), selectbackground="#4a6984", height=7)
        self.lb_all.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.lb_all.bind('<<ListboxSelect>>', self.on_listbox_all_select)

        ga_frame = ttk.LabelFrame(left_frame, text=" 🎯 TỐI ƯU LỘ TRÌNH & NAV2 ")
        ga_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        btn_frame2 = ttk.Frame(ga_frame)
        btn_frame2.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(btn_frame2, text="➕ Add", command=self.add_to_ga).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(btn_frame2, text="➖ Delete", command=self.delete_from_ga).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(btn_frame2, text="❌ Clear All", command=self.clear_all_ga).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        ttk.Button(ga_frame, text="🚀 GENERATE (Sắp Xếp Thứ Tự Nhanh)", command=self.start_optimal_thread).pack(fill=tk.X, padx=5, pady=5)
        
        self.lbl_nav_status = tk.Label(ga_frame, text="Trạng thái: Đang chờ lệnh...", font=("Arial", 10, "bold"), fg="black")
        self.lbl_nav_status.pack(anchor=tk.W, padx=5, pady=2)

        btn_send = tk.Button(ga_frame, text="▶️ BẮT ĐẦU CHẠY POINT-TO-POINT", bg="#32cd32", fg="white", font=("Arial", 10, "bold"), command=self.send_route_to_nav2)
        btn_send.pack(fill=tk.X, padx=5, pady=(0, 5))

        btn_stop = tk.Button(ga_frame, text="⏹️ DỪNG LẠI", bg="#dc143c", fg="white", font=("Arial", 10, "bold"), command=self.stop_nav2)
        btn_stop.pack(fill=tk.X, padx=5, pady=(0, 5))

        self.lb_ga = tk.Listbox(ga_frame, font=("Consolas", 10), selectbackground="#228b22", height=7)
        self.lb_ga.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)

        right_frame = ttk.Frame(main_pane)
        main_pane.add(right_frame, weight=1)

        self.canvas = tk.Canvas(right_frame, bg="#e0e0e0", highlightthickness=1, highlightbackground="#cccccc", cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self.on_canvas_resize)
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_press)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_release)

    def update_status(self, text, color):
        self.lbl_nav_status.config(text=text, fg=color)

    # --- CANVAS CONTROLS ---
    def zoom_in(self): self.scale += 5; self.redraw()
    def zoom_out(self): 
        if self.scale > 5: self.scale -= 5; self.redraw()
    def reset_map(self): self.scale = 30.0; self.pan_x = 0; self.pan_y = 0; self.redraw()
    def on_canvas_resize(self, event): self.origin_x = event.width / 2; self.origin_y = event.height / 2; self.redraw()
    def on_mouse_press(self, event): self._drag_data["x"] = event.x; self._drag_data["y"] = event.y; self._drag_data["moved"] = False
    def on_mouse_drag(self, event):
        self.pan_x += event.x - self._drag_data["x"]; self.pan_y += event.y - self._drag_data["y"]
        self._drag_data["x"] = event.x; self._drag_data["y"] = event.y; self._drag_data["moved"] = True
        self.redraw()
    def on_mouse_release(self, event):
        if not self._drag_data["moved"]:
            self.lb_all.selection_clear(0, tk.END)
            rx, ry = self.canvas_to_ros(event.x, event.y)
            self.temp_point = {'x': rx, 'y': ry}
            self.redraw()
    def on_listbox_all_select(self, event): self.temp_point = None; self.redraw()

    def get_canvas_coords(self, x, y):
        cx = self.origin_x + self.pan_x + (-y * self.scale)
        cy = self.origin_y + self.pan_y - (x * self.scale) 
        return cx, cy
    def canvas_to_ros(self, cx, cy):
        rot_x = (cx - self.origin_x - self.pan_x) / self.scale
        rot_y = (cy - self.origin_y - self.pan_y) / -self.scale
        return rot_y, -rot_x

    def load_waypoints(self):
        target_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'robot_omni', 'saved_waypoints.yaml')
        if not os.path.exists(target_path): target_path = self.file_path 
        self.all_waypoints.clear()
        self.lb_all.delete(0, tk.END)
        if os.path.exists(target_path):
            with open(target_path, 'r') as f: data = yaml.safe_load(f)
            if data and 'waypoints' in data:
                self.all_waypoints = data['waypoints']
                for i, wp in enumerate(self.all_waypoints):
                    self.lb_all.insert(tk.END, f" P{i+1:<2} | X:{wp['position']['x']:>5.2f} Y:{wp['position']['y']:>5.2f}")
        self.redraw()

    def add_to_ga(self):
        new_wp = None
        if self.temp_point:
            new_wp = {'position': {'x': float(self.temp_point['x']), 'y': float(self.temp_point['y'])}, 'source': 'Map', 'type': 'target'}
            self.temp_point = None
        elif self.lb_all.curselection():
            idx = self.lb_all.curselection()[0]
            wp = self.all_waypoints[idx]
            new_wp = {'position': {'x': wp['position']['x'], 'y': wp['position']['y']}, 'source': f'P{idx+1}', 'type': 'target'}
            if 'orientation' in wp: new_wp['orientation'] = wp['orientation']
            self.lb_all.selection_clear(0, tk.END)
        else: return
        self.ga_waypoints.append(new_wp)
        self.planned_lines.clear() 
        self.update_ga_listbox()

    def delete_from_ga(self):
        selection = self.lb_ga.curselection()
        if not selection: return
        self.ga_waypoints.pop(selection[0])
        self.planned_lines.clear() 
        self.update_ga_listbox()

    def clear_all_ga(self):
        self.ga_waypoints.clear()
        self.planned_lines.clear()
        self.update_ga_listbox()
        self.redraw()

    def update_ga_listbox(self):
        self.lb_ga.delete(0, tk.END)
        for i, wp in enumerate(self.ga_waypoints):
            self.lb_ga.insert(tk.END, f"⭐ Target {i+1} | X:{wp['position']['x']:>5.2f} Y:{wp['position']['y']:>5.2f}")

    def send_route_to_nav2(self):
        if not self.ga_waypoints: return
        if not self.ros_node: return
        success, msg = self.ros_node.start_point_by_point(self.ga_waypoints)
        if not success: messagebox.showerror("Lỗi Nav2", msg)

    def stop_nav2(self):
        if self.ros_node:
            self.ros_node.is_navigating = False
            self.update_status("Trạng thái: Đã Hủy bắt buộc!", "red")

    # --- TÌM ĐƯỜNG NGẮN NHẤT (ĐƯỜNG THẲNG) BẰNG VÉT CẠN ---
    def start_optimal_thread(self):
        if len(self.ga_waypoints) < 1: return
        self.update_status("Trạng thái: Đang sắp xếp lộ trình...", "orange")
        threading.Thread(target=self.run_optimal_routing, args=(self.ga_waypoints,), daemon=True).start()

    def run_optimal_routing(self, target_wps):
        # Lấy tọa độ robot hiện tại và tất cả các mục tiêu
        points = [{'x': self.ros_node.current_x, 'y': self.ros_node.current_y}] + [wp['position'] for wp in target_wps]
        n = len(points)
        
        # B1: TÍNH TOÁN KHOẢNG CÁCH EUCLIDEAN (Tốc độ ánh sáng ⚡)
        dist_matrix = [[0.0]*n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                dist_matrix[i][j] = math.hypot(points[i]['x'] - points[j]['x'], points[i]['y'] - points[j]['y'])

        # B2: VÉT CẠN TÌM THỨ TỰ TỐI ƯU
        best_route = []
        if n > 2:
            min_dist = float('inf')
            
            # Xét tất cả các trường hợp lộ trình có thể xảy ra
            for perm in itertools.permutations(range(1, n)):
                current_dist = dist_matrix[0][perm[0]]
                for i in range(len(perm) - 1):
                    current_dist += dist_matrix[perm[i]][perm[i+1]]
                
                if current_dist < min_dist:
                    min_dist = current_dist
                    best_route = list(perm)
        else:
            best_route = [1] if n == 2 else []

        # B3: CẬP NHẬT LẠI DANH SÁCH & VẼ ĐƯỜNG THẲNG
        reordered_targets = []
        full_planned_path = [{'x': points[0]['x'], 'y': points[0]['y']}] 
        
        for next_node in best_route:
            # Chỉ lưu 1 đường thẳng nối 2 điểm
            full_planned_path.append({'x': points[next_node]['x'], 'y': points[next_node]['y']})
            
            # Sắp xếp lại thứ tự Listbox
            actual_wp = target_wps[next_node - 1]
            reordered_targets.append(actual_wp)
            
        self.ga_waypoints = reordered_targets 
        self.planned_lines = full_planned_path

        self.master.after(0, self.update_ga_listbox)
        self.master.after(0, lambda: self.update_status("Trạng thái: Tối ưu hoàn tất! Sẵn sàng chạy.", "green"))
        
        def ask_ready():
            if messagebox.askyesno("Nav2 - Sẵn sàng", f"Đã tối ưu xong lộ trình cho {len(self.ga_waypoints)} điểm Target!\n\nBạn có muốn bắt đầu di chuyển Robot ngay không?"):
                self.send_route_to_nav2()
        self.master.after(0, ask_ready)

    def update_realtime_canvas(self):
        self.redraw()
        self.master.after(500, self.update_realtime_canvas)

    def redraw(self):
        self.canvas.delete("all")
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        cx, cy = self.origin_x + self.pan_x, self.origin_y + self.pan_y
        
        # 1. Vẽ bản đồ (Map)
        if self.map_img:
            if self.last_scale != self.scale or self.map_photo is None:
                new_w = int(self.map_orig_h * self.map_resolution * self.scale)
                new_h = int(self.map_orig_w * self.map_resolution * self.scale)
                if new_w > 0 and new_h > 0:
                    resized_img = self.map_img.resize((new_w, new_h), Image.NEAREST)
                    self.map_photo = ImageTk.PhotoImage(resized_img)
                    self.last_scale = self.scale
                    
            if self.map_photo:
                x_c = self.map_origin[0] + (self.map_orig_w * self.map_resolution) / 2.0
                y_c = self.map_origin[1] + (self.map_orig_h * self.map_resolution) / 2.0
                mcx, mcy = self.get_canvas_coords(x_c, y_c)
                self.canvas.create_image(mcx, mcy, image=self.map_photo, anchor=tk.CENTER)

        # 2. Vẽ trục tọa độ trung tâm
        self.canvas.create_line(0, cy, w, cy, fill="#ff9999", dash=(2, 4)) 
        self.canvas.create_line(cx, 0, cx, h, fill="#ff9999", dash=(2, 4)) 
        
        # 3. Vẽ vị trí Robot và vết đường đi thực tế
        if self.ros_node:
            path = self.ros_node.path_history
            if len(path) > 1:
                for i in range(len(path) - 1):
                    x1, y1 = self.get_canvas_coords(path[i]['x'], path[i]['y'])
                    x2, y2 = self.get_canvas_coords(path[i+1]['x'], path[i+1]['y'])
                    self.canvas.create_line(x1, y1, x2, y2, fill="#0000ff", width=2)
                    
            rx, ry = self.get_canvas_coords(self.ros_node.current_x, self.ros_node.current_y)
            self.canvas.create_oval(rx-8, ry-8, rx+8, ry+8, fill="#00FFFF", outline="#0000CD", width=2)
            self.canvas.create_text(rx+45, ry+15, text=f"ROBOT\n(X:{self.ros_node.current_x:.2f}, Y:{self.ros_node.current_y:.2f})", fill="#0000CD", font=("Arial", 8, "bold"))

        # 4. Vẽ Waypoints có sẵn
        for i, wp in enumerate(self.all_waypoints):
            px, py = self.get_canvas_coords(wp['position']['x'], wp['position']['y'])
            self.canvas.create_oval(px-4, py-4, px+4, py+4, fill="#b0e5b0", outline="#228b22")
            self.canvas.create_text(px+12, py-10, text=f"P{i+1}", fill="#228b22", font=("Arial", 8, "bold"))

        # 5. Vẽ điểm tạm thời
        if self.temp_point:
            px, py = self.get_canvas_coords(self.temp_point['x'], self.temp_point['y'])
            self.canvas.create_oval(px-7, py-7, px+7, py+7, fill="yellow", outline="orange", width=2)

        # 6. Vẽ đường thẳng lộ trình tối ưu
        if self.planned_lines:
            for i in range(len(self.planned_lines) - 1):
                x1, y1 = self.get_canvas_coords(self.planned_lines[i]['x'], self.planned_lines[i]['y'])
                x2, y2 = self.get_canvas_coords(self.planned_lines[i+1]['x'], self.planned_lines[i+1]['y'])
                # Dùng một đường nét đứt và có mũi tên để chỉ hướng di chuyển
                self.canvas.create_line(x1, y1, x2, y2, fill="#32cd32", width=3, dash=(5, 5), arrow=tk.LAST) 

        # 7. Vẽ Marker Target đang chạy
        if self.ga_waypoints:
            for i, wp in enumerate(self.ga_waypoints):
                px, py = self.get_canvas_coords(wp['position']['x'], wp['position']['y'])
                self.canvas.create_oval(px-6, py-6, px+6, py+6, fill="#ff4500", outline="#b22222", width=2)
                self.canvas.create_rectangle(px+8, py-18, px+45, py-2, fill="white", outline="white")
                self.canvas.create_text(px+25, py-10, text=f"Target {i+1}", fill="#b22222", font=("Arial", 8, "bold"))

# ================= MAIN =================
def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    ros_node = Nav2WaypointClient()
    
    spin_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    spin_thread.start()

    file_path = 'saved_waypoints.yaml' 
    root = tk.Tk()
    
    window_width = 1300; window_height = 800
    sw = root.winfo_screenwidth(); sh = root.winfo_screenheight()
    root.geometry(f'{window_width}x{window_height}+{int(sw/2 - window_width/2)}+{int(sh/2 - window_height/2)}')
    
    app = WaypointGUI(root, file_path)
    app.set_ros_node(ros_node) 
    
    def on_closing():
        ros_node.destroy_node()
        rclpy.shutdown()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()