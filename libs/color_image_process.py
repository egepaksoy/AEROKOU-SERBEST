#! GIBI
import threading
import cv2
import socket
import numpy as np
import struct
import time

class Handler:
    def __init__(self, stop_event, window_name: str="UDP Goruntusu", middle_range: int=0.4):
        self.model = None
        self.proccessing = False

        self.stop_event = stop_event
        self.running = True

        self.screen_res = None
        
        self.showing_image = True
        self.show_crosshair = True
        self.crosshair_color = (0, 0, 255)

        self.middle_range = middle_range

        self.broadcast_started = False

        self.window_name = window_name

        self.ters = False

        self.camera_width = 640
        self.camera_height = 480

        self.show_box = False
        self.orta_box_color = (0, 0, 0)
        self.oran = None
    
    def get_distance_x(self, obj_center, screen_x):
        middle_x1, middle_x2 = (screen_x - screen_x * self.middle_range) / 2, (screen_x + screen_x * self.middle_range) / 2

        x_distance = 0
        # Ekranın saginda ise + olucak
        if obj_center[0] > middle_x2:
            x_distance = obj_center[0] - middle_x2
        # Ekranın solunda ise - olucak
        elif obj_center[0] < middle_x1:
            x_distance = obj_center[0] - middle_x1
        
        return x_distance

    def local_camera(self, camera_path, detected_obj: dict=None, object_lock: threading.Lock=None):
        cap = cv2.VideoCapture(camera_path)
        class_name = None

        if not cap.isOpened():
            print(f"Dahili kamera {camera_path} açılamadı")
        
        while not self.stop_event.is_set() and self.running:
            _, frame = cap.read()

            if frame is not None:
                res = frame.shape[:2]
                self.screen_res = (int(res[1]), int(res[0]))

                if self.proccessing and self.model != None:
                    detected_obj = ""
                    object_pos = None

                    # 2. Görüntü işleme başlasın
                    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
                    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

                    for color_mask, shape_name, target_sides, color in [
                        (red_mask, "Ucgen", 3, red_bg),
                        (blue_mask, "Altigen", 6, blue_bg)
                    ]:
                        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        for cnt in contours:
                            epsilon = 0.02 * cv2.arcLength(cnt, True)
                            approx = cv2.approxPolyDP(cnt, epsilon, True)
                            if len(approx) == target_sides and is_equilateral(approx):
                                cv2.drawContours(frame, [approx], 0, color, 2)
                                x, y = approx[0][0]
                                cv2.putText(frame, shape_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                                detected_obj = shape_name
                                object_pos = (x, y)


                if object_lock != None and detected_obj != None:
                    if detected_obj == None:
                        with object_lock:
                            detected_obj["cls"] = None
                            detected_obj["pos"] = None
                            detected_obj["screen_res"] = self.screen_res

                    else:
                        with object_lock:
                            detected_obj["cls"] = detected_obj
                            detected_obj["pos"] = object_pos
                            detected_obj["screen_res"] = self.screen_res
                        

                if self.show_crosshair:
                    height, width, _ = frame.shape

                    # Ortadaki + işaretinin koordinatları
                    center_x = width // 2
                    center_y = height // 2
                    cross_size = 20  # Artı işaretinin uzunluğu

                    # Yatay çizgi
                    cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), self.crosshair_color, 2)
                    # Dikey çizgi
                    cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), self.crosshair_color, 2)

                            
                self.visualize_box(frame)
                
                if self.showing_image:
                    cv2.imshow(self.window_name, frame)

            # Çıkış için 'q' tuşuna basılması beklenir
            if cv2.waitKey(10) & 0xFF == ord('g'):
                self.running = False
                break

            self.broadcast_started = True

    def udp_camera_new(self, port, detected_obj: dict=None, detected_obj_lock: threading.Lock=None):
        BUFFER_SIZE = 65536
        HEADER_FMT = '<LHB'
        HEADER_SIZE = struct.calcsize(HEADER_FMT)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip = "0.0.0.0"
        sock.bind((ip, port))

        buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
        expected_counts = {}  # {frame_id: total_chunks, …}

        class_name = None

        while not self.stop_event.is_set() and self.running:
            packet, _ = sock.recvfrom(BUFFER_SIZE)
            frame_id, chunk_id, is_last = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
            chunk_data = packet[HEADER_SIZE:]
            
            # Kaydet
            if frame_id not in buffers:
                buffers[frame_id] = {}
            buffers[frame_id][chunk_id] = chunk_data
            
            # Toplam parça sayısını son pakette işaretle
            if is_last:
                expected_counts[frame_id] = chunk_id + 1

            # Hepsi geldiyse işle
            if frame_id in expected_counts and len(buffers[frame_id]) == expected_counts[frame_id]:
                # Birleştir
                data = b''.join(buffers[frame_id][i] for i in range(expected_counts[frame_id]))
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None:
                    if self.ters:
                        frame = cv2.flip(frame, -1)
                    self.screen_res = (frame.shape[:2][1], frame.shape[:2][0])

                    if self.proccessing and self.model != None:
                        results = self.model(frame, verbose=False)
                        for r in results:
                            boxes = r.boxes
                            for box in boxes:
                                if box.conf[0] < 0.8:
                                    continue
                                # Sınırlayıcı kutu koordinatlarını al
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                obj_center = ((x1+x2) / 2, (y1+y2) / 2)

                                # Sınıf ve güven skorunu al
                                cls = int(box.cls[0].item())
                                conf = box.conf[0].item()

                                # Sınıf adını al
                                class_name = self.model.names[cls]

                                # Bilgileri ekrana yazdır
                                #print(f"Algilanan hedef>> {class_name}, Güven: {conf:.2f}")

                                # Nesneyi çerçeve içine al ve etiketle
                                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                                
                    if detected_obj_lock != None and detected_obj != None:
                        if class_name == None:
                            with detected_obj_lock:
                                detected_obj["cls"] = None
                                detected_obj["pos"] = None
                                detected_obj["dist"] = None
                                detected_obj["screen_res"] = self.screen_res

                        else:
                            with detected_obj_lock:
                                detected_obj["cls"] = class_name
                                detected_obj["pos"] = obj_center
                                detected_obj["dist"] = self.get_distance_x(obj_center, screen_x=self.screen_res[0])
                                detected_obj["lt"] = time.time()
                                detected_obj["screen_res"] = self.screen_res
                                

                    if self.show_crosshair:
                        height, width, _ = frame.shape

                        # Ortadaki + işaretinin koordinatları
                        center_x = width // 2
                        center_y = height // 2
                        cross_size = 20  # Artı işaretinin uzunluğu

                        # Yatay çizgi
                        cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), self.crosshair_color, 1)
                        # Dikey çizgi
                        cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), self.crosshair_color, 1)

                    self.visualize_box(frame)

                    if self.showing_image:
                        self.broadcast_started = True
                        cv2.imshow(self.window_name, frame)

                if cv2.waitKey(10) & 0xFF == ord('g'):
                    self.running = False
                    break
        
        # Temizlik
        if frame_id in buffers:
            del buffers[frame_id]
        if frame_id in expected_counts:
            del expected_counts[frame_id]

    def ui_camera_updater(self, ip, port, camera_label, detected_obj: dict=None, detected_obj_lock: threading.Lock=None):
        from PIL import Image, ImageTk
        
        img = Image.new('RGB', (self.camera_width, self.camera_height), (0, 0, 0))
        imgtk = ImageTk.PhotoImage(image=img)
        camera_label.imgtk = imgtk
        camera_label.configure(image=imgtk)
        
        BUFFER_SIZE = 65536
        HEADER_FMT = '<LHB'
        HEADER_SIZE = struct.calcsize(HEADER_FMT)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))

        buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
        expected_counts = {}  # {frame_id: total_chunks, …}

        while not self.stop_event.is_set() and self.running:
            packet, _ = sock.recvfrom(BUFFER_SIZE)
            frame_id, chunk_id, is_last = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
            chunk_data = packet[HEADER_SIZE:]
            
            # Kaydet
            if frame_id not in buffers:
                buffers[frame_id] = {}
            buffers[frame_id][chunk_id] = chunk_data
            
            # Toplam parça sayısını son pakette işaretle
            if is_last:
                expected_counts[frame_id] = chunk_id + 1

            # Hepsi geldiyse işle
            if frame_id in expected_counts and len(buffers[frame_id]) == expected_counts[frame_id]:
                # Birleştir
                data = b''.join(buffers[frame_id][i] for i in range(expected_counts[frame_id]))
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None:
                    if self.ters:
                        frame = cv2.flip(frame, -1)
                    self.screen_res = (frame.shape[:2][1], frame.shape[:2][0])

                    x, y = (int(self.screen_res[1]), int(self.screen_res[0]))
                    if self.proccessing and self.model != None:
                        results = self.model(frame, verbose=False)
                        for r in results:
                            boxes = r.boxes
                            for box in boxes:
                                if box.conf[0] < 0.80:
                                    continue
                                # Sınırlayıcı kutu koordinatlarını al
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                x_center, y_center = (x1 + x2)/2, (y1 + y2)/2

                                middle_x1, middle_x2 = (x- x*self.middle_range)/2, (x + x*self.middle_range)/2
                                middle_y1, middle_y2 = (y- y*self.middle_range)/2, (y + y*self.middle_range)/2

                                x_distance = 0
                                if x_center > middle_x2:
                                    x_distance = x_center - middle_x2
                                elif x_center < middle_x1:
                                    x_distance = x_center - middle_x1
                                
                                y_distance = 0
                                if y_center > middle_y2:
                                    y_distance = y_center - middle_y2
                                elif y_center < middle_y1:
                                    y_distance = y_center - middle_y1

                                # Sınıf ve güven skorunu al
                                cls = int(box.cls[0].item())
                                conf = box.conf[0].item()

                                # Sınıf adını al
                                class_name = self.model.names[cls]

                                # Bilgileri ekrana yazdır
                                #print(f"Algilanan hedef>> {class_name}, Güven: {conf:.2f}")

                                # Nesneyi çerçeve içine al ve etiketle
                                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                                
                                if detected_obj_lock != None:
                                    with detected_obj_lock:
                                        detected_obj["cls"] = class_name
                                        detected_obj["dist"] = (x_distance, y_distance)
                                        detected_obj["lt"] = time.time()
                                        detected_obj["screen_res"] = self.screen_res
                            

                    if self.show_crosshair:
                        height, width, _ = frame.shape

                        # Ortadaki + işaretinin koordinatları
                        center_x = width // 2
                        center_y = height // 2
                        cross_size = 20  # Artı işaretinin uzunluğu

                        # Yatay çizgi
                        cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), self.crosshair_color, 1)
                        # Dikey çizgi
                        cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), self.crosshair_color, 1)
                    
                if frame:
                    frame = cv2.resize(frame, (self.camera_width, self.camera_height))
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(frame)
                else:
                    img = Image.new('RGB', (self.camera_width, self.camera_height), (0, 0, 0))

                imgtk = ImageTk.PhotoImage(image=img)
                camera_label.imgtk = imgtk
                camera_label.configure(image=imgtk)
                time.sleep(0.03)

                if cv2.waitKey(10) & 0xFF == ord('g'):
                    self.running = False
                    break

            self.broadcast_started = True
        
        # Temizlik
        if frame_id in buffers:
            del buffers[frame_id]
        if frame_id in expected_counts:
            del expected_counts[frame_id]

    def visualize_box(self, frame):
        oran = 0.5
        if self.show_box:
            if self.oran != None:
                oran = self.oran

            x1 = (self.screen_res[0] - (self.screen_res[0] * oran)) / 2
            x2 = self.screen_res[0] - x1
            y1 = (self.screen_res[1] - (self.screen_res[1] * oran)) / 2
            y2 = self.screen_res[1] - y1

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), self.orta_box_color, 1)

    def show_hide_box(self, show, oran, color=(255, 0, 255)):
        self.show_box = show
        self.oran = oran
        self.orta_box_color = color
    
    def show_hide_crosshair(self, show):
        self.show_crosshair = show
    
    def show_image(self, window_name: str="UDP Goruntu"):
        self.showing_image = True
        self.window_name = window_name
    
    def hide_image(self):
        self.showing_image = False
    
    def start_proccessing(self, model_path):
        if self.model == None:
            self.model = YOLO(model_path)
            print("model yuklendi")
            self.proccessing = True
    
    def stop_proccessing(self):
        self.proccessing = False
    
    def stop_camera(self):
        self.running = False
    
    def set_ters(self, ters: bool):
        self.ters = ters
