import threading
import json
import time
import sys
sys.path.append("./libs/")

from pymavlink_custom.pymavlink_custom import Vehicle
import libs.tcp_handler as tcp_handler
from libs.ortalama_fonksiyonlari import *
import libs.image_processing_handler as image_processing_handler
import libs.gimbal_controller as gimbal_controller
import libs.yki_handler as yki_handler

conf_file = "./config-test.json"
config = json.load(open(conf_file))
stop_event = threading.Event()

drone_conf = config["DRONES"][1]
DRONE_ID = drone_conf["id"]

vehicle = Vehicle(config["CONN-PORT"])
for i in drone_conf["GIMBAL"]["pwms"]:
    servo_pwm = i
    vehicle.set_servo(channel=drone_conf["GIMBAL"]["channel"], pwm=servo_pwm)
    time.sleep(1)

servo_pwm = drone_conf["GIMBAL"]["pwms"][1]
vehicle.set_servo(channel=drone_conf["GIMBAL"]["channel"], pwm=servo_pwm)
        
detected_obj = {"cls": None, "pos": None, "dist": None, "screen_res": None}
detected_obj_lock = threading.Lock()
        
saldiri_camera_handler = image_processing_handler.Handler(stop_event=stop_event)
saldiri_camera_handler.start_proccessing(config["MODEL-PATH"])
saldiri_camera_handler.show_hide_crosshair(False)
saldiri_camera_handler.show_image(f"{DRONE_ID} canli kamera")
saldiri_camera_handler.set_ters(False)

saldiri_camera_handler.show_hide_box(show=True, oran=drone_conf["ORTA-ORAN"])

threading.Thread(target=saldiri_camera_handler.local_camera, args=(0, detected_obj, detected_obj_lock), daemon=True).start()

while True:
    with detected_obj_lock:
        obj = detected_obj["cls"]
        pos = detected_obj["pos"]
        screen_res = detected_obj["screen_res"]

    if obj == None:
        start_time = time.time()
        while not stop_event.is_set() and time.time() - start_time <= 0.2 and obj == None:
            with detected_obj_lock:
                obj = detected_obj["cls"]
                pos = detected_obj["pos"]
                screen_res = detected_obj["screen_res"]
            
            time.sleep(0.01)

    #? Hedefi ortalama
    if obj != None:
        if camera_distance(pos, screen_res, oran=drone_conf["ORTA-ORAN"])[1] != 0:
            adjust_y(orta_orani=drone_conf["ORTA-ORAN"], y_fov=drone_conf["FOV"][1], gimbal_channel=drone_conf["GIMBAL"]["channel"], vehicle=vehicle, DRONE_ID=DRONE_ID, target_cls="fire-poster", detected_obj=detected_obj, detected_obj_lock=detected_obj_lock, stop_event=stop_event)
    
    time.sleep(0.05)