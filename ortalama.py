import threading
import json
import time
import sys
sys.path.append("./libs/")

from pymavlink_custom.pymavlink_custom import Vehicle
import libs.image_processing_handler as image_processing_handler
from libs.ortalama_fonksiyonlari import *

def failsafe(vehicle):
    def failsafe_drone_id(vehicle, drone_id):
        print(f"{drone_id}>> Failsafe alıyor")
        vehicle.set_mode(mode="RTL", drone_id=drone_id)

    thraeds = []
    for d_id in vehicle.drone_ids:
        args = (vehicle, d_id)

        thrd = threading.Thread(target=failsafe_drone_id, args=args)
        thrd.start()
        thraeds.append(thrd)


    for t in thraeds:
        t.join()

    print(f"{vehicle.drone_ids} id'li Drone(lar) Failsafe aldi")

def go_home(stop_event, vehicle: Vehicle, home_pos, DRONE_ID):
    vehicle.go_to(loc=home_pos, drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")

    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> iniş gerçekleşiyor")
            vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
            break


#? Gerekliler
config = json.load(open("./tracking-conf.json"))
drone_conf = config["DRONE"]
stop_event = threading.Event()

#? Uçuş hazırlıkları
ALT = config["DRONE"]["alt"]
DRONE_ID = int(config["DRONE"]["id"])

model_path = config["MODEL-PATH"]

vehicle = Vehicle(config["CONN-PORT"])

orta_orani = drone_conf["CAMERA"]["oran"]
gimbal_channel = drone_conf["GIMBAL"]["channel"]
x_fov, y_fov = None, None
if "fov" in drone_conf["UDP"]:
    if len(drone_conf["UDP"]["fov"]) == 2:
        x_fov, y_fov = drone_conf["UDP"]["fov"]

ucus = False
if len(sys.argv) == 2:
    if sys.argv[1] == "ucus":
        ucus = True
        print("UCUS")

try:
    detected_obj = {"cls": None, "pos": (), "dist": 0, "lt": 0, "screen_res": None}
    detected_obj_lock = threading.Lock()

    fov = drone_conf["UDP"]["fov"]

    # Drone'dan goruntu isleme
    saldiri_camera_handler = image_processing_handler.Handler(stop_event=stop_event)
    saldiri_camera_handler.start_proccessing(model_path)
    saldiri_camera_handler.show_hide_crosshair(True)
    saldiri_camera_handler.set_ters(False)
    saldiri_camera_handler.show_hide_box(show=True, oran=drone_conf["CAMERA"]["oran"])

    #threading.Thread(target=saldiri_camera_handler.local_camera, args=(0, detected_obj, detected_obj_lock), daemon=True).start()
    threading.Thread(target=saldiri_camera_handler.udp_camera_new, args=(drone_conf["UDP"]["port"], detected_obj, detected_obj_lock), daemon=True).start()

    while not saldiri_camera_handler.broadcast_started and not stop_event.is_set():
        time.sleep(0.5)
        

    if ucus:
        current_servo_pwm = 1600
        vehicle.set_servo(channel=gimbal_channel, pwm=current_servo_pwm, drone_id=DRONE_ID)

        input("Ucus baslatilmasi bekleniyor ENTER")

        # Ucus hazirligi
        vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
        time.sleep(0.5)
        vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
        time.sleep(0.5)
        vehicle.takeoff(alt=ALT, drone_id=DRONE_ID)
        
        home_pos = vehicle.get_pos(drone_id=DRONE_ID)
        print(f"{DRONE_ID}>> takeoff yaptı")

        vehicle.turn_around(default_speed=15, drone_id=DRONE_ID)
        centered = False
        start_time = time.time()
        while not stop_event.is_set() and saldiri_camera_handler.running:
            if time.time() - start_time >= 5:
                print("Nesne araniyor")
                start_time = time.time()

            with detected_obj_lock:
                obj = detected_obj["cls"]

            if obj == None:
                start_time = time.time()
                while not stop_event.is_set() and time.time() - start_time <= 0.2 and obj == None:
                    with detected_obj_lock:
                        obj = detected_obj["cls"]
                    
                    time.sleep(0.01)

            if obj != None:
                print(f"{DRONE_ID}>> {obj} algilandi hedef ortalaniyor")
                centered = go_to_obj(vehicle, DRONE_ID, config, detected_obj, detected_obj_lock, stop_event)

                if centered:
                    print("5 sn sonra LAND Aliniyor")
                    time.sleep(5)
                    print("LAND Aliniyor")
                    vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
                    break

            time.sleep(0.05)

        #go_home(stop_event=stop_event, vehicle=vehicle, home_pos=home_pos, DRONE_ID=DRONE_ID)

    else:
        servo_angle = int(input("Servo acisi girin: "))
        if servo_angle <= 90 and servo_angle >= 0:
            vehicle.set_servo(channel=gimbal_channel, pwm=angle_to_pwm(servo_angle), drone_id=DRONE_ID)
        else:
            print("Yanlis servo degeri deger araligi: 0-90")
            exit(1)
        
        current_servo_pwm = angle_to_pwm(servo_angle)

        while not stop_event.is_set() and saldiri_camera_handler.running:
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

            if obj != None:
                if camera_distance(pos, screen_res, orta_orani)[1] != 0:
                    print(f"{DRONE_ID}>> {obj} algilandi ortaliyor")
                    y_centered = adjust_y(orta_orani, y_fov, gimbal_channel, vehicle, DRONE_ID, detected_obj, detected_obj_lock, stop_event)
                    print("centered: ",y_centered)

            time.sleep(0.05)
    
    print("Görev tamamlandi")
        
except KeyboardInterrupt:
    print("Exiting...")
    failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()
    print(e)
    
finally:
    if not stop_event.is_set():
        stop_event.set()
    vehicle.vehicle.close()

