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

#! fire-poster
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

def second_miss(stop_event, vehicle: Vehicle, config, targets, target_locker):
    def second_miss(stop_event, config, model_path, vehicle: Vehicle, DRONE_ID, target_loc, target_cls):
        detected_obj = {"cls": None, "pos": None, "dist": None, "screen_res": None}
        detected_obj_lock = threading.Lock()

        ALT = config["alt"]
        fov = config["FOV"]
        orta_orani = config["ORTA-ORAN"]
        gimbal_channel = config["GIMBAL"]["channel"]
        gimbal_angles = config["GIMBAL"]["pwms"]

        #? Drone'dan goruntu isleme
        saldiri_camera_handler = image_processing_handler.Handler(stop_event=stop_event)
        saldiri_camera_handler.start_proccessing(model_path)
        saldiri_camera_handler.show_hide_crosshair(False)
        saldiri_camera_handler.show_image(f"Drone {DRONE_ID} kamera goruntusu")
        saldiri_camera_handler.set_ters(False)
        
        saldiri_camera_handler.show_hide_box(show=True, oran=config["ORTA-ORAN"])

        threading.Thread(target=saldiri_camera_handler.udp_camera_new, args=(config["UDP"]["port"], detected_obj, detected_obj_lock), daemon=True).start()
        #threading.Thread(target=saldiri_camera_handler.local_camera, args=(0, detected_obj, detected_obj_lock), daemon=True).start()

        while not saldiri_camera_handler.broadcast_started:
            time.sleep(0.5)

        #? Ucus hazirligi
        servo_pwm = 0
        vehicle.set_servo(channel=config["GIMBAL"]["channel"], pwm=config["GIMBAL"]["pwms"][servo_pwm], drone_id=DRONE_ID)
        current_servo_pwm = config["GIMBAL"]["pwms"][servo_pwm]
        vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
        vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
        vehicle.multiple_takeoff(alt=ALT, drone_id=DRONE_ID)
        
        #? Takeoff
        current_alt = 0
        start_time = time.time()
        while current_alt < ALT * 0.9 and not stop_event.is_set():
            current_alt = vehicle.get_pos(drone_id=DRONE_ID)[2]
            if time.time() - start_time > 2:
                print(f"{DRONE_ID}>> Anlık irtifa: {current_alt} metre")
                start_time = time.time()

        home_pos = vehicle.get_pos(drone_id=DRONE_ID)
        print(f"{DRONE_ID}>> takeoff yaptı")
            
        vehicle.go_to(loc=target_loc, alt=ALT, drone_id=DRONE_ID)

        start_time = time.time()
        while not stop_event.is_set():
            if time.time() - start_time > 5:
                print(f"{DRONE_ID}>> isaretlenen hedefe gidiyor...")
                start_time = time.time()

            if vehicle.on_location(loc=target_loc, seq=0, sapma=1, drone_id=DRONE_ID):
                print(f"{DRONE_ID}>> isaretlenen hedefe ulaştı")
                break
        
        #? Etrafinda donerek tarama
        centered = False
        start_time = time.time()
        while not stop_event.is_set() and saldiri_camera_handler.running:
            if time.time() - start_time >= 5:
                print(f"{DRONE_ID}>> Nesne araniyor")
                start_time = time.time()

            with detected_obj_lock:
                obj = detected_obj["cls"]

            if obj == None:
                start_time = time.time()
                while not stop_event.is_set() and time.time() - start_time <= 0.2 and obj == None:
                    with detected_obj_lock:
                        obj = detected_obj["cls"]
                    
                    time.sleep(0.01)

            #? Hedefi ortalama
            if obj != None:
                if obj == target_cls:
                    print(f"{DRONE_ID}>> {obj} algilandi hedef ortalaniyor")
                    centered, current_servo_pwm = go_to_obj(vehicle=vehicle, DRONE_ID=DRONE_ID, orta_orani=orta_orani, gimbal_channel=gimbal_channel, gimbal_angles=gimbal_angles, servo_pwm=current_servo_pwm, fov=fov, target_cls=target_cls, detected_obj=detected_obj, detected_obj_lock=detected_obj_lock, stop_event=stop_event)

                    if centered:
                        print(f"{DRONE_ID}>> Yuk birakiliyor")
                        vehicle.set_servo(channel=config["YUK"]["channel"], pwm=config["YUK"]["acik"], drone_id=DRONE_ID)
                        
                        start_time = time.time()
                        while not stop_event.is_set() and time.time() - start_time < 2:
                            time.sleep(0.05)

                        vehicle.set_servo(channel=config["YUK"]["channel"], pwm=config["YUK"]["kapali"], drone_id=DRONE_ID)

                        start_time = time.time()
                        while not stop_event.is_set() and time.time() - start_time < 2:
                            time.sleep(0.05)
                        
                        print(f"{DRONE_ID}>> Yuk birakildi kalkis konumuna donuyor")
                        break
            
                    #? Taramaya devam etme
                    else:
                        print(f"{DRONE_ID}>> Hedef {target_cls} ortalanamadi tekrar taraniyor")
                        if servo_pwm == 0:
                            servo_pwm += 1
                        vehicle.set_servo(channel=config["GIMBAL"]["channel"], pwm=config["GIMBAL"]["pwms"][servo_pwm], drone_id=DRONE_ID)
                        vehicle.turn_around(default_speed=15, drone_id=DRONE_ID)
            
            elif (obj == None or obj != target_cls) and vehicle.yaw_speed(drone_id=DRONE_ID) < 0.2:
                if servo_pwm + 1 < len(config["GIMBAL"]["pwms"]):
                    servo_pwm += 1
                else:
                    print(f"{DRONE_ID}>> Hedef bulunamadi kalkisa donuyor")
                    break
                vehicle.set_servo(channel=config["GIMBAL"]["channel"], pwm=config["GIMBAL"]["pwms"][servo_pwm], drone_id=DRONE_ID)
                vehicle.turn_around(default_speed=15, drone_id=DRONE_ID)

                while vehicle.yaw_speed(drone_id=DRONE_ID) <= 0.2:
                    time.sleep(0.05)
            
            time.sleep(0.05)

        go_home(stop_event=stop_event, vehicle=vehicle, home_pos=home_pos, DRONE_ID=DRONE_ID)
        print(f"{DRONE_ID}>> Gorevini tamamladi")
        
    
    miss_thrds = []
    with target_locker:
        for drone_id in targets:
            target_loc = targets[drone_id]["loc"]
            target_cls = targets[drone_id]["cls"]
            drone_conf = None

            for saldiri_iha in config["DRONES"]:
                if saldiri_iha["id"] == drone_id:
                    drone_conf = saldiri_iha
                    break
            
            if drone_conf is None:
                raise ValueError(f"{drone_id}>> Drone config bulunamadi")

            thrd = threading.Thread(target=second_miss, args=(stop_event, drone_conf, config["MODEL-PATH"], vehicle, drone_id, target_loc, target_cls), daemon=True)
            thrd.start()
            miss_thrds.append(thrd)
    
    if len(miss_thrds) != 0:
        for miss_thrd in miss_thrds:
            miss_thrd.join()


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
conf_file = "./config.json"
if len(sys.argv) == 2:
    conf_file = "./config-test.json"
config = json.load(open(conf_file))
stop_event = threading.Event()

#? YKI
yki_monitor = yki_handler.YKIMonitor(config=config, stop_event=stop_event)
yki_monitor.start()

#? Gozlemci kontrolculeri
gimbal_server = tcp_handler.TCPServer(port=config["GOZLEMCI"]["TCP"]["port"], stop_event=stop_event)
gozlemci_camera_handler = image_processing_handler.Handler(stop_event=stop_event)
gozlemci_camera_handler.show_image("Gozlemci ekrani")
gozlemci_camera_handler.set_ters(True)
gimbal_handler = gimbal_controller.GimbalHandler(server=gimbal_server, stop_event=stop_event)

threading.Thread(target=gozlemci_camera_handler.udp_camera_new, args=(config["GOZLEMCI"]["UDP"]["port"], ), daemon=True).start()
threading.Thread(target=gimbal_handler.joystick_controller, args=(yki_monitor, ), daemon=True).start()

#? Uçuş hazırlıkları
GOZLEMCI_ALT = config["GOZLEMCI"]["alt"]
GOZLEMCI_ID = int(config["GOZLEMCI"]["id"])

vehicle = Vehicle(config["CONN-PORT"])

#? Hedef seçme silme threadi
targets = {}
target_locker = threading.Lock()
selecter_started = threading.Event()
selecter_thrd = threading.Thread(target=gimbal_handler.gimbal_selecter, args=(stop_event, vehicle, GOZLEMCI_ID, gimbal_server, targets, target_locker, selecter_started, yki_monitor), daemon=True)

start_time = 0
while not stop_event.is_set():
    if yki_monitor.get_system_status():
        break
    if time.time() - start_time >= 5:
        print("\rYKI Başlatılması Bekleniyor", end="")
        start_time = time.time()

    time.sleep(0.5)

print("\nYKI Baslatildi")
selecter_thrd.start()

try:
    vehicle.set_mode(mode="GUIDED", drone_id=GOZLEMCI_ID)
    vehicle.arm_disarm(arm=True, drone_id=GOZLEMCI_ID)
    vehicle.takeoff(GOZLEMCI_ALT, drone_id=GOZLEMCI_ID)

    home_pos = vehicle.get_pos(drone_id=GOZLEMCI_ID)
    
    print(f"{GOZLEMCI_ID}>> takeoff yaptı")
    print(f"{GOZLEMCI_ID}>> hedefleri arıyor...")

    #? 1. aşama (tcp verisi bekleniyor)
    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time > 5:
            if targets != {}:
                print(targets)
            print(f"{GOZLEMCI_ID}>> hedefler aranıyor...")
            start_time = time.time()
        
        if not selecter_thrd.is_alive():
            GOZLEMCI_go_home = threading.Thread(target=go_home, args=(stop_event, vehicle, home_pos, GOZLEMCI_ID), daemon=True)
            GOZLEMCI_go_home.start()

            # Görev bitince kamera kapansın
            gozlemci_camera_handler.stop_camera()
            print("İHA'lar hedeflere saldiriya gecti")
            break
        
        time.sleep(0.05)
        
    #? 2. aşama (tcp verisine gidiliyor)
    second_miss_thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, config, targets, target_locker), daemon=True)
    second_miss_thrd.start()
    
    while (GOZLEMCI_go_home.is_alive() or second_miss_thrd.is_alive()) and not stop_event.is_set():
        time.sleep(2)
    
    print("Görev tamamlandı")

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