import threading
import json
import time
import sys
sys.path.append("./libs/")

from pymavlink_custom.pymavlink_custom import Vehicle
import libs.image_processing_handler as image_processing_handler

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

def pwm_to_angle(pwm_val):
    return 90 - (90 / (1000/(pwm_val - 999)))

def angle_to_pwm(angle):
    return 2000 - ((1000 / 90) * angle)

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
    
def camera_distance(pos: list, screen_res: list, oran: float=0.3):
    orta_x1 = (screen_res[0] - (screen_res[0] * oran)) / 2
    orta_x2 = screen_res[0] - orta_x1
    orta_y1 = (screen_res[1] - (screen_res[1] * oran)) / 2
    orta_y2 = screen_res[1] - orta_y1

    pos_x, pos_y = pos

    x_dist = 0
    y_dist = 0
    
    if pos_x < orta_x1:
        x_dist = pos_x - orta_x1
    elif pos_x > orta_x2:
        x_dist = orta_x2 - pos_x
    if pos_y < orta_y1:
        y_dist = orta_y1 - pos_y
    elif pos_y > orta_y2:
        y_dist = orta_y2 - pos_y
    
    return x_dist, y_dist

def center_distance(pos: list, screen_res: list):
    return pos[0] - screen_res[0] / 2, screen_res[1] / 2 - pos[1]

def turn_angle_calculate(dist, fov, screen_res):
    return (fov * dist) / screen_res

def adjust_x(orta_orani, x_fov):
    print("Hedefe Donuluyor")

    x_centered = False

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

    if obj == None:
        print("Hedef Kayboldu")
        return False

    azaltan = 1
    while not stop_event.is_set() and not x_centered:
        x_dist = center_distance(pos, screen_res)[0]
        print(f"X Dist: {x_dist}")

        # Donulecek aciyi bul
        turn_angle = turn_angle_calculate(x_dist, x_fov, screen_res[0]) * azaltan # Nesne ekrandan kayarsa daha az aci ile tekrar denencek
        if abs(turn_angle) <= 5:
            x_centered = True
            break

        #TODO: bu aci tam denk gelicek sekilde tek seferde dondur
        print(f"X Donulcek aci: {turn_angle}")

        vehicle.turn_way(turn_angle=turn_angle, drone_id=DRONE_ID)

        # Donmeye baslasin diye bekleme
        while vehicle.yaw_speed(drone_id=DRONE_ID) < 0.01 and not stop_event.is_set():
            time.sleep(0.01)

        # Donmenin bitmesini bekle
        while vehicle.yaw_speed(drone_id=DRONE_ID) >= 0.02 and not stop_event.is_set():
            time.sleep(0.01)
    
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
            
        # Donmeden sonra hedef kaybolursa ilk aciya donucek
        if obj == None:
            print("Hedef kayboldu eski yone donuluyor")
            vehicle.turn_way(turn_angle * -1, drone_id=DRONE_ID)
            azaltan /= 1.5

            # Donmeye baslasin diye bekleme
            while vehicle.yaw_speed(drone_id=DRONE_ID) < 0.01 and not stop_event.is_set():
                time.sleep(0.01)

            # Donmenin bitmesini bekle
            while vehicle.yaw_speed(drone_id=DRONE_ID) >= 0.02 and not stop_event.is_set():
                time.sleep(0.01)
        
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

            # İlk acida hedef yine kayip ise bu sefer taramaya donucek
            if obj == None:
                print("Hedef kayboldu")
                return False
        
        else:
            if camera_distance(pos, screen_res, orta_orani)[0] == 0:
                print("Drone hedefe dondu")
                x_centered = True
            
            else:
                azaltan /= 1.5
                if azaltan <= 0.2:
                    print("Dondurme basarisiz oldu")
                    return False
    
    return x_centered

def adjust_y(orta_orani, y_fov, gimbal_channel):
    global current_servo_pwm

    y_centered = False

    print("Servo hedefe donuyor")
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

    if obj == None:
        print("Hedef Kayboldu")
        return False
    
    azaltan = 1
    # Kamerayı dondurme
    while not y_centered and not stop_event.is_set():
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

        if obj == None:
            print("Nesne kayboldu")
            return False

        y_dist = center_distance(pos, screen_res)[1]
        print(f"Y Dist: {y_dist}")

        aci_degisimi = turn_angle_calculate(y_dist, y_fov, screen_res[1]) * azaltan # Nesne ekrandan kayarsa daha az aci ile tekrar denencek
        print(f"Y Aci degisimi: {aci_degisimi}")

        # Mevcut servo acisi
        print(f"Y Current pwm: {current_servo_pwm}")
        
        # Servoyu dondurulcek aci
        print("Y Current Angle: ", pwm_to_angle(current_servo_pwm))
        old_servo_pwm = current_servo_pwm
        current_servo_pwm = angle_to_pwm(aci_degisimi + pwm_to_angle(current_servo_pwm))
        print(f"Y Ayarlancak servo pwm: {current_servo_pwm}")

        # Servoyu dondurme
        if current_servo_pwm > 2000 or current_servo_pwm < 1000:
            if current_servo_pwm > 2000:
                current_servo_pwm = 2000
            elif current_servo_pwm < 1000:
                current_servo_pwm = 1000

        vehicle.set_servo(channel=gimbal_channel, pwm=current_servo_pwm)

        # Servo donene kadar 1 sn bekleme
        start_time = time.time()
        while time.time() - start_time < 1 and not stop_event.is_set():
            time.sleep(0.05)

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


        if obj == None:
            print("Hedef Kayboldu gimbal eski acisina getiriliyor")
            
            current_servo_pwm = old_servo_pwm
            vehicle.set_servo(channel=gimbal_channel, pwm=current_servo_pwm)

            azaltan /= 1.5
            
        # Hedef kayip degilse ve ortalanmissa
        else:
            # Orta nokta icinde ise
            y_dist = camera_distance(pos, screen_res, orta_orani)[1]
            
            if y_dist == 0 or (y_dist < 0 and current_servo_pwm == 1000) or (y_dist > 0 and current_servo_pwm == 2000):
                print(f"Y Ekseninde ortalandi")
                y_centered = True
            else:
                azaltan /= 1.5
                if azaltan <= 0.2:
                    print("Dondurme basarisiz oldu")
                    return False
        
        time.sleep(0.05)
    
    return y_centered


def go_to_obj(vehicle: Vehicle, DRONE_ID, config: dict, detected_obj: dict, detected_obj_lock: threading.Lock, stop_event: threading.Event):
    '''
    Eğer nesnenin uzerine ortalanırsa True dondurur, Nesne kayboldu ise False dondurur
    '''

    vehicle.move_drone_body((0,0,0), drone_id=DRONE_ID)
    
    while vehicle.yaw_speed(drone_id=DRONE_ID) >= 0.1 and not stop_event.is_set():
        time.sleep(0.05)
    
    start_time = time.time()
    while time.time() - start_time < 1.5 and not stop_event.is_set():
        time.sleep(0.05)

    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)

    orta_orani = config["CAMERA"]["oran"]
    gimbal_channel = config["GIMBAL"]["channel"]
    gimbal_angles = config["GIMBAL"]["pwms"]
    x_fov, y_fov = None, None
    if "fov" in config["UDP"]:
        if len(config["UDP"]["fov"]) == 2:
            x_fov, y_fov = config["UDP"]["fov"]
    
    if x_fov == None or y_fov == None:
        print("Config dosyasina fov degerleri giriniz")
        raise ValueError("Hatali config dosyasi")

    x_centered = adjust_x(orta_orani, x_fov)
    y_centered = adjust_y(orta_orani, y_fov, gimbal_channel)
    
    if not x_centered or not y_centered:
        print("Hedef eksenlerde ortalanamadi")
        return False

    ilerleme_hizi = 0.5
    while not stop_event.is_set():
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
        
        if obj == None:
            print("Hedef kayboldu")
            return False
        

        x_dist, y_dist = camera_distance(pos, screen_res, orta_orani)

        print("Uzakliklar: ", x_dist, y_dist)

        # Nesne ortalanmis ya da gerisinde kalmis ise
        if (y_dist == 0 or y_dist < 0) and (current_servo_pwm >= gimbal_angles[0] - 200 and current_servo_pwm <= gimbal_angles[0]):
            # Dronu ilerlemesini durdurma
            vehicle.move_drone_body((0,0,0), drone_id=DRONE_ID)

            while vehicle.get_speed(drone_id=DRONE_ID) >= 0.1 and not stop_event.is_set():
                time.sleep(0.05)
            
            start_time = time.time()
            while time.time() - start_time < 1 and not stop_event.is_set():
                time.sleep(0.05)

            vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)

            print("Drone hedef ustune getirildi")
            return True
        
        elif y_dist != 0:
            if adjust_y(orta_orani, y_fov, gimbal_channel) == False:
                print("Gimbal hedefi ortalamadi")
                return False
        
        print(f"{DRONE_ID}>> Drone {ilerleme_hizi} hizi ile ilerliyor...")
        vehicle.move_drone_body(rota=(ilerleme_hizi, 0, 0), drone_id=DRONE_ID)

        while not stop_event.is_set() and vehicle.get_speed(drone_id=DRONE_ID) < 0.05:
            time.sleep(0.05)
        
        while not stop_event.is_set() and vehicle.get_speed(drone_id=DRONE_ID) >= 0.05:
            time.sleep(0.05)

        time.sleep(0.01)

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

        centered = False
        start_time = time.time()
        while not stop_event.is_set() and saldiri_camera_handler.running:
            if time.time() - start_time >= 5:
                print("Nesne araniyor")
                start_time = time.time()

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
                print(f"{DRONE_ID}>> {obj} algilandi hedef ortalaniyor")
                centered = go_to_obj(vehicle, DRONE_ID, drone_conf, detected_obj, detected_obj_lock, stop_event)

                if centered:
                    print("Gorev bitiriliyiyor")
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
                print(f"{DRONE_ID}>> {obj} algilandi ortaliyor")

                y_centered = adjust_y(orta_orani, y_fov, gimbal_channel)
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
