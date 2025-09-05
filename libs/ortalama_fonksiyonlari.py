import time
import threading
import sys

sys.path.append("../")
from pymavlink_custom.pymavlink_custom import Vehicle

current_servo_pwm = 0

def pwm_to_angle(pwm_val):
    return 90 - (90 / (1000/(pwm_val - 999)))

def angle_to_pwm(angle):
    return 2000 - ((1000 / 90) * angle)
    
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

def adjust_x(orta_orani, x_fov, vehicle: Vehicle, DRONE_ID: int, target_cls: str, detected_obj: dict, detected_obj_lock: threading.Lock, stop_event: threading.Event):
    print("Hedefe Donuluyor")

    x_centered = False

    with detected_obj_lock:
        obj = detected_obj["cls"]
        pos = detected_obj["pos"]
        screen_res = detected_obj["screen_res"]

    if obj == None or obj != target_cls:
        start_time = time.time()
        while not stop_event.is_set() and time.time() - start_time <= 0.2 and (obj == None or obj != target_cls):
            with detected_obj_lock:
                obj = detected_obj["cls"]
                pos = detected_obj["pos"]
                screen_res = detected_obj["screen_res"]
            
            time.sleep(0.01)

    if obj == None or obj != target_cls:
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

        if obj == None or obj != target_cls:
            start_time = time.time()
            while not stop_event.is_set() and time.time() - start_time <= 0.2 and (obj == None or obj != target_cls):
                with detected_obj_lock:
                    obj = detected_obj["cls"]
                    pos = detected_obj["pos"]
                    screen_res = detected_obj["screen_res"]
                
                time.sleep(0.01)
            
        # Donmeden sonra hedef kaybolursa ilk aciya donucek
        if obj == None or obj != target_cls:
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

            if obj == None or obj != target_cls:
                start_time = time.time()
                while not stop_event.is_set() and time.time() - start_time <= 0.2 and (obj == None or obj != target_cls):
                    with detected_obj_lock:
                        obj = detected_obj["cls"]
                        pos = detected_obj["pos"]
                        screen_res = detected_obj["screen_res"]
                    
                    time.sleep(0.01)

            # İlk acida hedef yine kayip ise bu sefer taramaya donucek
            if obj == None or obj != target_cls:
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

def adjust_y(orta_orani, y_fov, gimbal_channel, vehicle: Vehicle, DRONE_ID: int, target_cls: str, detected_obj: dict, detected_obj_lock: threading.Lock, stop_event: threading.Event):
    global current_servo_pwm

    y_centered = False

    print("Servo hedefe donuyor")
    
    azaltan = 1
    # Kamerayı dondurme
    while not y_centered and not stop_event.is_set():
        with detected_obj_lock:
            obj = detected_obj["cls"]
            pos = detected_obj["pos"]
            screen_res = detected_obj["screen_res"]

        if obj == None or obj != target_cls:
            start_time = time.time()
            while not stop_event.is_set() and time.time() - start_time <= 0.2 and (obj == None or obj != target_cls):
                with detected_obj_lock:
                    obj = detected_obj["cls"]
                    pos = detected_obj["pos"]
                    screen_res = detected_obj["screen_res"]
                
                time.sleep(0.01)

        if obj == None or obj != target_cls:
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

        vehicle.set_servo(channel=gimbal_channel, pwm=current_servo_pwm, drone_id=DRONE_ID)

        # Servo donene kadar 1 sn bekleme
        start_time = time.time()
        while time.time() - start_time < 1 and not stop_event.is_set():
            time.sleep(0.05)

        with detected_obj_lock:
            obj = detected_obj["cls"]
            pos = detected_obj["pos"]
            screen_res = detected_obj["screen_res"]

        if obj == None or obj != target_cls:
            start_time = time.time()
            while not stop_event.is_set() and time.time() - start_time <= 0.2 and (obj == None or obj != target_cls):
                with detected_obj_lock:
                    obj = detected_obj["cls"]
                    pos = detected_obj["pos"]
                    screen_res = detected_obj["screen_res"]
                
                time.sleep(0.01)

        if obj == None or obj != target_cls:
            print("Hedef Kayboldu gimbal eski acisina getiriliyor")
            
            current_servo_pwm = old_servo_pwm
            vehicle.set_servo(channel=gimbal_channel, pwm=current_servo_pwm, drone_id=DRONE_ID)

            azaltan /= 1.5
            
        # Hedef kayip degilse ve ortalanmissa
        else:
            # Orta nokta icinde ise
            y_dist = camera_distance(pos, screen_res, orta_orani)[1]
            
            if y_dist == 0 or (y_dist < 0 and current_servo_pwm == 2000) or (y_dist > 0 and current_servo_pwm == 1000):
                print(f"Y Ekseninde ortalandi")
                y_centered = True
            else:
                azaltan /= 1.5
                if azaltan <= 0.2:
                    print("Dondurme basarisiz oldu")
                    return False
        
        time.sleep(0.05)
    
    return y_centered


def go_to_obj(vehicle: Vehicle, DRONE_ID, orta_orani: float, gimbal_channel: int, gimbal_angles: list, servo_pwm: float, fov: list, target_cls: str, detected_obj: dict, detected_obj_lock: threading.Lock, stop_event: threading.Event):
    '''
    Eğer nesnenin uzerine ortalanırsa True dondurur, Nesne kayboldu ise False dondurur
    '''
    global current_servo_pwm

    current_servo_pwm = servo_pwm

    vehicle.move_drone_body((0,0,0), drone_id=DRONE_ID)
    
    while vehicle.yaw_speed(drone_id=DRONE_ID) >= 0.1 and not stop_event.is_set():
        time.sleep(0.05)
    
    start_time = time.time()
    while time.time() - start_time < 1.5 and not stop_event.is_set():
        time.sleep(0.05)

    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)

    x_fov, y_fov = fov
    
    if x_fov == None or y_fov == None:
        print("Config dosyasina fov degerleri giriniz")
        raise ValueError("Hatali config dosyasi")

    x_centered = adjust_x(orta_orani, x_fov, vehicle, DRONE_ID, target_cls, detected_obj, detected_obj_lock, stop_event)
    y_centered = adjust_y(orta_orani, y_fov, gimbal_channel, vehicle, DRONE_ID, target_cls, detected_obj, detected_obj_lock, stop_event)
    
    if not x_centered or not y_centered:
        print("Hedef eksenlerde ortalanamadi")
        return False, current_servo_pwm

    ilerleme_hizi = 0.5
    while not stop_event.is_set():
        with detected_obj_lock:
            obj = detected_obj["cls"]
            pos = detected_obj["pos"]
            screen_res = detected_obj["screen_res"]

        if obj == None or obj != target_cls:
            start_time = time.time()
            while not stop_event.is_set() and time.time() - start_time <= 0.2 and (obj == None or obj != target_cls):
                with detected_obj_lock:
                    obj = detected_obj["cls"]
                    pos = detected_obj["pos"]
                    screen_res = detected_obj["screen_res"]
                
                time.sleep(0.01)
        
        if obj == None or obj != target_cls:
            print("Hedef kayboldu")
            return False, current_servo_pwm
        

        x_dist, y_dist = camera_distance(pos, screen_res, orta_orani)

        print("Uzakliklar: ", x_dist, y_dist)

        if x_dist != 0:
            if adjust_x(orta_orani, x_fov, vehicle, DRONE_ID, target_cls, detected_obj, detected_obj_lock, stop_event) == False:
                print("Drone hedefi ortalamadi")
                return False, current_servo_pwm


        # Nesne ortalanmis ya da gerisinde kalmis ise
        if (y_dist == 0 or y_dist < 0) and (current_servo_pwm >= gimbal_angles[0] - 300 and current_servo_pwm <= gimbal_angles[0]):
            # Dronu ilerlemesini durdurma
            vehicle.move_drone_body((0,0,0), drone_id=DRONE_ID)

            while vehicle.get_speed(drone_id=DRONE_ID) >= 0.1 and not stop_event.is_set():
                time.sleep(0.05)
            
            start_time = time.time()
            while time.time() - start_time < 1 and not stop_event.is_set():
                time.sleep(0.05)

            vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)

            print("Drone hedef ustune getirildi")
            return True, current_servo_pwm
        
        elif y_dist != 0:
            if adjust_y(orta_orani, y_fov, gimbal_channel, vehicle, DRONE_ID, target_cls, detected_obj, detected_obj_lock, stop_event) == False:
                print("Gimbal hedefi ortalamadi")
                return False, current_servo_pwm
        
        print(f"{DRONE_ID}>> Drone {ilerleme_hizi} hizi ile ilerliyor...")
        vehicle.move_drone_body(rota=(ilerleme_hizi, 0, 0), drone_id=DRONE_ID)

        time.sleep(0.05)