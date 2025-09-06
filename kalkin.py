import threading
import time
import sys
sys.path.append("./libs/")

from pymavlink_custom.pymavlink_custom import Vehicle

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

#? Gerekliler
stop_event = threading.Event()

#? Uçuş hazırlıkları
ALT = 5

vehicle = Vehicle("com16")

try:
    # Drone'dan goruntu isleme
    input("Ucus baslatilmasi bekleniyor ENTER")

    # Ucus hazirligi
    vehicle.set_mode(mode="GUIDED")
    time.sleep(0.5)
    vehicle.arm_disarm(arm=True)
    time.sleep(0.5)
    vehicle.takeoff(alt=ALT)
    
    home_pos = vehicle.get_pos()
    print(f"takeoff yaptı")

    time.sleep(3)

    vehicle.set_mode(mode="LAND")
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


