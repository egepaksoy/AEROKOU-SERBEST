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

def fail(vehicle):
    while not stop_event.is_set():
        print(vehicle.error_messages())
        time.sleep(0.05)

#? Gerekliler
stop_event = threading.Event()

vehicle = Vehicle("COM16")
DRONE_ID = 4

try:
    # Drone'dan goruntu isleme
    threading.Thread(target=fail, args=(vehicle, ), daemon=True).start()
    input(f"{DRONE_ID} arm edilcek ENTER")

    # Ucus hazirligi
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    time.sleep(0.5)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    time.sleep(3)
    vehicle.arm_disarm(arm=False, drone_id=DRONE_ID)

    vehicle.set_mode(mode="POSHOLD", drone_id=DRONE_ID)

    stop_event.set()

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


