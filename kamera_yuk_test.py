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

print("YKI Başlatılması Bekleniyor")
while not stop_event.is_set():
    if yki_monitor.get_system_status():
        break
    time.sleep(0.5)

print("YKI Baslatildi")
selecter_thrd.start()

try:
    for drone_id in vehicle.drone_ids:
        for dron_conf in config["DRONES"]:
            if dron_conf["id"] == drone_id and dron_conf["miss"] != "gozlemci":
                for pwm in dron_conf["GIMBAL"]["pwms"]:
                    vehicle.set_servo(channel=dron_conf["GIMBAL"]["channel"], pwm=pwm, drone_id=drone_id)
                    
                    start_time = time.time()
                    while time.time() - start_time < 1:
                        time.sleep(0.05)
                
                vehicle.set_servo(channel=dron_conf["YUK"]["channel"], pwm=dron_conf["YUK"]["acik"], drone_id=drone_id)
                start_time = time.time()
                while time.time() - start_time < 2:
                    time.sleep(0.05)
                vehicle.set_servo(channel=dron_conf["YUK"]["channel"], pwm=dron_conf["YUK"]["kapali"], drone_id=drone_id)

    input("Kontrol tamamlandı kapatmak için ENTER")

except KeyboardInterrupt:
    print("Exiting...")
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    if not stop_event.is_set():
        stop_event.set()
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
    vehicle.vehicle.close()
