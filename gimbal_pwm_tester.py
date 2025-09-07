import sys
import time
import json
sys.path.append("./libs/")

from pymavlink_custom.pymavlink_custom import Vehicle

#? Gerekliler
conf_file = "./config.json"
if len(sys.argv) == 2:
    conf_file = "./config-test.json"
config = json.load(open(conf_file))

if len(sys.argv) == 2:
    conn_port = sys.argv[1]

else:
    raise ValueError("Kullanım: python gimbal_pwm_tester.py <baglanti_adresi>")

vehicle = Vehicle(address=conn_port, on_flight=False)

angles = [2000, 1700, 1500, 1200, 1000]
channel = 14

#angles = [1800, 1300]
drone_id = 4
#channel = 13

#angles = [1700, 1200]
#drone_id = 3
#channel = 13

try:
    for angle in angles:
        vehicle.set_servo(channel=channel, pwm=angle, drone_id=drone_id)
        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting...")

except Exception as e:
    print(e)

finally:
    vehicle.vehicle.close()

