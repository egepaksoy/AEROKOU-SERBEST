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

vehicle = Vehicle(conn_port)

try:
    vehicle.set_servo(channel=13, pwm=1700, drone_id=3)
    time.sleep(2)
    vehicle.set_servo(channel=13, pwm=1200, drone_id=3)
    time.sleep(2)

except KeyboardInterrupt:
    print("Exiting...")

except Exception as e:
    print(e)

finally:
    vehicle.vehicle.close()

