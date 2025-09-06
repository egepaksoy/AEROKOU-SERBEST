import sys
sys.path.append("./libs/")

from pymavlink_custom.pymavlink_custom import Vehicle

if len(sys.argv) == 4:
    conn_port = sys.argv[1]
    DRONE_ID = sys.argv[2]
    servo_channel = sys.argv[3]
else:
    raise ValueError("Kullanım: python gimbal_pwm_tester.py <baglanti_adresi> <drone_id> <servo_channel>")

vehicle = Vehicle(conn_port)

try:
    while True:
        pwm = input("Servo pwm girin cikis icin ENTER")
        if pwm == None or pwm == "":
            break
        vehicle.set_servo(channel=servo_channel, pwm=int(pwm), drone_id=DRONE_ID)

except KeyboardInterrupt:
    print("Exiting...")

except Exception as e:
    print(e)

finally:
    vehicle.vehicle.close()

