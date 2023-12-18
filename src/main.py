import pigpio
from time import sleep, time
from detect import detect
from err_calc import err_calc
from pid import PID
import cv2 as cv

# Pin constant (In BCM)
L_PWM = 6
R_PWM = 12
IN_1 = 11
IN_2 = 9
IN_3 = 8
IN_4 = 25

# Algorithm constant
# These constants works in volt 4.11+4.13+4.15 (3S)
thresh = 92      # thresh boundary, 0-255
base_speed = 200  # base speed, 0-255
kp = 8           # Kp in pid
ki = 0.08            # Ki in pid
kd = 0.8         # Kd in pid
baseline = 450    # scanline of image, based on your camera
max_err = 120      # maximum error value
max_pid = 365     # maximum pid value
stop_line_baseline = 320   # scanline of stopline, based on your camera
'''
thresh = 98      # thresh boundary, 0-255
base_speed = 208  # base speed, 0-255
kp = 8.0           # Kp in pid
ki = 0.08            # Ki in pid
kd = 0.8        # Kd in pid
baseline = 450    # scanline of image, based on your camera
max_err = 135      # maximum error value
max_pid = 380     # maximum pid value
'''
# Variable
stop_line_count = 0
stop_line_update_time = None
fast_turn = False   # fast turn flag
fturn_start_time = 0 # fast turn start time

# Initalize
pi = pigpio.pi()

while not pi.connected:
    print("Can't access GPIO, check your pigpio service. Retry in 3 sec...")
    sleep(3)
    pi = pigpio.pi()

pi.set_mode(IN_1, pigpio.OUTPUT)
pi.set_mode(IN_2, pigpio.OUTPUT)
pi.set_mode(IN_3, pigpio.OUTPUT)
pi.set_mode(IN_4, pigpio.OUTPUT)
pi.set_mode(L_PWM, pigpio.OUTPUT)
pi.set_mode(R_PWM, pigpio.OUTPUT)

pi.write(IN_1, 1)
pi.write(IN_2, 0)
pi.write(IN_3, 1)
pi.write(IN_4, 0)
pi.set_PWM_frequency(L_PWM, 8000)
# pi.set_PWM_dutycycle(L_PWM, 255)
pi.set_PWM_frequency(R_PWM, 8000)
# pi.set_PWM_dutycycle(R_PWM, 255)

pid = PID(kp, ki, kd, max_pid) # PID object init

cam = cv.VideoCapture(0) # Camera init
cam.set(cv.CAP_PROP_FPS, 60)


# Function definition
def set_l_duty_cycle(dc: float) -> None:
    if dc < 0:
        pi.write(IN_1, 0)
        pi.write(IN_2, 1)
    else:
        pi.write(IN_1, 1)
        pi.write(IN_2, 0)
    pi.set_PWM_dutycycle(L_PWM, abs(dc))

def set_r_duty_cycle(dc: float) -> None:
    if dc < 0:
        pi.write(IN_3, 0)
        pi.write(IN_4, 1)
    else:
        pi.write(IN_3, 1)
        pi.write(IN_4, 0)
    pi.set_PWM_dutycycle(R_PWM, abs(dc))

def stop_car():
    #pi.set_PWM_dutycycle(L_PWM, -150)
    #pi.set_PWM_dutycycle(R_PWM, -150)
    pi.write(IN_1, 0)
    pi.write(IN_2, 0)
    pi.write(IN_3, 0)
    pi.write(IN_4, 0)
    pi.set_PWM_dutycycle(L_PWM, 0)
    pi.set_PWM_dutycycle(R_PWM, 0)
    pi.stop()
    exit()

# Main
for i in range(0, 5): cam.read()
input("Press enter to launch.")

pid.update_time()
try: 
    while True:
        tnow = time()
        ret, frame = cam.read()
        if not ret:
            print("Camera error...")
            continue

        img = cv.GaussianBlur(frame, (5, 5), 1)
        img = cv.cvtColor(img ,cv.COLOR_RGB2GRAY)
        ret, img = cv.threshold(img, thresh, 255, cv.THRESH_BINARY)

        if not ret:
            print("convert error...")
            continue
    
        err = err_calc(img, baseline, max_err)
        flag = detect(img, 120)

        if (err > 14.5) and not fast_turn and not flag:
            print("fast turn on.")
            fast_turn = True
            fturn_start_time = tnow
            # pid.update_max(430)

        if fast_turn and (tnow - fturn_start_time < 0.5): pid.update(err)
        else:
            pid.update(err)
            fast_turn = False
            # pid.update_max(300)

        p = pid.get_output()

        # Stopline detection
        if detect(img, stop_line_baseline, True):
            stop_line_count += 1
            stop_line_update_time = tnow
        elif stop_line_update_time != None:
            if tnow - stop_line_update_time > 1: stop_line_count = 0
        if stop_line_count >= 3: stop_car()

        print("err = %f\npid = %f" % (err, p))
        print("fast turn" if fast_turn else '', end=None)
        print("\nl = %f\nr = %f" % (base_speed if (p > 0 and not fast_turn) else base_speed + p if (not fast_turn) else 120, base_speed if p < 0 else base_speed - p))
        
        set_l_duty_cycle(base_speed if (p > 0 and not fast_turn) else base_speed + p if (not fast_turn) else 208)
        set_r_duty_cycle(base_speed if (p < 0 and not fast_turn) else base_speed - p if (not fast_turn) else -208)

except KeyboardInterrupt:
    print("\nStopped by user.")

except SystemExit:
    print("Stopline detected.")
        
except Exception as e:
    raise

finally:
    pi.write(IN_1, 0)
    pi.write(IN_2, 0)
    pi.write(IN_3, 0)
    pi.write(IN_4, 0)
    pi.set_PWM_dutycycle(L_PWM, 0)
    pi.set_PWM_dutycycle(R_PWM, 0)
    pi.stop()