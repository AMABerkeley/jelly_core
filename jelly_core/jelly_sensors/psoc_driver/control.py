import serial


ser = serial.Serial("/dev/ttyACM0", 115200);

def pwm_input(pwm_value):
    try:
        print("you typed " + pwm_value)
        ser.write(pwm_value)
    except:
        print("ERRORRRRR")

if __name__ == "__main__":
    while True:
        val = raw_input("pwm cmp width: ")
        pwm_input(val)
