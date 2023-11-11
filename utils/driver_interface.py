import serial
import json
import time


def main(ser):
    dir = 0.01
    spd = 0.0
    last = time.time()
    while True:

        if time.time() - last > 0.2:
            if spd > 0.25 or spd < -0.25:
                dir = -dir
            spd += dir
            last = time.time()

        command = f"R {spd} L {-spd} E"
        ser.write(command.encode('utf-8'))

        ser.reset_input_buffer()
        t = ser.readline()[:-2]
        try:
            out = json.loads(t)
            print(out)
        except:
            pass
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        ser = serial.Serial("/dev/cu.usbmodem1101")
        main(ser)
    except KeyboardInterrupt:
        print(" Closing serial port...")
        ser.close()
