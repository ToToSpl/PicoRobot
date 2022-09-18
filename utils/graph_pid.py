import matplotlib.pyplot as plt
import matplotlib.animation as anim
import serial


def main(ser):

    targets = []
    values = []
    # errors = []
    steps = []

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    def update(i):
        t = ser.readline()
        nums = t.split()
        if len(nums) != 3:
            return

        targets.append(float(nums[0]))
        values.append(float(nums[1]))
        # errors.append(float(nums[2]))

        if len(steps) >= 40:
            targets.pop(0)
            values.pop(0)
            # errors.pop(0)
            steps.pop(0)

        ax.clear()
        steps.append(i)
        ax.plot(steps, targets)
        ax.plot(steps, values)
        # ax.plot(steps, errors)
        print(nums[0], nums[1], nums[2])
        ser.flushInput()

    a = anim.FuncAnimation(fig, update, interval=1)
    plt.show()


if __name__ == "__main__":
    try:
        ser = serial.Serial("/dev/cu.usbmodem1101")
        main(ser)
    except KeyboardInterrupt:
        ser.close()
