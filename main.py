# THRESHOLD = (5, 70, -23, 15, -57, 0) # Grayscale threshold for dark things...
THRESHOLD = (10, 77, 15, 60, -20, 60)
# THRESHOLD = (30, 100, 15, 127, 15, 127) # generic_red_thresholds
import sensor, image, time
from pyb import LED
import car
from pid import PID

rho_pid = PID(p=0.4, i=0)
theta_pid = PID(p=0.001, i=0)

roi1 = (0, 0, 80, 60)  # 第一个区域
roi2 = (0, 10, 80, 20)  # 第二个区域

LED(1).on()
LED(2).on()
LED(3).on()

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
# sensor.set_windowing([0,20,80,40])
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
sensor.skip_frames(time=100)  # WARNING: If you use QQVGA it may take seconds
clock = time.clock()  # to process a frame sometimes.

while True:
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    cross_line = 0
    # 使用img.find_lines()方法检测线段
    lines = img.find_lines(threshold=10000, theta_margin=30, rho_margin=10)

    if lines:
        # 获取最长的线段
        max_len = 0
        for l in lines:
            img.draw_line(l.line(),color = (255,0,0))
            if l.length() > max_len:
                max_len = l.length()
                max_line = l
            if abs(l.theta()) > 70:
                cross_line = 1
        rho_err = abs(max_line.rho()) - img.width() / 2
        if max_line.theta() > 90:
            theta_err = max_line.theta() - 180
        else:
            theta_err = max_line.theta()
        img.draw_rectangle(roi1, color=127)
        img.draw_rectangle(roi2, color=(255, 0, 0))
        img.draw_line(max_line.line(), color=127)

        if max_len > 80:  # 像素长度8
            # 检测十字线
            if cross_line:
                print("Found cross line!")
            else:
                print("no")
                rho_output = rho_pid.get_pid(rho_err, 1)
                theta_output = theta_pid.get_pid(theta_err, 1)
                output = rho_output + theta_output
                car.run(50 + output, 50 - output)
        else:
            car.run(0, 0)
    else:
        car.run(50, -50)
        pass
    # print(clock.fps())
