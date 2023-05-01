#THRESHOLD = (5, 70, -23, 15, -57, 0) # Grayscale threshold for dark things...
THRESHOLD = (25, 75, -1, 77, 6, 62)
#THRESHOLD = (30, 100, 15, 127, 15, 127) # generic_red_thresholds
import sensor, image, time
from pyb import LED
import car
from pid import PID
rho_pid = PID(p=0.4, i=0)
theta_pid = PID(p=0.001, i=0)

roi1 = (20, 0, 40, 60)  # 第一个区域
roi2 = (0, 25, 80, 10)  # 第二个区域

LED(1).on()
LED(2).on()
LED(3).on()

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
#sensor.set_windowing([0,20,80,40])
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], robust = True ,roi = roi1)
    lineH = img.get_regression([(100,100)], robust = True,roi = roi2)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_rectangle(roi1,color = 127)
        img.draw_rectangle(roi2,color = (255,0,0))
        img.draw_line(line.line(), color = 127)
        #img.draw_line(lineH.line(), color = (255,0,0))
        # print(rho_err,line.magnitude(),rho_err)
        #print(rho_err,theta_err)

        if line.magnitude()>8: #像素长度8
            # Check for cross or T lines
            if lineH is not None:
                lineL = img.get_regression([(100,100)], robust = True ,roi = (20, lineH.y1() - 2, 40, 2))
            else:
                lineL = None
                print("no")
            #if line is not None and lineH is not None and abs(line.theta()-lineH.theta()) > 70 and lineH.magnitude() > 8 and abs((lineH.y1()+lineH.y2())/2 - line.y1()) > 5:
            if lineH is not None and lineL is not None and abs(line.theta()-lineH.theta()) > 70 and lineH.magnitude() > 8:
                img.draw_line(lineH.line(), color = (255,0,0))
                print("Found cross line!")
                print(line.y1(),lineH.y1())
            #if line is not None and lineH is not None and abs(line.theta()-lineH.theta()) > 70 and lineH.magnitude() > 8 and abs((lineH.y1()+lineH.y2())/2 - line.y1()) < 3:
            elif lineH is not None and lineL is None and abs(line.theta()-lineH.theta()) > 70 and lineH.magnitude() > 8:
                print("Found T line!")
                print(line.y1(),lineH.y1())
            else:
                rho_output = rho_pid.get_pid(rho_err,1)
                theta_output = theta_pid.get_pid(theta_err,1)
                output = rho_output+theta_output
                car.run(50+output, 50-output)
        else:
            car.run(0,0)
    else:
        car.run(50,-50)
        pass
    #print(clock.fps())
