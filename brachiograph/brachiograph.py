# coding=utf-8

from time import sleep
import readchar
import math
import numpy
import json
import tqdm

# Section to control with PCA9685
print('Servos controlled with PCA9685')
import board
import busio
import adafruit_pca9685
import adafruit_motor.servo

class BrachioGraph:

    def __init__(
        self,
        inner_arm,                  # the lengths of the arms
        outer_arm,
        wait=None,
        bounds=None,                # the maximum rectangular drawing area
        angle_up=90,
        angle_down=100,
    ):

        # set the pantograph geometry
        self.INNER_ARM = inner_arm
        self.OUTER_ARM = outer_arm

        # the box bounds describe a rectangle that we can safely draw in
        self.bounds = bounds
        
        self.angle_up = angle_up
        self.angle_down = angle_down

        # i2c handle
        self.i2c_pca9685 = busio.I2C(board.SCL, board.SDA)
        # Setup servos controlled with pca9685
        self.pca = adafruit_pca9685.PCA9685(self.i2c_pca9685)
        self.pca.frequency = 50
        # Setup servo channel
        self.shoulder_servo_channel = self.pca.channels[0]
        self.elbow_servo_channel = self.pca.channels[1]
        self.pen_servo_channel = self.pca.channels[2]
        # User servo to control in angle degree instead of pwm
        self.servo_shoulder = adafruit_motor.servo.Servo(self.shoulder_servo_channel)
        self.servo_elbow = adafruit_motor.servo.Servo(self.elbow_servo_channel)
        self.servo_pen = adafruit_motor.servo.Servo(self.pen_servo_channel)
        # Default init
        self.servo_shoulder.set_pulse_width_range(500,2650)
        self.servo_elbow.set_pulse_width_range(500,2650)
        self.servo_pen.set_pulse_width_range(1000,2000) # Need not the be accurate

        # create the pen object, and make sure the pen is up
        self.pen = PenAngle(bg=self,angle_up=self.angle_up, angle_down=self.angle_down)

        # Initialise the pantograph with the motors in the centre of their travel
        self.servo_shoulder.angle = 90
        self.current_x = self.OUTER_ARM
        sleep(0.3)
        self.servo_elbow.angle = 90
        self.current_y = self.INNER_ARM
        sleep(0.3)

        # by default we use a wait factor of 0.1 for accuracy
        self.wait = wait or .1

        # Now the plotter is in a safe physical state.


    # methods in this class:
    # drawing
    # line-processing
    # test patterns
    # pen-moving methods
    # angles-to-pulse-widths
    # hardware-related
    # trigonometric methods
    # manual driving methods
    # reporting methods


    # ----------------- drawing methods -----------------
    def plot_file(self, filename="", wait=0, interpolate=10, bounds=None):
        wait = wait or self.wait
        bounds = bounds or self.bounds
        
        print(bounds)

        if not bounds:
            return "File plotting is only possible when BrachioGraph.bounds is set."

        with open(filename, "r") as line_file:
            lines = json.load(line_file)

        self.plot_lines(lines=lines, wait=wait, interpolate=interpolate, bounds=bounds, flip=False)


    def plot_lines(self, lines=[], wait=0, interpolate=10, rotate=False, flip=False, bounds=None):
        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Line plotting is only possible when BrachioGraph.bounds is set."

        lines = self.rotate_and_scale_lines(lines=lines, bounds=bounds, flip=False)

        for line in tqdm.tqdm(lines, desc="Lines", leave=False):
            x, y = line[0]

            # only if we are not within 1mm of the start of the line, lift pen and go there
            if (round(self.current_x, 1), round(self.current_y, 1)) != (round(x, 1), round(y, 1)):
                self.xy(x, y, wait=wait, interpolate=interpolate)

            for point in tqdm.tqdm(line[1:], desc="Segments", leave=False):
                x, y = point
                self.draw(x, y, wait=wait, interpolate=interpolate)

        self.park()


    def draw_line(self, start=(0, 0), end=(0, 0), wait=0, interpolate=10, both=False):
        wait = wait or self.wait

        start_x, start_y = start
        end_x, end_y = end

        self.pen.up()
        self.xy(x=start_x, y=start_y, wait=wait, interpolate=interpolate)

        self.pen.down()
        self.draw(x=end_x, y=end_y, wait=wait, interpolate=interpolate)

        if both:
            self.draw(x=start_x, y=start_y, wait=wait, interpolate=interpolate)

        self.pen.up()


    def draw(self, x=0, y=0, wait=0, interpolate=10):
        wait = wait or self.wait

        self.xy(x=x, y=y, wait=wait, interpolate=interpolate, draw=True)


    # ----------------- line-processing methods -----------------
    def rotate_and_scale_lines(self, lines=[], rotate=False, flip=False, bounds=None):
        rotate, x_mid_point, y_mid_point, box_x_mid_point, box_y_mid_point, divider = self.analyse_lines(
            lines=lines, rotate=rotate, bounds=bounds
        )

        for line in lines:
            for point in line:
                if rotate:
                    point[0], point[1] = point[1], point[0]

                x = point[0]
                x = x - x_mid_point         # shift x values so that they have zero as their mid-point
                x = x / divider             # scale x values to fit in our box width
                x = x + box_x_mid_point     # shift x values so that they have the box x midpoint as their endpoint

                if flip ^ rotate:
                    x = -x

                y = point[1]
                y = y - y_mid_point
                y = y / divider
                y = y + box_y_mid_point

                point[0], point[1] = x, y

        return lines


    def analyse_lines(self, lines=[], rotate=False, bounds=None):
        # lines is a tuple itself containing a number of tuples, each of which contains a number of 2-tuples
        #
        # [                                                                                     # |
        #     [                                                                                 # |
        #         [3, 4],                               # |                                     # |
        #         [2, 4],                               # |                                     # |
        #         [1, 5],  #  a single point in a line  # |  a list of points defining a line   # |
        #         [3, 5],                               # |                                     # |
        #         [3, 7],                               # |                                     # |
        #     ],                                                                                # |
        #     [                                                                                 # |  all the lines
        #         [...],                                                                        # |
        #         [...],                                                                        # |
        #     ],                                                                                # |
        #     [                                                                                 # |
        #         [...],                                                                        # |
        #         [...],                                                                        # |
        #     ],                                                                                # |
        # ]                                                                                     # |

        # First, we create a pair of empty sets for all the x and y values in all of the lines of the plot data.
        x_values_in_lines = set()
        y_values_in_lines = set()

        # Loop over each line and all the points in each line, to get sets of all the x and y values:

        for line in lines:
            x_values_in_line, y_values_in_line = zip(*line)

            x_values_in_lines.update(x_values_in_line)
            y_values_in_lines.update(y_values_in_line)

        # Identify the minimum and maximum values.
        min_x, max_x = min(x_values_in_lines), max(x_values_in_lines)
        min_y, max_y = min(y_values_in_lines), max(y_values_in_lines)

        # Identify the range they span.
        x_range, y_range = max_x - min_x, max_y - min_y
        box_x_range, box_y_range = bounds[2] - bounds[0], bounds[3] - bounds[1]

        # And their mid-points.
        x_mid_point, y_mid_point = (max_x + min_x) / 2, (max_y + min_y) / 2
        box_x_mid_point, box_y_mid_point = (bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2

        # Get a 'divider' value for each range - the value by which we must divide all x and y so that they will
        # fit safely inside the drawing range of the plotter.

        # If both image and box are in portrait orientation, or both in landscape, we don't need to rotate the plot.
        if (x_range >= y_range and box_x_range >= box_y_range) or (x_range <= y_range and box_x_range <= box_y_range):
            divider = max((x_range / box_x_range), (y_range / box_y_range))
            rotate = False
        else:
            divider = max((x_range / box_y_range), (y_range / box_x_range))
            rotate = True
            x_mid_point, y_mid_point = y_mid_point, x_mid_point
        return rotate, x_mid_point, y_mid_point, box_x_mid_point, box_y_mid_point, divider


    # ----------------- test pattern methods -----------------
    def test_pattern(self, bounds=None, wait=0, interpolate=10, repeat=1):
        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when BrachioGraph.bounds is set."

        for r in tqdm.tqdm(tqdm.trange(repeat, desc='Iteration'), leave=False):
            for y in range(bounds[1], bounds[3], 2):
                self.xy(bounds[0],   y,     wait, interpolate)
                self.draw(bounds[2], y,     wait, interpolate)
                self.xy(bounds[2],   y + 1, wait, interpolate)
                self.draw(bounds[0], y + 1, wait, interpolate)

        self.park()


    def vertical_lines(self, bounds=None, lines=4, wait=0, interpolate=10, repeat=1, reverse=False, both=False):
        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when BrachioGraph.bounds is set."

        if not reverse:
            top_y =    self.bounds[1]
            bottom_y = self.bounds[3]
        else:
            bottom_y = self.bounds[1]
            top_y =    self.bounds[3]

        step = (self.bounds[2] - self.bounds[0]) /  lines
        x = self.bounds[0]
        while x <= self.bounds[2]:
            self.draw_line((x, top_y), (x, bottom_y), interpolate=interpolate, both=both)
            x = x + step

        self.park()


    def horizontal_lines(self, bounds=None, lines=4, wait=0, interpolate=10, repeat=1, reverse=False, both=False):
        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when BrachioGraph.bounds is set."

        if not reverse:
            min_x = self.bounds[0]
            max_x = self.bounds[2]
        else:
            max_x = self.bounds[0]
            min_x = self.bounds[2]

        step = (self.bounds[3] - self.bounds[1]) / lines
        y = self.bounds[1]
        while y <= self.bounds[3]:
            self.draw_line((min_x, y), (max_x, y), interpolate=interpolate, both=both)
            y = y + step

        self.park()


    def grid_lines(self, bounds=None, lines=4, wait=0, interpolate=10, repeat=1, reverse=False, both=False):
        self.vertical_lines(
            bounds=bounds, lines=lines, wait=wait, interpolate=interpolate, repeat=repeat, reverse=reverse, both=both
            )
        self.horizontal_lines(
            bounds=bounds, lines=lines, wait=wait, interpolate=interpolate, repeat=repeat, reverse=reverse, both=both
            )


    def box(self, bounds=None, wait=0, interpolate=10, repeat=1, reverse=False):
        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Box drawing is only possible when BrachioGraph.bounds is set."

        self.xy(bounds[0], bounds[1], wait, interpolate)

        for r in tqdm.tqdm(tqdm.trange(repeat), desc='Iteration', leave=False):
            if not reverse:
                self.draw(bounds[2], bounds[1], wait, interpolate)
                self.draw(bounds[2], bounds[3], wait, interpolate)
                self.draw(bounds[0], bounds[3], wait, interpolate)
                self.draw(bounds[0], bounds[1], wait, interpolate)
            else:
                self.draw(bounds[0], bounds[3], wait, interpolate)
                self.draw(bounds[2], bounds[3], wait, interpolate)
                self.draw(bounds[2], bounds[1], wait, interpolate)
                self.draw(bounds[0], bounds[1], wait, interpolate)

        self.park()


    # ----------------- pen-moving methods -----------------
    def xy(self, x=0, y=0, wait=0, interpolate=10, draw=False):
        # Moves the pen to the xy position; optionally draws
        wait = wait or self.wait
        
        if draw:
            self.pen.down()
        else:
            self.pen.up()

        # Calculate the angle to set the point P(x,y)
        (angle_1, angle_2) = self.xy_to_angles(x, y)
        angle_1 = round(angle_1)
        angle_2 = round(angle_2)
#         print("target x :", x, " target y: ", y)
#         print("target shoulder angle: ",angle_1, " target elbow angle: ", angle_2)

        # Check out of bounds and do nothing
        if (x < self.bounds[0] or x > self.bounds[2] 
            or y < self.bounds[1] or y > self.bounds[3]):
            print("xy plot out of bounds")
            return

        # if they are the same, we don't need to move anything
        if (angle_1, angle_2) == (round(self.servo_shoulder.angle), round(self.servo_elbow.angle)):
            # ensure the pantograph knows its x/y positions
            self.current_x = x
            self.current_y = y
            return

        # we assume the pantograph knows its x/y positions - if not, there could be
        # a sudden movement later

        # calculate how many steps we need for this move, and the x/y length of each
        (x_length, y_length) = (x - self.current_x, y - self.current_y)

        length = math.sqrt(x_length ** 2 + y_length **2)

        no_of_steps = int(length * interpolate) or 1

        if no_of_steps < 100:
            disable_tqdm = True
        else:
            disable_tqdm = False

        (length_of_step_x, length_of_step_y) = (x_length/no_of_steps, y_length/no_of_steps)

        for step in tqdm.tqdm(range(no_of_steps), desc='Interpolation', leave=False, disable=disable_tqdm):
            self.current_x = self.current_x + length_of_step_x
            self.current_y = self.current_y + length_of_step_y

            angle_1, angle_2 = self.xy_to_angles(self.current_x, self.current_y)
            
            # Here we follow the trajectoy with stepping through the angles, drawing the line
            self.set_angles(angle_1, angle_2)

            if step + 1 < no_of_steps:
                sleep(length * wait/no_of_steps)

        sleep(length * wait/10)


    # Todo: This function I need to rework to be able to use the PCA9685
    # calculate based on the angle the on off times
    def set_angles(self, angle_1=0, angle_2=0):
        # moves the servo motor
        self.servo_shoulder.angle = angle_1
        self.servo_elbow.angle = angle_2


    def park(self):
        # parks the plotter
        self.pen.up()
        self.xy(self.INNER_ARM, self.OUTER_ARM)
        sleep(1)
        # self.quiet()

    # ----------------- trigonometric methods -----------------
    # Every x/y position of the plotter corresponds to a pair of angles of the arms. These methods
    # calculate:
    #
    # the angles required to reach any x/y position
    # the x/y position represented by any pair of angles
    def xy_to_angles(self, x=0, y=0):
        # convert x/y co-ordinates into motor angles
        # Get the hypothenuse directly from the P(x,y)
        hypotenuse = math.sqrt(x**2+y**2)

        # Check if the point could be reached
        if hypotenuse > self.INNER_ARM + self.OUTER_ARM:
            raise Exception(f"Cannot reach {hypotenuse}; total arm length is {self.INNER_ARM + self.OUTER_ARM}")
        
        # Point can be reached so calculate the angle for the hypothenuse
        if(y < 0):
            hypotenuse_angle = -1 * math.asin(x/hypotenuse) + math.pi    
        else:
            hypotenuse_angle = math.asin(x/hypotenuse)
            
        inner_angle = math.acos((hypotenuse**2 + self.INNER_ARM**2 - self.OUTER_ARM**2)
                                / (2 * hypotenuse * self.INNER_ARM))       


        # Here I added 90 degree as the original start was 0 which need to be
        # for the pca9685 a 90 degree angle
        shoulder_motor_angle_deg = 90 + math.degrees(inner_angle - hypotenuse_angle)
        
        outer_angle = math.acos((self.INNER_ARM**2 + self.OUTER_ARM**2 - hypotenuse**2)
                                / (2 * self.INNER_ARM * self.OUTER_ARM))

        elbow_motor_angle_deg = math.degrees(math.pi - outer_angle)
        
#         # Debug out
#         print(hypotenuse)
#         print(math.degrees(hypotenuse_angle))
#         print(math.degrees(inner_angle))
#         print(math.degrees(outer_angle))        

        return (shoulder_motor_angle_deg, elbow_motor_angle_deg)


    # ----------------- manual driving methods -----------------
    def drive(self):
        # adjust the pulse-widths using the keyboard

        angle_1 = 90
        angle_2 = 90

        self.servo_shoulder.angle = angle_1
        self.servo_elbow.angle = angle_2
        
        while True:
            key = readchar.readchar()

            if key == "0":
                return
            elif key=="a":
                angle_1 = angle_1 - 1
            elif key=="s":
                angle_1 = angle_1 + 1

            elif key=="k":
                angle_2 = angle_2 - 1
            elif key=="l":
                angle_2 = angle_2 + 1


            print(angle_1, angle_2)

            self.servo_shoulder.angle = angle_1
            self.servo_elbow.angle = angle_2


    def drive_xy(self):

        # move the pen up/down and left/right using the keyboard

        while True:
            key = readchar.readchar()

            if key == "0":
                return
            elif key=="a":
                self.current_x = self.current_x - 1
            elif key=="s":
                self.current_x = self.current_x + 1
            elif key=="A":
                self.current_x = self.current_x - .1
            elif key=="S":
                self.current_x = self.current_x + .1
            elif key=="k":
                self.current_y = self.current_y - 1
            elif key=="l":
                self.current_y = self.current_y + 1
            elif key=="K":
                self.current_y = self.current_y - .1
            elif key=="L":
                self.current_y = self.current_y + .1

            print(self.current_x, self.current_y)

            self.xy(self.current_x, self.current_y)

    @property
    def bl(self):
        return (self.bounds[0], self.bounds[1])

    @property
    def tl(self):
        return (self.bounds[0], self.bounds[3])

    @property
    def tr(self):
        return (self.bounds[2], self.bounds[3])

    @property
    def br(self):
        return (self.bounds[2], self.bounds[1])


class PenAngle:

    def __init__(self, bg, angle_up=80, angle_down=100):
        self.bg = bg
        self.angle_up = angle_up
        self.angle_down = angle_down
        
        self.up()
        sleep(0.3)
        self.down()
        sleep(0.3)
        self.up()
        sleep(0.3)
        
    def down(self):
        self.bg.servo_pen.angle = self.angle_down

    def up(self):
        self.bg.servo_pen.angle = self.angle_up

    def calibrate(self):

        print(f"Calibrating the pen-lifting servo.")
        print(f"See https://brachiograph.art/how-to/calibrate.html")

        angle1 = self.angle_down
        angle2 = self.angle_up

        while True:
            key = readchar.readchar()

            if key == "0":
                break
            elif key=="a":
                angle1 = angle1 - 1
                self.bg.servo_pen.angle = angle1
                continue
            elif key=="s":
                angle1 = angle1 + 1
                self.bg.servo_pen.angle = angle1               
                continue
            elif key=="k":
                angle2 = angle2 - 1
                self.bg.servo_pen.angle = angle2              
                continue
            elif key=="l":
                angle2 = angle2 + 1
                self.bg.servo_pen.angle = angle2              
                continue

            elif key=="u":
                self.angle_down = angle1
            elif key=="d":
                self.angle_up = angle2
            else:
                continue

            print(f"Pen-up pulse-width: {self.angle_up}deg, pen-down pulse-width: {self.angle_down}deg")

        print()
        print("Use these values in your BrachioGraph definition:")
        print()
        print(f"pen_up={self.angle_up}, pen_down={self.angle_down}")
        
