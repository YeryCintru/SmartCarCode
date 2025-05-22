"""
SmartCar class for WIP Smart Systems lecture
version 0.6:
Video stream optimized with MJPG encoder (much faster now)
Lane detection optimized using gradient function of numpy (much faster now)
version 0.5:
Image size reduced to the rows to analyze to improve image conversion performance
version 0.4:
Time measurement functions were integrated to analyze performance of software parts
version 0.3:
Debugging functionality including modification of color thresholds was added
version 0.2:
Lane detection function was integrated from existing implementation 12.09.2022.
version 0.1:
Driving functions were integrated into the new class from existing implementation 12.09.2022.
"""
import picar
import cv2
import numpy as np
import time
import copy
import smbus # for doing analog voltage measurements on the robot hat 
from periphery import GPIO
#import RPi.GPIO as GPIO
#import paho.mqtt.client as mqtt
#import paho.mqtt.subscribe as subscribe
import socket
import struct
import pickle
import select
from enum import Enum

# Debug constants for displayed video
SC_DEBUG_NONE = 0 # No debug information in image
SC_DEBUG_MASK = 1 # Show the b/w mask image
SC_DEBUG_OVERLAY = 2 # Show the masked areas inside original image


class VoiceCommands(Enum):
    FORWARD = 0
    BACKWARD = 1
    STOP = 2
    FASTER = 3
    SLOWER = 4
    RIGHT = 5
    LEFT = 6

class SmartCar(object):
    # Constructor
    def __init__(self, drive_control = False,
                        steer_control = False,
                        camera_control = False,
                        recording_control = False,
                        use_ultrasonic = False,
                        kp = 0.15,
                        ki = 0.002,
                        kd = 0.002,
                        # original values
                        #upper_threshold = np.array([123, 255, 255]),
                        #lower_threshold = np.array([99, 23, 65]),
                        # home of Markus
                        #upper_threshold = np.array([117, 255, 255]),
                        #lower_threshold = np.array([105, 43, 76]),
                        # office Markus - both directions
                        upper_threshold = np.array([116, 219, 255]),
                        lower_threshold = np.array([105, 57, 81]),
                        debug_image = SC_DEBUG_NONE,
                        tune_thresholds = False,
                        use_local_window = True,
                        measure_performance = False,
                        use_socket = False):
        # GPIOs for ultrasonic
        self.gpio_p29_trigger = GPIO("/dev/gpiochip0", 7, "out")
        self.gpio_p31_echo = GPIO("/dev/gpiochip0", 8, "in")
        # Initialize PiCar
        picar.setup()
        # Member variables
        self.bw = picar.back_wheels.Back_Wheels()
        self.fw = picar.front_wheels.Front_Wheels()
        self.pan_servo = picar.Servo.Servo(1)
        self.tilt_servo = picar.Servo.Servo(2)
        self.speed = 0
        self.steer = 90
        self.campan = 85
        self.camtilt = 90
        self.showhelp = True
        self.recording = False
        self.textactivated = 0 # time in s when recording text was started to be displayed
        self.drive_control = drive_control
        self.steer_control = steer_control
        self.camera_control = camera_control
        self.recording_control = recording_control
        self.use_ultrasonic = use_ultrasonic
        self.distance = 255
        self.last_distance = 255
        self.i2c_analog_in = smbus.SMBus(1)
        self.vcc = 0
        self.quit = False
        self.p_part = 0  #pid calculated p part
        self.i_part = 0  #pid calculated i part
        self.d_part = 0  #pid calculated d part
        self.kp = kp     #pid kp value
        self.ki = ki     #pid ki value
        self.kd = kd     #pid kd value
        self.old_error = 0 # for pid
        self.old_time = time.time() # for pid
        self.lower_threshold = lower_threshold #lower lane mask values
        self.upper_threshold = upper_threshold #upper lane mask values
        self.debug_image = debug_image # Show special image for debugging
        self.tune_thresholds = tune_thresholds # With additional keys, the color thresholds can be tuned
        self.use_local_window = use_local_window # If False, the handle_window function will not show the local window and no keys are handled
        self.use_socket = use_socket # If True and use_local_window False, then video and kayboard inputs are tranferred via socket connection
        self.measure_performance = measure_performance # Measure time consumptions and write to file
        self.time_consumptions = {} # Dictionary containing time consumption lists
        self.x_left = -10000 # Initial last know left lane
        self.x_right = 10639 # Initial last known right lane
        # Create a VideoCapture object
        """
        The following resolutions are tested with the original pi cam:
        640x480 (standard)
        1920x1088 (like hd)
        The following resolutions are officially supported by the ELP camera of the smart systems lab:
        2592 (H) x 1944 (V) Pixel MJPEG 15 fps YUY2 3 fps
        2048 (H) x 1536 (V) Pixel MJPEG 15PS YUY2 3 fps
        1600 (H) x 1200 (V) Pixel MJPEG 15 fps YUY2 3 fps
        1920 (H) x 1080 (V) Pixel MJPEG 15 fps YUY2 3 fps
        1280 (H) x 1024 (V) Pixel MJPEG 15 fps YUY2 7.5 FPS
        1280 (H) x 720 (V) Pixeln MJPEG 30 fps YUY2 7.5 FPS
        1024 (H) x 768 (V) Pixeln MJPEG 30 fps YUY2 15 fps
        800 (H) x 600 (V) Pixeln MJPEG 30 fps YUY2 30 fps
        640 (H) x 480 (V) Pixel MJPEG 30 fps YUY2 30 fps
        """
        # Print additional information
        if tune_thresholds:
            print("You can use these keys to tune the color thresholds:")
            print("  Upper HSV threshold +/-")
            print("  7    8    9")
            print("  u    i    o")
            print("  Lower HSV threshold +/-")
            print("  j    k    l")
            print("  m    ,    .")
            print("Current color tresholds")
            print(f"{self.upper_threshold[0]} {self.upper_threshold[1]} {self.upper_threshold[2]}")
            print(f"{self.lower_threshold[0]} {self.lower_threshold[1]} {self.lower_threshold[2]}")
        # Create the video capture object
        self.cap = cv2.VideoCapture(1)
        #self.cap.set(3, 1024)  # Set horizontal resolution
        #self.cap.set(4, 768)  # Set vertical resolution
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        # Minimize the buffer size to get more up to date images
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)
        # Set the encoder to have higher performance (frame rate) while reading images
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        # Check if camera opened successfully
        if (self.cap.isOpened() == False): 
            print("Unable to read camera feed")
        # Default resolutions of the frame are obtained.The default resolutions are system dependent.
        # We convert the resolutions from float to integer.
        self.frame_width = int(self.cap.get(3))
        self.frame_height = int(self.cap.get(4))
        # Video writer initially None
        self.out = None
        # Default writer object
        # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
        # self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
        # self.out.release()
        # On demand, open the socket connection (server)
        if not self.use_local_window and self.use_socket:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.bind(('0.0.0.0', 5000))
            self.server_socket.listen(5)
            print("Server listening on 0.0.0.0:5000")
            print("Please start your client program on remote computer and connect to the SmartCar IP Address.")
            while True:
                self.conn, self.addr = self.server_socket.accept()
                print(f"Connection from: {self.addr}")
                #self.conn.settimeout(0.1)  # Prevent blocking - original value 0.1 which means seconds
                break

    # Destructor
    def __del__(self):
        print("Destructor called")
        # Write the time consumptions to a csv file
        if self.measure_performance:
            # Create the file
            now = time.time() # get snapshot of current time
            lt = time.localtime(now) # convert to local time struct
            filename = "time_consumption-" + time.strftime("%Y-%m-%d-%H-%M-%S", lt) + ".csv"
            time_consumption_file = open(filename, "w+")
            # Write the header
            firstkey = True
            for key in self.time_consumptions:
                if not firstkey:
                    time_consumption_file.write(";")
                firstkey = False
                time_consumption_file.write(key)
            time_consumption_file.write("\n")
            # Write the data lines
            line_index = 0
            data_found = True
            while data_found:
                data_found = False
                # Take data for each existing key
                firstkey = True
                for key in self.time_consumptions:
                    if not firstkey:
                        time_consumption_file.write(";")
                    firstkey = False
                    datastr2 = "end"
                    if len(self.time_consumptions[key]) > line_index:
                        data_found = True
                        datastr = str(self.time_consumptions[key][line_index])
                        datastr2 = datastr.replace(".", ",")
                    time_consumption_file.write(datastr2)
                time_consumption_file.write("\n")
                line_index += 1
            time_consumption_file.close()
        # When everything done, release the video capture and video write objects
        self.cap.release()
        if self.use_socket:
            self.conn.close()
        if self.out != None:
            self.out.release()
        # Closes all the frames
        cv2.destroyAllWindows()
        # Cleanup GPIO at the end
        #GPIO.cleanup()
        #reset the hardware
        self.bw.stop()
        self.fw.turn(90)
        self.pan_servo.write(90)
        self.tilt_servo.write(90)

    # Adds the help text to the frame
    # cv2frame: an image as a numpy array in which the text will be drawn
    # text: The text that shall be displayed (list of strings)
    def addhelp(self):
        # Define the help text depending on available controls
        text = ["Usage of the car control:",
                "show or hide this help: ?"]
        if self.drive_control or self.steer_control or self.camera_control:
            text.append("reset actuators: x")
        text.append("quit program: q")
        if self.drive_control:
            text.append("speed up: w")
            text.append("speed down: s")
        if self.steer_control:
            text.append("left/right: a/d")
        if self.camera_control:
            text.append("camera up: t")
            text.append("camera left/right: f/h")
            text.append("camera down: g")
        if self.recording_control:
            text.append("start or stop recording: c")
        if self.use_ultrasonic:
            text.append(f"us distance: {self.distance:.1f}cm")
        # Render the text into the frame
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,20)
        fontScale              = 0.5
        fontColor              = (255,100,100)
        lineType               = 2
        linepos = 0
        for line in text:
            bottomLeftCornerOfText = (10,20+linepos*20)
            cv2.putText(self.frame, line, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
            linepos = linepos + 1

    # Adds the "blinking" recording text in red color to a frame
    # Must be called cyclically to work
    # cv2frame: an image as a numpy array in which the text will be drawn
    def addrecording(self):
        # show text for 0.5s
        if self.textactivated > time.time()-0.5:
            font                   = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (10,self.frame_height-40)
            fontScale              = 0.5
            fontColor              = (0,0,255)
            lineType               = 2
            linepos = 0
            cv2.putText(self.frame, "recording", 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
        else:
            if self.textactivated < time.time()-1:
                self.textactivated = time.time()

    # Adds the PID controller values to the window
    def add_pid(self):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 0.5
        fontColor              = (255,100,100)
        lineType               = 2
        bottomLeftCornerOfText = (470,self.frame_height-100)
        cv2.putText(self.frame, "P: " + str(self.p_part), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
        bottomLeftCornerOfText = (470,self.frame_height-80)
        cv2.putText(self.frame, "I: " + str(self.i_part), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
        bottomLeftCornerOfText = (470,self.frame_height-60)
        cv2.putText(self.frame, "D: " + str(self.d_part), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)

    # Adds additional text to the window
    def add_text(self, text):
        if text != "":
            font                   = cv2.FONT_HERSHEY_SIMPLEX
            fontScale              = 0.5
            fontColor              = (255,100,100)
            lineType               = 2
            bottomLeftCornerOfText = (150,self.frame_height-40)
            cv2.putText(self.frame, text, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)

    # Measures the ultrasonic sensor distance
    # Stores it in self.distance
    # self.distance is backed up before in self.last_distance
    def measure_us_distance(self):
        # Store last measurement
        self.last_distance = self.distance
        # Initialize distance
        d = 255
        # The measurement is still valid
        validity = True
        # Set trigger to HIGH
        self.gpio_p29_trigger.write(True)
        # After 0.01ms set trigger to LOW
        time.sleep(0.00001)
        self.gpio_p29_trigger.write(False)
        begintime = time.time()
        starttime = begintime
        # Store start time
        while not self.gpio_p31_echo.read():
            starttime = time.time()
            if starttime - begintime > 500/34300:
                validity = False
                break
        if validity:
            # Store arrival time
            stoptime = time.time() + 500/34300
            while self.gpio_p31_echo.read():
                stoptime = time.time()
                if stoptime - starttime > 500/34300:
                    validity = False
                    break
            if validity:
                # Time difference
                elapsedtime = stoptime - starttime
                # Distance in cm (using sound speed 34300 cm/s)
                d = (elapsedtime * 34300) / 2
        # Store measurement
        self.distance = d

    def measure_vcc(self):
        # initialize measurement
        value = self.i2c_analog_in.read_byte_data(0x48, 0x40)
        # get converted value - 3 is because of voltage divider
        self.vcc = self.i2c_analog_in.read_byte_data(0x48, 0x40)/0xFF*3.3*3
    
    # Read camera stream and us sensor
    def handle_sensors(self):
        self.start_timer("handle_sensors.capture_image")
        self.cap.read()
        self.ret, self.frame = self.cap.read()
        self.stop_timer("handle_sensors.capture_image")
        self.start_timer("handle_sensors.measure_us_distance")
        self.measure_us_distance()
        self.stop_timer("handle_sensors.measure_us_distance")
        self.measure_vcc()
        
    # Show the window
    def handle_window(self, additional_text = ""):
        if self.ret == True:
            self.start_timer("handle_window.add_text")
            # Add the optional text to the image
            self.add_text(additional_text)
            self.stop_timer("handle_window.add_text")
            self.start_timer("handle_window.copy_frame")
            # copy the frame for showing text only in window and not in recording
            self.recordingframe = copy.copy(self.frame)
            self.stop_timer("handle_window.copy_frame")
            self.start_timer("handle_window.showhelp")
            # on demand, show the help text
            if self.showhelp:
                self.addhelp()
            self.stop_timer("handle_window.showhelp")
            self.start_timer("handle_window.addrecording")
            # on demand, show the recording text
            if self.recording:
                self.addrecording()
            self.stop_timer("handle_window.addrecording")
            self.start_timer("handle_window.imshow")
            # Display the resulting frame
            if self.use_local_window:
                cv2.imshow('SmartCar camera',self.frame)
            elif self.use_socket:
                _, buffer = cv2.imencode('.jpg', self.frame)
                data = pickle.dumps(buffer)
                size = struct.pack(">L", len(data))
                self.conn.sendall(size + data)  # Send frame
            self.stop_timer("handle_window.imshow")

    # Handle the user input
    def user_command(self):
        self.start_timer("user_command")
        key = None
        if self.use_local_window:
            # catch the key
            key = cv2.waitKey(1) & 0xFF
        elif self.use_socket:
            # Use select to check if data is available (non-blocking)
            readable, _, _ = select.select([self.conn], [], [], 0.1)
            if readable:
                try:
                    key = self.conn.recv(1).decode('utf-8')  # Read 1-byte key input
                    if key:
                        print(f"Received key: {key}")  # Process key input here
                        key = ord(key)
                except:
                    pass  # Ignore small errors
        # Press q on keyboard to stop program
        if key == ord('q'):
            self.quit = True
        # driving commands
        if self.drive_control:
            if key == ord('w'):
                self.speed = self.speed + 10
                print("speed:", self.speed)
            elif key == ord('s'):
                self.speed = self.speed - 10
                print("speed:", self.speed)
        # steering commands
        if self.steer_control:
            if key == ord('a'):
                self.steer = self.steer - 5
                if self.steer < 50:
                    self.steer = 50
                print("steer:", self.steer)
            elif key == ord('d'):
                self.steer = self.steer + 5
                if self.steer > 140:
                    self.steer = 140
                print("steer:", self.steer)
        # camera commands
        if self.camera_control:
            if key == ord('h'):
                self.campan = self.campan - 5
                if self.campan < 45:
                    self.campan = 45
                print("campan:", self.campan)
            elif key == ord('f'):
                self.campan = self.campan + 5
                if self.campan > 135:
                    self.campan = 135
                print("campan:", self.campan)
            elif key == ord('g'):
                self.camtilt = self.camtilt - 5
                if self.camtilt < 70:
                    self.camtilt = 70
                print("camtilt:", self.camtilt)
            elif key == ord('t'):
                self.camtilt = self.camtilt + 5
                if self.camtilt > 135:
                    self.camtilt = 135
                print("camtilt:", self.camtilt)
        if self.drive_control or self.steer_control or self.camera_control:
            if key == ord('x'):
                self.speed = 0
                self.steer = 90
                self.campan = 85
                self.camtilt = 90
        if key == ord('?'):
            self.showhelp = not self.showhelp
        if self.recording_control:
            if key == ord('c'):
                if self.recording:
                    if self.out != None:
                        self.out.release()
                else:
                    now = time.time() # get snapshot of current time
                    lt = time.localtime(now) # convert to local time struct
                    filename = time.strftime("%Y-%m-%d-%H-%M-%S", lt) + ".avi"
                    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
                    self.out = cv2.VideoWriter(filename,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.frame_width,self.frame_height))
                    textactivated = time.time()
                self.recording = not self.recording
        if self.tune_thresholds:
            print_it = False
            if key == ord('7'):
                self.upper_threshold[0] = self.upper_threshold[0] + 1
                print_it = True
            elif key == ord('u'):
                self.upper_threshold[0] = self.upper_threshold[0] - 1
                print_it = True
            elif key == ord('j'):
                self.lower_threshold[0] = self.lower_threshold[0] + 1
                print_it = True
            elif key == ord('m'):
                self.lower_threshold[0] = self.lower_threshold[0] - 1
                print_it = True
            elif key == ord('8'):
                self.upper_threshold[1] = self.upper_threshold[1] + 1
                print_it = True
            elif key == ord('i'):
                self.upper_threshold[1] = self.upper_threshold[1] - 1
                print_it = True
            elif key == ord('k'):
                self.lower_threshold[1] = self.lower_threshold[1] + 1
                print_it = True
            elif key == ord(','):
                self.lower_threshold[1] = self.lower_threshold[1] - 1
                print_it = True
            elif key == ord('9'):
                self.upper_threshold[2] = self.upper_threshold[2] + 1
                print_it = True
            elif key == ord('o'):
                self.upper_threshold[2] = self.upper_threshold[2] - 1
                print_it = True
            elif key == ord('l'):
                self.lower_threshold[2] = self.lower_threshold[2] + 1
                print_it = True
            elif key == ord('.'):
                self.lower_threshold[2] = self.lower_threshold[2] - 1
                print_it = True
            if print_it:
                print(f"{self.upper_threshold[0]} {self.upper_threshold[1]} {self.upper_threshold[2]}")
                print(f"{self.lower_threshold[0]} {self.lower_threshold[1]} {self.lower_threshold[2]}")
        # Secrect commands for switching debug image modes
        if key == ord('y'):
            if self.debug_image == SC_DEBUG_NONE:
                self.debug_image = SC_DEBUG_MASK
            elif self.debug_image == SC_DEBUG_MASK:
                self.debug_image = SC_DEBUG_OVERLAY
            elif self.debug_image == SC_DEBUG_OVERLAY:
                self.debug_image = SC_DEBUG_NONE
        self.stop_timer("user_command")

    # Handle the user input
    def user_command_voiceRec(self):
        self.start_timer("user_command")
        key = None
        if self.use_local_window:
            # catch the key
            key = cv2.waitKey(1) & 0xFF
        elif self.use_socket:
            # Use select to check if data is available (non-blocking)
            readable, _, _ = select.select([self.conn], [], [], 0.1)
            if readable:
                try:
                    key = self.conn.recv(1).decode('utf-8')  # Read 1-byte key input
                    if key:
                        print(f"Received key: {key}")  # Process key input here
                        key = ord(key)
                except:
                    pass  # Ignore small errors
        # Press q on keyboard to stop program
        if key == ord('q'):
            self.quit = True
        # driving commands
        if self.drive_control:
            if key == ord('f'): # forward
                self.speed = 10
                print("speed:", self.speed)
            if key == ord('w'): # faster
                self.speed = self.speed + 10
                print("speed:", self.speed)
            elif key == ord('s'): #slower
                if self.speed > 0:
                    self.speed = self.speed - 10
                print("speed:", self.speed)
            elif key == ord('b'): # backward
                self.speed = -10
                print("speed:", self.speed)
            elif key == ord('x'): # stop
                self.speed = 0
                print("speed:", self.speed)
            

        # steering commands
        if self.steer_control:
            if key == ord('a'): # left
                self.steer = self.steer - 5
                if self.steer < 50:
                    self.steer = 50
                print("steer:", self.steer)
            elif key == ord('d'): # right
                self.steer = self.steer + 5
                if self.steer > 140:
                    self.steer = 140
                print("steer:", self.steer)
        
        self.stop_timer("user_command")


     #Handle the user input
    def voiceRec_actuator(self,commandInput):
        
        self.start_timer("user_command")
        
        if self.use_local_window:
            # catch the key
            key = cv2.waitKey(1) & 0xFF
        elif self.use_socket:
            # Use select to check if data is available (non-blocking)
            readable, _, _ = select.select([self.conn], [], [], 0.1)
            if readable:
                try:
                    key = self.conn.recv(1).decode('utf-8')  # Read 1-byte key input
                    if key:
                        print(f"Received key: {key}")  # Process key input here
                        key = ord(key)
                except:
                    pass  # Ignore small errors
        # driving commands
        # Press q on keyboard to stop program
        if key == ord('q'):
            self.quit = True

        # driving commands
        if self.drive_control:
            if commandInput == VoiceCommands.FORWARD:
                self.speed = 10
                print("speed:", self.speed)
            elif commandInput == VoiceCommands.FASTER:
                self.speed += 10
                print("speed:", self.speed)
            elif commandInput == VoiceCommands.SLOWER:
                if self.speed > 0:
                    self.speed -= 10
                print("speed:", self.speed)
            elif commandInput == VoiceCommands.BACKWARD:
                self.speed = -10
                print("speed:", self.speed)
            elif commandInput == VoiceCommands.STOP:
                self.speed = 0
                print("speed:", self.speed)

        # steering commands
        if self.steer_control:
            if commandInput == VoiceCommands.LEFT:
                self.steer -= 5
                if self.steer < 50:
                    self.steer = 50
                print("steer:", self.steer)
            elif commandInput == VoiceCommands.RIGHT:
                self.steer += 5
                if self.steer > 140:
                    self.steer = 140
                print("steer:", self.steer)


        
        self.stop_timer("user_command")

    # Apply the values to the PiCar actuators
    def handle_actuators(self):
        self.start_timer("handle_actuators")
        # Stop the vehicle if:
        # a) US distance is smaller than 20cm AND
        # b) Car drives forward
        # c) Measurement is trustful (no steps > 100cm)
        if self.distance < 20 and self.speed > 0 and abs(self.last_distance-self.distance) < 100:
            print(f"stopped because d = {self.distance}")
            print(f"Last distance was d = {self.last_distance}")
            self.speed = 0
        # apply the values to the hardware
        if self.speed < 0:
            self.bw.backward()
            #self.gpio_p13.write(True)
        else:
            self.bw.forward()
            #self.gpio_p13.write(False)
        self.bw.speed = abs(self.speed)
        self.fw.turn(self.steer)
        self.pan_servo.write(self.campan)
        self.tilt_servo.write(self.camtilt)
        # Write the frame into the file
        if self.recording:
            self.out.write(self.recordingframe)
        self.stop_timer("handle_actuators")

    # Detects the lanes and calculates steering angle automatically
    def lane_detection(self):
        # These are the rows incl. order of the image we want to scan for lanes (0...479 are possible)
        rows_to_scan = [360, 330, 300, 270, 240]
        self.start_timer("lane_detection.hsv")
        # 1. Masking depending on color thresholds
        ##########################################
        # a) Extract only relevant lines
        # Original shape is (480, 640, 3), new one is (5, 640, 3)
        frame_rows = np.full((len(rows_to_scan), 640, 3), 0, np.uint8)
        line_index = 0
        for i in rows_to_scan:
            frame_rows[line_index] = self.frame[i]
            line_index += 1
        # b) Create HSV of these lines
        hsv_rows = cv2.cvtColor(frame_rows, cv2.COLOR_BGR2HSV)
        self.stop_timer("lane_detection.hsv")
        self.start_timer("lane_detection.mask")
        # c) Mask these lines
        mask_rows = cv2.inRange(hsv_rows, self.lower_threshold, self.upper_threshold)
        self.stop_timer("lane_detection.mask")
        self.start_timer("lane_detection.debugimage")
        # 2. Handle debug image settings
        ################################
        line_color = (255, 255, 255)
        if self.debug_image == SC_DEBUG_MASK:
            line_color = (0, 0, 200)
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            self.frame = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
        elif self.debug_image == SC_DEBUG_OVERLAY:
            line_color = (0, 0, 200)
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            maskvis = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
            mask_as_bgr = cv2.cvtColor(maskvis, cv2.COLOR_GRAY2BGR)
            self.frame = np.bitwise_or(self.frame, mask_as_bgr)
        self.stop_timer("lane_detection.debugimage")
        self.start_timer("lane_detection.detect_lanes")
        # 3. Detect lanes
        #################
        # a) Get indices of gradients
        line_index = 0
        gradient_indices = []
        for i in rows_to_scan:
            #print(np.amax(mask_rows[line_index], axis=0))
            # Calculate the gradients (detect start and end edge of lane marks in rows)
            gradients = np.gradient(mask_rows[line_index], axis=0)
            # Get the indices of the high gradients
            gradient_indices.append(np.nonzero(gradients))
            line_index += 1
        # b) Try to find best lane pair
        width_threshold = [10, 35] # valid width of lane mark in pixels
        line_index = 0
        # Format: [first, second][x, y]
        lane_pair = [ [-1, -1], [-1, -1] ]
        for gradient_row in gradient_indices:
            temp_lane_pair = [ [-1, -1], [-1, -1] ]
            x_old = -1
            for x in gradient_row[0]:
                # First x
                if x_old == -1:
                    x_old = x
                # All next x values
                else:
                    if x-x_old > width_threshold[0] and x-x_old < width_threshold[1]:
                        if temp_lane_pair[0][0] == -1:
                            temp_lane_pair[0][0] = x_old
                            temp_lane_pair[0][1] = rows_to_scan[line_index]
                            # Store first lane mark that was found
                            if lane_pair[0][0] == -1:
                                lane_pair = temp_lane_pair
                            # Draw detection line
                            cv2.line(self.frame, (x_old, rows_to_scan[line_index]), (x, rows_to_scan[line_index]), line_color, 2)
                        else:
                            temp_lane_pair[1][0] = x
                            temp_lane_pair[1][1] = rows_to_scan[line_index]
                            # Store the lane pair
                            lane_pair = temp_lane_pair
                            # Draw detection line
                            cv2.line(self.frame, (x_old, rows_to_scan[line_index]), (x, rows_to_scan[line_index]), line_color, 2)
                    x_old = x
            # Stop searching if a lane pair was found
            if lane_pair[0][0] != -1 and lane_pair[1][0] != -1:
                break
            line_index += 1
        deviation = 0
        # A pair of lane marks was found
        if lane_pair[0][0] != -1 and lane_pair[1][0] != -1:
            # distance plausible - take this measurement
            if abs(lane_pair[1][0] - lane_pair[0][0]) > 200:
                self.x_left = lane_pair[0][0]
                self.x_right = lane_pair[1][0]
                deviation = (self.x_left + self.x_right) / 2 - 320
            #print("self.x_left: " + str(self.x_left) + " self.x_right: " + str(self.x_right))
        # Only one lane was found
        elif temp_lane_pair[0][0] != 0:
            x = lane_pair[0][0]
            y = lane_pair[0][1]
            #print("self.x_left: " + str(self.x_left))
            # Left lane detected
            if abs(x-self.x_left) < abs(x-self.x_right): # center of lane in pixels is 55 (original: 119)
                deviation = 55 + (y - 269) * 183 / 99 + x - 320
                self.x_left = x
                self.x_right = 10000 # very high
            # Right lane detected
            else: # center of lane in pixels is 55 (original: 82)
                deviation = x - (y - 265) * 209 / 108 - 55 - 320    #159 + (y - 139) * 142 / 73 - 320
                self.x_left = -10000 # very low
                self.x_right = x
            #print("self.x_left: " + str(self.x_left) + " self.x_right: " + str(self.x_right))
        # Draw deviation line
        cv2.line(self.frame, (int(deviation+320), 480), (int(deviation+320), 240), (255, 255, 255), 1)
        # Draw center line
        cv2.line(self.frame, (320, 480), (320, 240), (255, 0, 0), 1)
        #print("Detection line: " + str(lane_pair[0][1]))
        #print("deviation: " + str(deviation))
        self.stop_timer("lane_detection.detect_lanes")
        self.start_timer("lane_detection.controller")
        # 4. Steering controller
        ########################
        # Controller shall only be active while car is driving
        if self.speed != 0:
            self.p_part = self.kp * deviation;
            self.i_part = self.i_part + (time.time()-self.old_time) * deviation * self.ki;
            self.d_part = (deviation - self.old_error) / (time.time() - self.old_time) * self.kd;
            controlValue = self.p_part + self.i_part + self.d_part;
            self.old_error = deviation;
            self.old_time = time.time();
            self.steer = 90.0 + controlValue
        self.add_pid()
        self.stop_timer("lane_detection.controller")

    # Starts the timer for a specified code block
    # Stores the start time in the dictionary
    def start_timer(self, blockname):
        if self.measure_performance:
            if blockname in self.time_consumptions:
                self.time_consumptions[blockname].append(time.time())
            else:
                self.time_consumptions[blockname] = [time.time()]
        
    # Stops the timer for a specified code block
    # Replaces the start time with the elapsed time in the dictionary
    def stop_timer(self, blockname):
        if self.measure_performance:
            if blockname in self.time_consumptions:
                self.time_consumptions[blockname][-1] = time.time() - self.time_consumptions[blockname][-1]
            else:
                self.time_consumptions[blockname] = ["error"]
    
    # Only shows a camera window including recording option
    def camera_window_loop(self):
        # Redefine avaiable controls
        self.drive_control = False
        self.steer_control = False
        self.camera_control = True
        self.recording_control = True
        self.use_ultrasonic = False
        # Main loop for image processing
        while(True): 
            self.handle_sensors()
            self.handle_window()
            self.user_command()
            if self.quit:
                break
            self.handle_actuators()

    # Drive the car manually while showing a camera window
    # The user can interact with the car using the keyboard:
    # See variable 'text' for key assignments
    def drive_and_record_loop(self):
        # Redefine avaiable controls
        self.drive_control = True
        self.steer_control = True
        self.camera_control = True
        self.recording_control = True
        self.use_ultrasonic = True
        while(True):
            self.handle_sensors()
            self.handle_window()
            self.user_command()
            if self.quit:
                break
            self.handle_actuators()

    
    # Drive the car manually while showing a camera window
    # The user can interact with the car using the keyboard:
    # See variable 'text' for key assignments
    def drive_and_record_loop_voiceRec(self):
        # Redefine avaiable controls
        self.drive_control = True
        self.steer_control = True
        self.camera_control = True
        self.recording_control = True
        self.use_ultrasonic = True
        while(True):
            self.handle_sensors()
            self.handle_window()
            self.user_command_voiceRec()
            if self.quit:
                break
            self.handle_actuators()



    # Drive the car manually while showing a camera window
    # The user can interact with the car using the keyboard:
    # See variable 'text' for key assignments
    def lane_detection_loop(self):
        # Redefine avaiable controls
        self.drive_control = True
        self.steer_control = False
        self.camera_control = True
        self.recording_control = True
        self.use_ultrasonic = True
        while(True):
            self.handle_sensors()
            self.lane_detection()
            self.user_command()
            if self.quit:
                break
            self.handle_actuators()
            self.handle_window()

    
    


            

#flows-dashboard-SmartCar.json