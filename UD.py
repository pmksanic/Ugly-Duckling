#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 27 May 2015

###########################################################################
# Copyright (c) 2015 iRobot Corporation
# http://www.irobot.com/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

from tkinter import *
from tkinter import messagebox as tkMessageBox
from tkinter import simpledialog as tkSimpleDialog
# import tkMessageBox
# import tkSimpleDialog

import struct, time, threading
import sys, glob # for listing serial ports
import face_recognition as fr
import cv2

try:
    import serial
except ImportError:
    tkMessageBox.showerror('Import error', 'Please install pyserial.')
    raise

connection = None

TEXTWIDTH = 40 # window width, in characters
TEXTHEIGHT = 16 # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 200

helpText = """\
Supported Keys:
P\tPassive
S\tSafe
F\tFull
C\tClean
D\tDock
R\tReset
Space\tBeep
Arrows\tMotion

If nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.
"""

class TetheredDriveApp(Tk):
    # static variables for keyboard callback -- I know, this is icky
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ''

    def __init__(self):
        Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        createMenu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.text = Text(self, height = TEXTHEIGHT, width = TEXTWIDTH, wrap = WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=LEFT, fill=BOTH, expand=True)
        self.scroll.pack(side=RIGHT, fill=Y)

        self.text.insert(END, helpText)

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

    # sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
    def sendCommandASCII(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))

        self.sendCommandRaw(cmd.encode())

    # sendCommandRaw takes a string interpreted as a byte array
    def sendCommandRaw(self, command):
        global connection

        try:
            if connection is not None:
                connection.write(command)
            else:
                tkMessageBox.showerror('Not connected!', 'Not connected to a robot!')
                print("Not connected.")
        except serial.SerialException:
            print("Lost connection")
            tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None
        print(command)
        print(' '.join([ str(c) for c in command ]))
        self.text.insert(END, ' '.join([ str(c) for c in command ]))
        self.text.insert(END, '\n')
        self.text.see(END)

    # getDecodedBytes returns a n-byte value decoded using a format string.
    # Whether it blocks is based on how the connection was set up.
    def getDecodedBytes(self, n, fmt):
        global connection
        
        try:
            return struct.unpack(fmt, connection.read(n))[0]
        except serial.SerialException:
            print("Lost connection")
            tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None
            return None
        except struct.error:
            print("Got unexpected data from serial port.")
            return None

    # get8Unsigned returns an 8-bit unsigned value.
    def get8Unsigned(self):
        return getDecodedBytes(1, "B")

    # get8Signed returns an 8-bit signed value.
    def get8Signed(self):
        return getDecodedBytes(1, "b")

    # get16Unsigned returns a 16-bit unsigned value.
    def get16Unsigned(self):
        return getDecodedBytes(2, ">H")

    # get16Signed returns a 16-bit signed value.
    def get16Signed(self):
        return getDecodedBytes(2, ">h")

    def move(self, motionChange):
        if motionChange == True:
            velocity = 300
            velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
            velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
            rotation = 0
            rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
            rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

            # compute left and right wheel velocities
            vr = velocity + (rotation/2)
            vl = velocity - (rotation/2)

            # create drive command
            cmd = struct.pack(">Bhh", 145, int(vr), int(vl))
            if cmd != self.callbackKeyLastDriveCommand:
                self.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd

    def Left(self, x):
        self.callbackKeyLeft = True
        motionChange = True
        self.move(motionChange)
        time.sleep(0.01 * x)
        self.callbackKeyLeft = False
        motionChange = True
        self.move(motionChange)

    def stop(self):
        self.callbackKeyLeft = False
        self.callbackKeyRight = False
        motionChange = True
        self.move(motionChange)

    def Right(self, x):
        self.callbackKeyRight = True
        motionChange = True
        self.move(motionChange)
        time.sleep(0.01 * x)
        self.callbackKeyRight = False
        motionChange = True
        self.move(motionChange)

    def Up(self):
        self.callbackKeyUp = True
        motionChange = True
        self.move(motionChange)
        time.sleep(0.2)
        self.callbackKeyUp = False
        motionChange = True
        self.move(motionChange)

    def Down(self):
        self.callbackKeyDown = True
        motionChange = True
        self.move(motionChange)
        time.sleep(0.2)
        self.callbackKeyDown = False
        motionChange = True
        self.move(motionChange)

    # A handler for keyboard events. Feel free to add more!
    def callbackKey(self, event):
        k = event.keysym.upper()
        motionChange = False

        if event.type == '2': # KeyPress; need to figure out how to get constant
            if k == 'P':   # Passive
                self.sendCommandASCII("128")
            elif k == 'S': # Safe
                self.sendCommandASCII("131")
            elif k == 'F': # Full
                self.sendCommandASCII("132")
            elif k == 'C': # Clean
                self.sendCommandASCII("135")
            elif k == 'D': # Dock
                self.sendCommandASCII("143")
            elif k == 'SPACE': # Beep
                self.sendCommandASCII("140 3 1 64 16 141 3")
            elif k == 'R': # Reset
                self.sendCommandASCII("7")
            elif k == 'UP':
                self.callbackKeyUp = True
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = True
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = True
                motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = True
                motionChange = True
            elif k == 'RETURN':
                for i in range(100):
                    vr = 10 * i
                    vl = -10 * i
                    cmd = struct.pack(">Bhh", 145, vr, vl)
                    self.sendCommandRaw(cmd)
                    time.sleep(0.1)
                cmd = struct.pack(">Bhh", 145, 0, 0)
                self.sendCommandRaw(cmd)
                # for i in range(100):
                #     vr = -10 * i
                #     vl = 10 * i
                #     cmd = struct.pack(">Bhh", 145, vr, vl)
                #     self.sendCommandRaw(cmd)
                #     time.sleep(0.1)
                cmd = struct.pack(">Bhh", 145, 0, 0)
                self.sendCommandRaw(cmd)
            else:
                print(repr(k), "not handled")
        elif event.type == '3': # KeyRelease; need to figure out how to get constant
            if k == 'UP':
                self.callbackKeyUp = False
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = False
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = False
                motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = False
                motionChange = True
            
        if motionChange == True:
            velocity = 0
            velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
            velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
            rotation = 0
            rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
            rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

            # compute left and right wheel velocities
            vr = velocity + (rotation/2)
            vl = velocity - (rotation/2)

            # create drive command
            cmd = struct.pack(">Bhh", 145, int(vr), int(vl))
            if cmd != self.callbackKeyLastDriveCommand:
                self.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd

    def onConnect(self):
        global connection

        if connection is not None:
            tkMessageBox.showinfo('Oops', "You're already connected!")
            return

        try:
            ports = self.getSerialPorts()
            # port = tkSimpleDialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
            port = "COM3"
        except EnvironmentError:
            print("Environments")
            pass
            # port = tkSimpleDialog.askstring('Port?', 'Enter COM port to open.')

        if port is not None:
            print("Trying " + str(port) + "... ")
            try:
                connection = serial.Serial(port, baudrate=115200, timeout=1)
                print("Connected!")
                # tkMessageBox.showinfo('Connected', "Connection succeeded!")
            except:
                print("Failed.")
                tkMessageBox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))


    def onHelp(self):
        # pass
        tkMessageBox.showinfo('Help', helpText)

    def onQuit(self):
        # pass
        if tkMessageBox.askyesno('Really?', 'Are you sure you want to quit?'):
            self.destroy()

    def getSerialPorts(self):
        """Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

def face_recog(app):
    app.onConnect()
    app.sendCommandASCII("128")
    app.sendCommandASCII("131")
    video_capture = cv2.VideoCapture(0)

    obama_encodings = []
    pranav_encodings = []

    # Learn a person's face based on images

    # for i in range(2):
    #     obama_encodings += [fr.face_encodings(fr.load_image_file("obama" + str(i+1) + ".jpg"), num_jitters=1)[0]]

    for i in range(2):
        pranav_encodings += [fr.face_encodings(fr.load_image_file("pranav" + str(i+1) + ".jpg"), num_jitters=1)[0]]

    # Initialize some variables
    face_locations = []
    face_encodings = []
    face_names = []
    process_this_frame = True

    while True:
        # Grab a single frame of video
        ret, frame = video_capture.read()

        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

        # Only process every other frame of video to save time
        if process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            face_locations = fr.face_locations(small_frame, model="hog")
            print(face_locations)
            face_encodings = fr.face_encodings(small_frame, face_locations)

            face_names = []
            for face_encoding in face_encodings:
                name = "Subra"

                # See if the face is a match for the known face(s)
                # match_obama = fr.compare_faces(obama_encodings, face_encoding, tolerance = 0.6)
                # print("Match obama:",match_obama)
                # c=sum(1 for i in match_obama if i)
                # if c>=len(match_obama)/2:
                #     name = "Barack"
                
                match_pranav = fr.compare_faces(pranav_encodings, face_encoding, tolerance = 0.6)
                # print("Match pranav:",match_pranav)
                # print(fr.face_distance(pranav_encodings, face_encoding))
                c=sum(1 for i in match_pranav if i)
                if c>=len(match_pranav)/2:
                    name = "Mama Goose"

                face_names.append(name)

        process_this_frame = not process_this_frame


        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            
            if name == "Mama Goose" and left < 130:
                print("Turn left")
                app.Left(130 - left)
            elif name == "Mama Goose" and left > 130:
                print("Turn right")
                app.Right(left - 130)
            # elif name == "Mama Goose" and 80 < left < 180 and (app.callbackKeyLeft == True or app.callbackKeyRight == True):
            #     print("stop")
            #     app.stop()

            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 2
            right *= 2
            bottom *= 2
            left *= 2

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow('Video', frame)

        # Hit 'q' on the keyboard to quit!
        k = cv2.waitKey(1)
        # print(k)
        if k & 0xFF == ord('q'):
            app.sendCommandASCII("128")
            break
        elif k != -1:
            print("calling Up()")
            app.Up()

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()

app = TetheredDriveApp()
# thread1 = threading.Thread(target = app.mainloop)
thread2 = threading.Thread(target = face_recog, args = (app,))
# thread2.daemon = True
thread2.start()
# thread1.start()
app.mainloop()