import serial
import struct
import sys
import time
from threading import Thread

# number of samples for one line from the camera
n = 640
# maximum value for an uint8
max_value = 255
# time measurement stuff interval thingy
interval = 0.001

# handler when closing the window
def handle_close(evt):
    # we stop the serial thread
    reader_thd.stop()
    print("goodbye")


# reads the data in uint8 from the serial
def readUint8Serial(port):
    state = 0
    while(state != 5):
        # reads 1 byte
        c1 = port.read(1)
        # timeout condition
        if(c1 == b''):
            print('Timout...')
            return []
        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    # reads the size
    # converts as short int in little endian the two bytes read
    size = struct.unpack('<h', port.read(2))
    # removes the second element which is void
    size = size[0]

    # reads the data
    rcv_buffer = port.read(size*2)
    # rcv_buffer = port.read(size)
    data = []

    i=0
    while(i < size):
        data.append(struct.unpack_from('<h', rcv_buffer, i*2))
        i = i+1

    speed_l = data[0][0]
    speed_r = data[1][0]
    return [speed_l, speed_r]
#    if(len(rcv_buffer) == size*2):
#        i = 0
#        while(i < size):
#            data.append(struct.unpack_from('<i', rcv_buffer, i*2))
#            i = i+1
#        print('received !')
#        return data
#    else:
#        print('Timout...')
#        return []


# Data process function
def processData(data):
    print(data)


# thread used to control the communication part
class serial_thread(Thread):
    # init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = True
        self.alive = True
        self.need_to_update = False
        i = 0
        data = []

        print('Connecting to port {}'.format(port))

        try:
            self.port = serial.Serial(port, timeout=0.5)
        except BaseException:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    # function called after the init
    def run(self):
        while(self.alive):
            if(self.contReceive):
                data[i] = readUint8Serial(self.port)
                i++
                processData(data)
                # update_cam_plot(self.port)
            else:
                # flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    # enables the continuous reading
    def setContReceive(self, val):
        self.contReceive = True

    # disables the continuous reading
    def stop_reading(self, val):
        self.contReceive = False

    # clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()


# test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)

# serial reader thread config
# begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()
