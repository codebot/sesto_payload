#!/usr/bin/env python
# license removed for brevity
# This scripts runs a node that perform as a payload manager task
# Created by Tan You Liang, at Nov 2018

import rclpy
import RPi.GPIO as GPIO
import MFRC522
import signal
import thread

from std_msgs.msg import Int32

continue_reading = True

# Capture SIGINT for cleanup when the script is aborted
def end_read(signal,frame):
    global continue_reading
    print "Ctrl+C captured, ending read."
    continue_reading = False
    GPIO.cleanup()
    exit(0)


# Read RFID Value with RC522
def readRFID():
    
    print "Read rfid thread"
    

    # Create an object of the class MFRC522
    MIFAREReader = MFRC522.MFRC522()

    # This loop keeps checking for chips. If one is near it will get the UID and authenticate
    while continue_reading:
        
        # Scan for cards, will block here while none to read    
        (status,TagType) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQIDL)

        # If we have the UID, continue
        if status == MIFAREReader.MI_OK:
            
            print "---- Card detected ----"
        
            # Get the UID of the card
            (status,uid) = MIFAREReader.MFRC522_Anticoll()

            # Print UID
            
            print "Card read UID: %s,%s,%s,%s" % (uid[0], uid[1], uid[2], uid[3])
            id = uid[0] + uid[1] + uid[2] + uid[3]
            print id
            
            # This is the default key for authentication
            key = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
            
            # Select the scanned tag
            MIFAREReader.MFRC522_SelectTag(uid)
            
            # Authenticate
            status = MIFAREReader.MFRC522_Auth(MIFAREReader.PICC_AUTHENT1A, 8, key, uid)

            # Check if authenticated
            if status == MIFAREReader.MI_OK:
                Data = MIFAREReader.MFRC522_Read(8)
                print "Data ", Data
                MIFAREReader.MFRC522_StopCrypto1()
            else:
                print "Authentication error"


def doorStatus_callback(msg):
    # global node
    # g_node.get_logger().info('I heard: "%s"' % msg.data)
    print 'door current state', msg.data


if  __name__ == "__main__":

    rclpy.init(args=None)
    node = rclpy.create_node('payload_controller_node')
    publisher = node.create_publisher(Int32, 'door_control_command')
    subscription = node.create_subscription(Int32, 'door_status', doorStatus_callback)

    # msg = Int32()
    # msg.data = somthing
    # publisher.publish(msg)


    # Hook the SIGINT
    signal.signal(signal.SIGINT, end_read)

    try:
        thread.start_new_thread(readRFID, ())
    except:
        print "Error in thread creation!!!"
    
    while 1:
       # ros pub? maybe
        rclpy.spin_once(node)
