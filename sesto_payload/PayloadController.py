#!/usr/bin/env python3

# license removed for brevity
# This scripts runs a node that perform as a payload manager task
# Created by Tan You Liang, at Nov 2018

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from . import MFRC522
import _thread

from std_msgs.msg import Int32


class PayloadControllerNode(Node):
    def __init__(self):
        super().__init__('payload_controller_node')
        self.pub = self.create_publisher(Int32, 'door_control_command')
        self.sub = self.create_subscription(Int32, 'door_status', self.doorStatus_callback)
        self.continue_reading = True

        # msg = Int32()
        # msg.data = somthing
        # publisher.publish(msg)

        try:
            _thread.start_new_thread(self.readRFID, ())
        except:
            print("Error in thread creation!!!")
    
    def main(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:  # this happens after Ctrl+C
            pass

        print("starting shutdown...")
        self.continue_reading = False
        GPIO.cleanup()
        # need to figure out how to get the MFRC522_Request() call to un-block
        # maybe using a select() call or something else that is
        # interruptable when it's time to quit.

    def doorStatus_callback(self, msg):
        print("door current state: {}".format(msg.data))


    # Read RFID Value with RC522
    def readRFID(self):
        print("Read rfid thread")
        # Create an object of the class MFRC522
        MIFAREReader = MFRC522.MFRC522()
        # This loop keeps checking for chips. If one is near it will get the UID and authenticate
        while self.continue_reading:
            # Scan for cards, will block here while none to read    
            (status,TagType) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQIDL)
            # If we have the UID, continue
            if status == MIFAREReader.MI_OK:
                print("---- Card detected ----")
                # Get the UID of the card
                (status,uid) = MIFAREReader.MFRC522_Anticoll()
                # Print UID
                print("Card read UID: %s,%s,%s,%s" % (uid[0], uid[1], uid[2], uid[3]))
                card_id = uid[0] + uid[1] + uid[2] + uid[3]
                print(card_id)
                # This is the default key for authentication
                key = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
                # Select the scanned tag
                MIFAREReader.MFRC522_SelectTag(uid)
                # Authenticate
                status = MIFAREReader.MFRC522_Auth(MIFAREReader.PICC_AUTHENT1A, 8, key, uid)
                # Check if authenticated
                if status == MIFAREReader.MI_OK:
                    Data = MIFAREReader.MFRC522_Read(8)
                    print("Data: {}".format(Data))
                    MIFAREReader.MFRC522_StopCrypto1()
                else:
                    print("Authentication error")

######################################################################
def payload_controller_main(args=None):
    rclpy.init(args=args)
    node = PayloadControllerNode()
    node.main()


if  __name__ == "__main__":
    payload_controller_main()
