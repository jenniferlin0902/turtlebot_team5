#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class DeliveryRequestPublisher:
    """Publishes food delivery requests to the PavoneCart food delivery service."""

    def __init__(self):
        # Initialize node.
        rospy.init_node('delivery_request_publisher', anonymous=True)

        # Requested items.
        self.delivery_request = None

        # Create publisher.
        self.request_publisher = rospy.Publisher('/delivery_request', String, queue_size=10)

    def publish_request(self):
        # Publish the request t times, once every s seconds.
        t = 1
        s = 1.
        for i in range(t):
            self.request_publisher.publish(self.delivery_request)
            rospy.sleep(s)
    
    def loop(self):
        """The main loop of the script. The script will ask for food items to add to the 
        delivery_request string until an empty answer ("") is given, at which point it will 
        publish the string. The current request will be published several few times after which the
        user will be prompted to create a new request."""
        if self.delivery_request is None:
            # Initialize string of requested items.
            self.delivery_request = ""
            
            # Gather requests from user input.
            request_more_items = True
            while request_more_items:
                new_item = raw_input("Add an item to your delivery request: ")
                if new_item == "":
                    request_more_items = False
                else:
                    self.delivery_request += new_item + ","

            # Eliminate trailing comma and publish.
            self.delivery_request = self.delivery_request[:-1]
            print "Sending order:", self.delivery_request
            self.publish_request()

            # Reset delivery request to be ready for new inputs
            self.delivery_request = None
            print "\n", "Create a delivery request:"

    def run(self):
        print "Create a delivery request:"
        print "You'll be prompted to enter food items one at a time. Once your order list is " \
            "complete, press enter to send your order."
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    may_i_take_your_order = DeliveryRequestPublisher()
    may_i_take_your_order.run()
