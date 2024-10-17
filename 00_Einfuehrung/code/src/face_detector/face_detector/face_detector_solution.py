import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import os 

# Create a FaceDetectorPubSub class, which is a subclass of the Node class. It subscribes for the video_raw topic and publishes to the video_tace topic.
class FaceDetectorPubSub(Node): 
    """ 
    Create a FaceDetectorPubSub class, which is a subclass of the Node class. 
    """ 
    def __init__(self): 
        """ 
        Class constructor   
        """ 
        # Initiate the Node class's constructor and give it a name. 
        super().__init__('face_detector_pubsub') 
         
        # Create the publisher. This publisher will publish an Image 
        # to the video_face topic. The queue size is 10 messages. 
        self.publisher_ = self.create_publisher(Image, 'video_face', 10) 
         
        # Create the subscriber. This subscriber will subscribe to the video_raw topic.
        self.subscription = self.create_subscription(Image, 'video_raw', self.image_callback, 10) 
        self.subscription

        # Used to convert between ROS and OpenCV images.
        self.br = CvBridge()

        # Load the haar cascade classifier.
        self.haar_path = os.path.join(os.path.dirname(__file__), 'resource/haarcascade_frontalface_default.xml')
        self.haar_path = '/home/sebastian/Desktop/Vorlesungen/Nordakademie/VL_Robotik/00_Intro/code/src/face_detector/resource/haarcascade_frontalface_default.xml' 
        self.face_cascade = cv2.CascadeClassifier(self.haar_path)

    def image_callback(self, msg):
        
        # Convert the ROS Image message to a CV2 image.
        frame = self.br.imgmsg_to_cv2(msg)
        
        # Convert to gray scale image.
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces & returns positions of faces as Rect(x,y,w,h).
        face_rects = self.face_cascade.detectMultiScale(frame_gray, 1.3, 5)

        # Draw rectangles representing the detected faces.
        for (x, y, w, h) in face_rects:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Convert the CV2 image to a ROS Image message.
        msg = self.br.cv2_to_imgmsg(frame, encoding='rgb8')

        # Publish the image.
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    face_detector_pubsub = FaceDetectorPubSub()

    rclpy.spin(face_detector_pubsub)

    face_detector_pubsub.destroy_node()
    rclpy.shutdown()