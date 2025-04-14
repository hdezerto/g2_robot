import cv2
from cv_bridge import CvBridge

import rclpy
import os
import time
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from my_custom_interfaces.srv import Pickup  
from std_msgs.msg import Int64MultiArray, Int16MultiArray,MultiArrayDimension, MultiArrayLayout, String
from armplanner.kinematics3 import inverse_kinematics, compute_fk, translate_to_servo

class PickupSuccess(Node):
    def __init__(self):
        super().__init__('check_pickup_service')
        
        # Create a service client for the Pickup service
        self.client = self.create_client(Pickup, 'pickup/check',self.callback)

        self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.subscription = self.create_subscription(
            Image, '/arm_camera/image_raw', self.image_callback, 10)
        
        # Loading calibration data
        curFolder = os.path.dirname(os.path.abspath(__file__))
        paramPath = os.path.join(curFolder, 'calibration2.npz')
        data = np.load(paramPath, allow_pickle=True)
        self.camMatrix = data['camMatrix']
        self.distCoeff = data['distCoeff']
        

        


    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def callback(self, object_type, color):
        # detection logic
        plush_orientation = 'DEFAULT'
        start = time.time()
        frame = self.latest_frame
        if frame is None:
                self.get_logger().warn("No frame received yet!")
                return
        print('frame received')
        img = frame.copy()
        height, width = img.shape[:2]
        camMatrixNew,roi, = cv2.getOptimalNewCameraMatrix(self.camMatrix, self.distCoeff,(width,height),1, (width,height))
        imgUndist = cv2.undistort(img,self.camMatrix, self.distCoeff, None, camMatrixNew)
        
        # Crop the undistorted image
        x, y, w, h = roi
        cropped_img = imgUndist[y:y+h-34, x:x+w]

        if object_type == 'Cube' or object_type == 'Sphere':
            


            # Convert the image to grayscale
            gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)

            # Apply Canny edge detection
            kernel = np.ones((5, 5), np.uint8)
            edges = cv2.Canny(gray_img, threshold1=50, threshold2=150)
            inverted_edges = cv2.bitwise_not(edges)
            maskk = cv2.erode(inverted_edges, kernel, iterations=1)

            # Optionally, visualize the edges
            cv2.imwrite('/home/happy/maskk.png', maskk)
            #end = time.time()
            #print('time', end-start)

            points = np.column_stack(np.where(maskk > 0))
            cx,cy = 0, 0 # maybe change this
            avg_point = 0
            mask2 = maskk.copy()
            if points.size > 0:
                # Apply DBSCAN clustering
                clustering = DBSCAN(eps=2, min_samples=10).fit(points)  # Adjust 'eps' based on pixel scale
                
                # Get labels and find the largest cluster
                labels, counts = np.unique(clustering.labels_, return_counts=True)

                # Ignore noise points (-1 label) and find the largest cluster
                valid_labels = labels[labels != -1]
                
                filtered_clusters = []

                for label in valid_labels:
                    cluster_points = points[clustering.labels_ == label]
                    print(label)
                    ys, xs = cluster_points[:, 0], cluster_points[:, 1]
                    width = xs.max() - xs.min()
                    height = ys.max() - ys.min()
                    #width < 80 and height < 80 and 
                    # Example condition: must be roughly square and not too thin
                    if 0.7 < (width / height) < 1.5:
                        filtered_clusters.append((label, len(cluster_points)))

                if filtered_clusters:
                    # Choose largest valid cluster from filtered
                    best_label = max(filtered_clusters, key=lambda x: x[1])[0]
                    largest_cluster_points = points[clustering.labels_ == best_label]
                    cluster_size = len(largest_cluster_points)

                    # Compute centroid
                    avg_point = np.mean(largest_cluster_points, axis=0)
                    cx, cy = int(avg_point[1]), int(avg_point[0])

                    filtered_mask = np.zeros_like(mask2)
                    filtered_mask[largest_cluster_points[:, 0], largest_cluster_points[:, 1]] = 255

                else:
                    cx, cy = None, None
                    cluster_size = 0
                    filtered_mask = np.zeros_like(mask2)
            
            end = time.time()
            print('detection time:', end-start)
                
        
            
            

            #return msg

        if object_type == 'Plushie':
            # Implement Plushie detection logic here
            gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)

            # Apply Canny edge detection
            kernel = np.ones((5, 5), np.uint8)
            edges = cv2.Canny(gray_img, threshold1=50, threshold2=150)
            inverted_edges = cv2.bitwise_not(edges)
            maskk = cv2.erode(inverted_edges, kernel, iterations=1)

            # Optionally, visualize the edges
            cv2.imwrite('/home/happy/maskk.png', maskk)


            points = np.column_stack(np.where(maskk > 0))
            cx,cy = 0, 0 # maybe change this
            avg_point = 0
            mask2 = maskk.copy()
            if points.size > 0:
                # Apply DBSCAN clustering
                clustering = DBSCAN(eps=2, min_samples=10).fit(points)  # Adjust 'eps' based on pixel scale
                
                # Get labels and find the largest cluster
                labels, counts = np.unique(clustering.labels_, return_counts=True)

                # Ignore noise points (-1 label) and find the largest cluster
                valid_labels = labels[labels != -1]
                
                filtered_clusters = []
                iteration =  0
                plush_orientation = {}
                for label in valid_labels:
                    cluster_points = points[clustering.labels_ == label]

                    ys, xs = cluster_points[:, 0], cluster_points[:, 1]
                    width = xs.max() - xs.min()
                    height = ys.max() - ys.min()
                    #width < 80 and height < 80 and 
                    # Example condition: must be roughly square and not too thin
                    iteration += 1
                    if True:   
                        filtered_clusters.append((label, len(cluster_points)))
                    if 0.7 < (width / height) < 1.5 and len(cluster_points) > 500:
                        plush_orientation[label] = 'SIDE'
                    else:
                        plush_orientation[label] = 'DEFAULT'

                if filtered_clusters:
                    # Choose largest valid cluster from filtered
                    best_label = max(filtered_clusters, key=lambda x: x[1])[0]
                    largest_cluster_points = points[clustering.labels_ == best_label]
                    cluster_size = len(largest_cluster_points)
                    self.plush_orientation = plush_orientation[best_label]

                    # Compute centroid
                    avg_point = np.mean(largest_cluster_points, axis=0)
                    cx, cy = int(avg_point[1]), int(avg_point[0])

                    filtered_mask = np.zeros_like(mask2)
                    filtered_mask[largest_cluster_points[:, 0], largest_cluster_points[:, 1]] = 255

                else:
                    cx, cy = None, None
                    cluster_size = 0
                    filtered_mask = np.zeros_like(mask2)
            pass

        if cx == None or cy == None:
                print('No object detected')
                cx, cy = 0, 0
                return None
        # Compute the real-world coordinates of the centroid
        h, w = cropped_img.shape[:2]
        xnew = h - cy 
        ynew = w/2 - cx

        pos_x = xnew/h * 17.5/100 + 10/100  # KANSKE BORDE VARA TYP 17.6-17.7 ish
        pos_y = ynew/(w/2) * 25/200 
        if object_type == 'Cube' or object_type == 'Sphere':
            if pos_y < 0:
                pos_y = pos_y + 1/100
            elif pos_y > 0:
                pos_y = pos_y - 1/100
        elif object_type == 'Plushie':
            pos_x = pos_x - 1/100


        # Return the position of the object
        msg = PointStamped()
        msg.point.x = float(pos_x)
        msg.point.y = float(pos_y)
        msg.point.z = -0.13 # placeholder value
        self.get_logger().info(f"Object detected at: {msg.point.x}, {msg.point.y}, {msg.point.z}")


        # Draw a circle at the center
        cv2.circle(cropped_img, (cx, cy), 5, (0, 255, 0), -1)
        cv2.circle(cropped_img, (cy, cx), 5, (0, 0, 255), -1)  # Red dot for the centroid
        cv2.putText(cropped_img, f"({cx}, {cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) 

        cv2.circle(cropped_img,(int(w/2), int(h/2)), 5, (0, 255, 0), -1)
        cv2.putText(cropped_img, f"({w/2}, {h/2})", (int(w/2 + 10),int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the processed frame
        original_image = cropped_img.copy()
        if len(original_image.shape) == 2:  # If grayscale, convert to BGR
            original_image = cv2.cvtColor(original_image, cv2.COLOR_GRAY2BGR)

        # Create a copy of the original image
        overlay = original_image.copy()

        # Highlight the largest cluster in red
        overlay[filtered_mask > 0] = [0, 255, 0]  # Set only cluster points to red

        # Blend the overlay with the original image
        alpha = 0.5  # Transparency factor
        result = cv2.addWeighted(original_image, 1 - alpha, overlay, alpha, 0)

        # Show the result
        cv2.imwrite("/home/happy/cluster_overlay.png", result)

        return msg


def main(args=None):
    rclpy.init(args=args)

    # Create the client node
    client_node = PickupClient()

    # Example usage: Request pickup of a Green Cube
    client_node.send_request('Cube', 'Red')

    # Spin the client node to process the request
    rclpy.spin(client_node)

    # Shutdown when done
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()