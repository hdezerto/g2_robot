def pickup_callback2(self, request, response):
        self.get_logger().info(f'Received pickup request for: {request.object_type}, {request.color}')

        # Dummy position, ideally use detection logic
        obj_pos = PointStamped()
        obj_pos.point.x = 0.2
        obj_pos.point.y = 0.0
        obj_pos.point.z = -0.13

        # Create and send detection request
        detection_request = GetPosition.Request()
        detection_request.object_type = request.object_type
        detection_request.color = request.color
       

        # Camera trigger (optional)
        if not self.detection_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Detection service not available!")
            response.success = False
            response.message = "Could not connect to detection service."
            return response

        
        print('boo')
        future = self.detection_client.call_async(detection_request)
        # Use a timeout with spin_until_future_complete
        result = rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        print(f"spin_until_future_complete returned: {result}")

        if result == rclpy.executors.FutureReturnCode.SUCCESS:
            print('yo')
            response_obj = future.result()
            print(f"Got response: success={response_obj.success}, message={response_obj.message}")
        else:
            print(f"Future did not complete in time! Return code: {result}")
            response.success = False
            response.message = "Detection service timed out"
            return response
        """ 
        #print(future.result())
        # Wait for result (sync way for simplicity)
        rclpy.spin_until_future_complete(self, future)
        print('yo') """
        if future.result() is not None and future.result().success:
            position = future.result().position
            self.get_logger().info(f"Object detected at ({position.point.x:.2f}, {position.point.y:.2f})")
        else:
            self.get_logger().error("Object detection failed!")
            response.success = False
            response.message = "Object detection failed."
            return response
        # Run IK and send servo commands
        angles, servo_angles = inverse_kinematics(position)
        self.control_servos(servo_angles, 'PICK')

        response.success = True
        response.message = f"Pickup done for: {request.object_names}"
        return response



## color range for RED CUBE
lower_color1 = np.array([170, 100, 150])  # Lower bound of red in HSV
                upper_color1 = np.array([180, 220, 255]) # Upper bound of red in HSV

                lower_color2 = np.array([0, 85, 204])  # Lower bound of red in HSV
                upper_color2 = np.array([10, 200, 200]) # Upper bound of red in HSV

                lower_color3 = np.array([0, 1, 250]) # Lower bound of red in HSV
                upper_color3 = np.array([10, 125, 255]) # Upper bound of red in HSV

                lower_color4 = np.array([170, 3, 251])  # Lower bound of red in HSV
                upper_color4 = np.array([179, 98, 255]) # Upper bound of red in HSV



mask11 = cv2.inRange(hsv_frame, lower_color1, upper_color1)
            mask22 = cv2.inRange(hsv_frame, lower_color2, upper_color2)
            mask33 = cv2.inRange(hsv_frame, lower_color3, upper_color3)
            mask44 = cv2.inRange(hsv_frame, lower_color4, upper_color4)



if len(valid_labels) > 0:
                    print('yob')
                    largest_cluster_label = valid_labels[np.argmax(counts[labels != -1])]
                    # remove small clusters here ideally (DO)
                    largest_cluster_points = points[clustering.labels_ == largest_cluster_label]
                    # Compute centroid of the largest cluster
                    avg_point = np.mean(largest_cluster_points, axis=0)
                    cx, cy = int(avg_point[1]), int(avg_point[0])
                    
                    filtered_mask = np.zeros_like(mask2)  # Reset mask
                    filtered_mask[largest_cluster_points[:, 0], largest_cluster_points[:, 1]] = 255  # Keep only valid points
                else:
                    cx, cy = None, None  # No valid clusters detected """
                    filtered_mask = np.zeros_like(mask2)





            ### old code assuming colors:
            if color == 'Green' and object_type == 'Cube':
                print('GREEN')
                hue_min = 35
                hue_max = 85

                # Define increment step size
                s_step = 40
                v_step = 40
                #lower_color = np.array([75, 50, 180])  # Lower bound of green in HSV
                #upper_color = np.array([90, 200, 240]) # Upper bound of green in HSV

            elif color == 'Red' and object_type == 'Cube':
                print('RED')
                hue_min = 170
                hue_max = 180

                # Define increment step size
                s_step = 40
                v_step = 40

            elif color == 'Blue' and object_type == 'Cube':
                print('BLUE')
                hue_min = 100
                hue_max = 140

                # Define increment step size
                s_step = 40
                v_step = 40
                
            elif color == 'Green' and object_type == 'Sphere':
                print('GREEN')
                hue_min = 35
                hue_max = 85

                # Define increment step size
                s_step = 40
                v_step = 40

            elif color == 'Red' and object_type == 'Sphere':
                print('RED')
                hue_min = 170
                hue_max = 180

                # Define increment step size
                s_step = 40
                v_step = 40

            elif color == 'Blue' and object_type == 'Sphere':
                print('BLUE')
                hue_min = 100
                hue_max = 140

                # Define increment step size
                s_step = 40
                v_step = 40
                

            else:
                self.get_logger().warn("Unsupported color!")
                return False





# pickup service old code detection:
def arm_detection(self, object_type, color):
        # detection logic
        
        frame = self.latest_frame
        if frame is None:
            self.get_logger().warn("No frame received yet!")
            msg1 = PointStamped()
            msg1.point.x = 0.2
            msg1.point.y = 0.0
            msg1.point.z = 0.0
            return msg1
        print('frame received')
        img = frame.copy()
        height, width = img.shape[:2]
        camMatrixNew,roi, = cv2.getOptimalNewCameraMatrix(self.camMatrix, self.distCoeff,(width,height),1, (width,height))
        imgUndist = cv2.undistort(img,self.camMatrix, self.distCoeff, None, camMatrixNew)
       
        # Crop the undistorted image
        x, y, w, h = roi
        cropped_img = imgUndist[y:y+h-34, x:x+w]
        cv2.imwrite('/home/happy/img0.png', cropped_img)
        #cropped_img = cv2.bilateralFilter(cropped_img, d=20, sigmaColor=75, sigmaSpace=150)
        kernel = np.ones((5, 5), np.uint8)

        # Apply Erosion followed by Dilation (Opening operation)
        eroded_img = cv2.erode(cropped_img, kernel, iterations=1)
        dilated_img = cv2.dilate(eroded_img, kernel, iterations=1)
        cropped_img = dilated_img
        #height2, width2 = cropped_img.shape[:2]
        print('cropped:', cropped_img.shape)
        # Process the image to find the object
        if object_type == 'Cube' or object_type == 'Sphere':
            print(object_type)
            # Convert the image to HSV color space
            red_hue_min1 = 170
            red_hue_max1 = 180
            red_hue_min2 = 170
            red_hue_max2 = 180
            green_hue_min = 40
            green_hue_max = 80
            blue_hue_min = 100
            blue_hue_max = 140
            """ hue_vec = [[green_hue_min, green_hue_max],
                        [red_hue_min2, red_hue_max2],
                        [blue_hue_min, blue_hue_max]] """
                       
            # Define increment step size
            
            start = time.time()
            largest_cluster_sizes = []
            center_points = []
            mask_list = []
            RGB_arr = ['Red', 'Green', 'Blue']
            for i in range(0,3):
                print(i)
                hsv_frame = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

                if i == 0:
                    s_step = 20
                    v_step = 20
                    mask1 = self.createCombinedMask(s_step, v_step, red_hue_min1, red_hue_max1, hsv_frame)
                    mask2 = self.createCombinedMask(s_step, v_step, red_hue_min2, red_hue_max2, hsv_frame)
                    mask = cv2.bitwise_or(mask1, mask2)
                    
                    kernel = np.ones((10, 10), np.uint8)
                    maskk = mask.copy()
                    maskk = cv2.erode(maskk, kernel, iterations = 2)
                    maskk = cv2.morphologyEx(maskk, cv2.MORPH_OPEN, kernel)
                    cv2.imwrite('/home/happy/maskk.png', maskk)
                    mask = maskk

                elif i == 1:
                    s_step = 10
                    v_step = 10
                    mask = self.createCombinedMask(s_step, v_step, green_hue_min, green_hue_max, hsv_frame)
                    kernel = np.ones((10, 10), np.uint8)
                    mask = cv2.dilate(mask, kernel, iterations=3)
                elif i == 2:
                    s_step = 10
                    v_step = 10
                    mask = self.createCombinedMask(s_step, v_step, blue_hue_min, blue_hue_max, hsv_frame)

                #print(hue_min, hue_max)
                # Create a mask for the specified color
                
                
                """ masks = []
                for s_min in range(0, 256, s_step):
                    for v_min in range(0, 256, v_step):
                        lower = np.array([hue_min, s_min, v_min])
                        upper = np.array([hue_max, min(s_min + s_step - 1, 255), min(v_min + v_step - 1, 255)])
                        mask = cv2.inRange(hsv_frame, lower, upper)
                        masks.append(mask)
                final_mask = np.zeros_like(masks[0])
                for m in masks:
                    final_mask = cv2.bitwise_or(final_mask, m)
                mask = final_mask """
                #mask = cv2.bitwise_or(mask11, mask22, mask33,mask44)
                
                cv2.imwrite('/home/happy/img.png', cropped_img)
                mask_filename = f'/home/happy/mask_{i}.png'  # Name the file mask_0.png, mask_1.png, etc.
                cv2.imwrite(mask_filename, mask)
                #cv2.imwrite('/home/happy/mask.png', mask)
                

                # missing code to find the object center, clustering etc. add tomorrow
                points = np.column_stack(np.where(mask > 0))
                cx,cy = 0, 0 # maybe change this
                avg_point = 0
                mask2 = mask.copy()
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
                largest_cluster_sizes.append(cluster_size)
                center_points.append(avg_point)
                mask_list.append(filtered_mask)
            end = time.time()
            print('detection time:', end-start)
             
            max_index = np.argmax(largest_cluster_sizes)
            print('Biggest cluster:',RGB_arr[max_index])
            print('Cluster sizes:', largest_cluster_sizes)
            cx, cy = int(center_points[max_index][1]), int(center_points[max_index][0])

            #cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            #print(len(contours))
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
            if pos_y < 0:
                pos_y = pos_y + 1/100
            elif pos_y > 0:
                pos_y = pos_y - 1/100


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
            filtered_mask = mask_list[max_index]
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

            #cv2.imshow("cluster", filtered_mask)
            #cv2.imshow("Mask", mask)  # Show mask for debugging
            #cv2.imshow("frame_cropped", cropped_img)
            #cv2.waitKey(100)

             return msg
        elif object_type == 'Plush':
            # Handle plush object detection
            # Implement the logic for plush object detection here
            img = cv2.cvtColor(cropped_img,cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(img, (5, 5), 0)

            edges = cv2.Canny(blurred, 200, 250)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            output_image = frame.copy()
            centroids = []
            for contour in contours:
                if cv2.contourArea(contour) > 20:  # Filter by area to remove small contours
                    # Approximate the contour to a polygon
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)


                    M = cv2.moments(contour)
        
                    # Calculate the centroid (center) of the contour
                    if M["m00"] != 0:  # Avoid division by zero
                        cx = int(M["m10"] / M["m00"])  # X coordinate of the centroid
                        cy = int(M["m01"] / M["m00"])  # Y coordinate of the centroid
                    else:
                        cx, cy = 0, 0  # If area is zero (empty contour), set to (0, 0)
                    centroids.append([cx,cy])
                    # Draw the approximated polygon on the image
                    output_image = cv2.drawContours(output_image, [approx], -1, (0, 255, 0), 2)

            cx, cy = 0, 0
            for centroid in centroids:
                cx += centroid[0]
                cy += centroid[1]
            cx, cy = cx/len(centroids), cy/len(centroids)
            cx, cy = int(cx), int(cy)
            cv2.circle(output_image, (cx, cy), 5, (0, 0, 255), -1)  # Red dot for the centroid
            cv2.imshow('Edges', output_image)
            cv2.waitKey(10000)
            cv2.destroyAllWindows()
            h, w = cropped_img.shape[:2]
            xnew = h - cy 
            ynew = cx - w/2


            pos_x = xnew/h * 17.5/100 + 10/100  # KANSKE BORDE VARA TYP 17.6-17.7 ish
            pos_y = ynew/(w/2) * 25/200

            msg = PointStamped()
            msg.point.x = float(pos_x)
            msg.point.y = float(pos_y)
            msg.point.z = -0.13 # placeholder value
            self.get_logger().info(f"Object detected at: {msg.point.x}, {msg.point.y}, {msg.point.z}")
            return msg
        



def arm_detection2(self, object_type, color):
            # detection logic
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
                ### old code assuming colors:
                if color == 'Green' and object_type == 'Cube':
                    print('GREEN')
                    hue_min = 35
                    hue_max = 85

                    # Define increment step size
                    s_step = 10
                    v_step = 10
                    #lower_color = np.array([75, 50, 180])  # Lower bound of green in HSV
                    #upper_color = np.array([90, 200, 240]) # Upper bound of green in HSV

                elif color == 'Red' and object_type == 'Cube':
                    print('RED')
                    hue_min = 170
                    hue_max = 180

                    # Define increment step size
                    s_step = 10
                    v_step = 10

                elif color == 'Blue' and object_type == 'Cube':
                    print('BLUE')
                    hue_min = 100
                    hue_max = 140

                    # Define increment step size
                    s_step = 10
                    v_step = 10
                    
                elif color == 'Green' and object_type == 'Sphere':
                    print('GREEN')
                    hue_min = 35
                    hue_max = 85

                    # Define increment step size
                    s_step = 10
                    v_step = 10

                elif color == 'Red' and object_type == 'Sphere':
                    print('RED')
                    hue_min = 170
                    hue_max = 180

                    # Define increment step size
                    s_step = 40
                    v_step = 40

                elif color == 'Blue' and object_type == 'Sphere':
                    print('BLUE')
                    hue_min = 100
                    hue_max = 140

                    # Define increment step size
                    s_step = 40
                    v_step = 40
                    

                else:
                    self.get_logger().warn("Unsupported color!")
                    return False

                hsv_frame = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
                masks = []
                for s_min in range(20, 256-20, s_step):
                    for v_min in range(20, 256-20, v_step):
                        lower = np.array([hue_min, s_min, v_min])
                        upper = np.array([hue_max, min(s_min + s_step - 1, 255), min(v_min + v_step - 1, 255)])
                        mask = cv2.inRange(hsv_frame, lower, upper)
                        masks.append(mask)
                final_mask = np.zeros_like(masks[0])
                for m in masks:
                    final_mask = cv2.bitwise_or(final_mask, m)
                mask = final_mask
                #mask = cv2.bitwise_or(mask11, mask22, mask33,mask44)
                
                cv2.imwrite('/home/happy/img.png', cropped_img)
                cv2.imwrite('/home/happy/mask.png', mask)
                

                # missing code to find the object center, clustering etc. add tomorrow
                points = np.column_stack(np.where(mask > 0))
                cx,cy = 0, 0 # maybe change this
                avg_point = 0
                mask2 = mask.copy()
                if points.size > 0:
                    # Apply DBSCAN clustering
                    clustering = DBSCAN(eps=2, min_samples=300).fit(points)  # Adjust 'eps' based on pixel scale
                    
                    # Get labels and find the largest cluster
                    labels, counts = np.unique(clustering.labels_, return_counts=True)

                    # Ignore noise points (-1 label) and find the largest cluster
                    valid_labels = labels[labels != -1]
                    
                    filtered_clusters = []

                    for label in valid_labels:
                        cluster_points = points[clustering.labels_ == label]

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
                        print('Cluster size:', cluster_size)

                        # Compute centroid
                        avg_point = np.mean(largest_cluster_points, axis=0)
                        cx, cy = int(avg_point[1]), int(avg_point[0])

                        filtered_mask = np.zeros_like(mask2)
                        filtered_mask[largest_cluster_points[:, 0], largest_cluster_points[:, 1]] = 255

                    else:
                        cx, cy = None, None
                        filtered_mask = np.zeros_like(mask2)


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
                if pos_y < 0:
                    pos_y = pos_y + 1/100
                elif pos_y > 0:
                    pos_y = pos_y - 1/100


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

                #cv2.imshow("cluster", filtered_mask)
                #cv2.imshow("Mask", mask)  # Show mask for debugging
                #cv2.imshow("frame_cropped", cropped_img)
                #cv2.waitKey(100)


                return msg