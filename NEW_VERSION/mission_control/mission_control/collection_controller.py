#!/usr/bin/env python


"""

COLLECTION LOGIC:

1. Initialization:
    - Publish the workspace to RViz (just once).
    - Initialize the 'mapping' node, which should:
        - Initialize the occupancy grid map with the workspace file.
        - Subscribe to the /scan topic and update the map with Lidar data.
    - Subscribe to the /detections topic, which will be used to receive the positions of detected 
    objects/boxes/obstacles from the 3D camera.
    - Extract the objects and boxes from the map file (in separate lists).
    

2. Start collection (ONLY of objects extracted from the map file. 0 points otherwise):
    - Publish the boxes and remaining objects to RViz.
    - Add the detected objects/boxes/obstacles to the occupancy grid map and publish it to RViz.
    - Select the next closest object from the list, compute a path and move to a pick position. Publish the path to RViz.
    - While moving, when something is published to /detections:
        - If it is an object/box, ignore it. We only care about the object we are moving to.
        - If it is a new obstacle, add it to the obstacles list and stop the robot. Recompute the path to the object (now with
          the new obstacle in the occupancy grid map) and move to it if a path is found.
    - When the pick position is reached, reobserve the object and pick it.
    - Pick the object. If the pick is successful, remove the object from the list and publish the updated list to RViz.
    - Select the closest box and move to a place position.

3. End collection (when no more objects remain)


"""




