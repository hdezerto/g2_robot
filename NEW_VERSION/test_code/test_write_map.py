
import os

# Test the write_map_file function
if __name__ == "__main__":
    # Create a dummy class to simulate the behavior
    class TestWriteMap:
        def __init__(self):
            self.detected_objects = [
                (1.2334, 4.56, 1),
                (7.8934, 0.12, 2),
                (3.4534, 6.78, 3)
            ]
            self.detected_boxes = [
                (2.3434, 5.67, 45),
                (8.9034, 1.23, 90),
                (4.5634, 7.89, 135)
            ]

        def write_map_file(self):
            file_name = "map_file.txt"
            current_directory = os.getcwd()  # Get the current working directory

            with open(file_name, 'w') as file:
                # Write the objects to the file
                for x, y, category in self.detected_objects:
                    file.write(f"{category}\t{x:.2f}\t{y:.2f}\t0\n")  # Angle is 0 for objects

                # Write the boxes to the file
                for x, y, theta in self.detected_boxes:
                    file.write(f"B\t{x:.2f}\t{y:.2f}\t{theta:.0f}\n")  # Use theta for the angle

            print(f"Map file '{file_name}' has been written successfully to '{current_directory}'.")

    # Instantiate the test class
    test_instance = TestWriteMap()

    # Call the write_map_file function
    test_instance.write_map_file()

    # Inform the user that the test is complete
    print("Test complete. Check the 'map_file.txt' for correctness.")