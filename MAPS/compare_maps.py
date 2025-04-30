import math


# -------- File Paths --------
EXPLORATION_FILE = 'map_exploration.tsv'
GROUND_TRUTH_FILE = 'exploration_1_2.tsv'  # EDIT HERE
THRESHOLD = 20  # [cm] Object matches with same type and distance <= THRESHOLD
# ----------------------------


def load_map(file_path):
    """
    Load the map file and return a list of detections.
    Each detection is represented as a tuple: (type, x, y).
    """
    detections = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split('\t')
            if len(parts) >= 3:  # Ensure there are at least 3 parts (TYPE, X, Y)
                obj_type = parts[0]
                x = int(parts[1])
                y = int(parts[2])
                detections.append((obj_type, x, y))  # Ignore theta completely
            else:
                print(f"Invalid line in {file_path}: {line.strip()}")
    return detections


def calculate_distance(x1, y1, x2, y2):
    """
    Calculate the Euclidean distance between two points (x1, y1) and (x2, y2).
    """
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def compare_maps(detected_map, ground_truth_map, threshold):
    """
    Compare the detected map with the ground truth map.
    Returns a list of results indicating whether each detection is within the threshold distance.
    """
    results = []
    matched_count = 0
    for detected in detected_map:
        obj_type, x, y = detected
        matched = False
        for ground_truth in ground_truth_map:
            gt_type, gt_x, gt_y = ground_truth
            if obj_type == gt_type:  # Match type only
                distance = calculate_distance(x, y, gt_x, gt_y)
                if distance <= threshold:
                    matched = True
                    break
        if matched:
            matched_count += 1
        results.append((detected, matched, distance if matched else None))
    return results, matched_count


def main():
    # Load maps
    exploration_map = load_map(EXPLORATION_FILE)
    ground_truth_map = load_map(GROUND_TRUTH_FILE)

    # Compare maps
    results, matched_count = compare_maps(exploration_map, ground_truth_map, threshold=THRESHOLD)

    # Print results
    print("---- Comparison Results:")
    print(f"NOTE: Match means same type and within {THRESHOLD} cm")
    for detection, matched, distance in results:
        obj_type, x, y = detection
        if matched:
            print(f"{obj_type} at ({x}, {y}): Matched (Error: {distance:.2f} cm)")
        else:
            print(f"{obj_type} at ({x}, {y}): Not Matched")

    # Print statistics
    total_detected = len(exploration_map)
    total_ground_truth = len(ground_truth_map)
    print("\n---- Statistics:")
    print(f"Matched: {matched_count} out of {total_detected} detected")
    print(f"Matched: {matched_count} out of {total_ground_truth} ground truth")
    print("-----------------")


if __name__ == "__main__":
    main()