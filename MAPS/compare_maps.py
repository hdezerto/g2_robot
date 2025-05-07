import math


# -------- File Paths --------
EXPLORATION_FILE = 'map_exploration.tsv'
GROUND_TRUTH_FILE = 'exploration_hard.tsv'  # EDIT HERE
THRESHOLD = 30  # [cm] Object matches with same type and distance <= THRESHOLD
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
                try:
                    x = int(parts[1])
                    y = int(parts[2])
                    detections.append((obj_type, x, y))  # Ignore theta completely
                except ValueError:
                     print(f"Invalid coordinates in {file_path}: {line.strip()}")
            else:
                print(f"Invalid line in {file_path}: {line.strip()}")
    return detections


def calculate_distance(x1, y1, x2, y2):
    """
    Calculate the Euclidean distance between two points (x1, y1) and (x2, y2).
    """
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def compare_maps(detection_map, ground_truth_map, threshold):
    """
    Compare the detected map with the ground truth map.
    Returns a list of results indicating whether each detection is within the threshold distance.
    Also counts object matches (types 1, 2, 3) and box matches (type B) separately.
    """
    results = []
    matched_objects = 0
    matched_boxes = 0
    matched_gt_indices = set() # Keep track of matched ground truth items to avoid double counting

    for detection in detection_map:
        obj_type, x, y = detection
        best_match_distance = float('inf')
        match_found = False
        best_gt_index = -1

        for i, ground_truth in enumerate(ground_truth_map):
            if i in matched_gt_indices:
                continue # Skip already matched ground truth items

            gt_type, gt_x, gt_y = ground_truth
            if obj_type == gt_type:  # Match type only
                distance = calculate_distance(x, y, gt_x, gt_y)
                if distance <= threshold and distance < best_match_distance: # Check if within threshold and closer than previous best
                    best_match_distance = distance
                    match_found = True
                    best_gt_index = i

        if match_found:
            matched_gt_indices.add(best_gt_index) # Mark this ground truth item as matched
            if obj_type in ['1', '2', '3']:
                matched_objects += 1
            elif obj_type == 'B':
                matched_boxes += 1
            results.append((detection, True, best_match_distance))
        else:
            results.append((detection, False, None))

    return results, matched_objects, matched_boxes


def compare_maps(detection_map, ground_truth_map, threshold):
    """
    Compare maps by finding the best detection for each ground truth item.
    Returns a list of results indicating whether each *detection* was matched,
    along with counts of matched objects and boxes based on the ground-truth-first matching.
    """
    matched_objects = 0
    matched_boxes = 0
    matched_detection_indices = set() # Keep track of matched detections to avoid double counting
    # Store details of successful matches: {detection_index: (ground_truth, distance)}
    successful_matches = {}

    # --- Phase 1: Find best detection for each ground truth item ---
    for i, ground_truth in enumerate(ground_truth_map):
        gt_type, gt_x, gt_y = ground_truth
        best_match_distance = float('inf')
        best_detection_index = -1

        for j, detection in enumerate(detection_map):
            # Skip detections that have already been matched
            if j in matched_detection_indices:
                continue

            obj_type, x, y = detection
            if obj_type == gt_type:  # Match type only
                distance = calculate_distance(x, y, gt_x, gt_y)
                # Check if within threshold and closer than previous best for this GT item
                if distance <= threshold and distance < best_match_distance:
                    best_match_distance = distance
                    best_detection_index = j

        # If a best match was found for this ground truth item
        if best_detection_index != -1:
            # Mark this detection as matched
            matched_detection_indices.add(best_detection_index)
            # Store the match details, keyed by the detection index
            successful_matches[best_detection_index] = (ground_truth, best_match_distance)
            # Increment the appropriate counter based on the ground truth type
            if gt_type in ['1', '2', '3']:
                matched_objects += 1
            elif gt_type == 'B':
                matched_boxes += 1

    # --- Phase 2: Build results list based on detections ---
    results = []
    for j, detection in enumerate(detection_map):
        if j in successful_matches:
            # This detection was matched to a ground truth item
            _, distance = successful_matches[j]
            results.append((detection, True, distance))
        else:
            # This detection was not matched to any ground truth item
            results.append((detection, False, None))

    # Return the results list (per detection) and the counts (from GT matching)
    return results, matched_objects, matched_boxes



def main():
    # Load maps
    exploration_map = load_map(EXPLORATION_FILE)
    ground_truth_map = load_map(GROUND_TRUTH_FILE)

    # Separate ground truth items by type for counting
    gt_objects = [item for item in ground_truth_map if item[0] in ['1', '2', '3']]
    gt_boxes = [item for item in ground_truth_map if item[0] == 'B']

    # Separate detectiosn items by type for counting
    det_objects = [item for item in exploration_map if item[0] in ['1', '2', '3']]
    det_boxes = [item for item in exploration_map if item[0] == 'B']

    # Compare maps
    results, matched_objects, matched_boxes = compare_maps(exploration_map, ground_truth_map, threshold=THRESHOLD)

    # Print results
    print("---- Comparison Results:")
    print(f"NOTE: Each ground truth item is matched to the closest detection of the same type within {THRESHOLD} cm.")
    for detection, matched, distance in results:
        obj_type, x, y = detection
        if matched:
            print(f"{obj_type} at ({x}, {y}): Matched (Error: {distance:.2f} cm)")
        else:
            print(f"{obj_type} at ({x}, {y}): Not Matched")

    # Print statistics
    total_detected = len(exploration_map)
    total_ground_truth_objects = len(gt_objects)
    total_ground_truth_boxes = len(gt_boxes)
    total_matched = matched_objects + matched_boxes

    print("\n---- Statistics:")
    print(f"Ground Truth: {total_ground_truth_objects + total_ground_truth_boxes} ({total_ground_truth_objects} objects, {total_ground_truth_boxes} boxes)")
    print(f"Detected: {total_detected} ({len(det_objects)} objects, {len(det_boxes)} boxes)\n")
    
    print(f"MATCHED: {total_matched} ({matched_objects} objects, {matched_boxes} boxes)")
    print("-----------------")


if __name__ == "__main__":
    main()