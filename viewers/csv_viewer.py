import cv2
import numpy as np
import time

def get_config():
    print("-+-+-+-+- Input Data -+-+-+-+-")
    # Input CSV file path
    csv_path = input("Enter the path to your CSV file: ")

    print("-+-+-+-+- Camera Resolution -+-+-+-+-")
    # Define preset resolutions
    resolutions = {
        "1": (1280, 720),
        "2": (640, 480),
        "3": (480, 320),
        "4": (346, 260),
        "5": (240, 180),
        "6": (128, 128)
    }

    # Resolution selection
    while True:
        print("0. Custom resolution...")
        for key, (w, h) in resolutions.items():
            print(f"{key}. {w}x{h}")
        
        choice = input("Select resolution: ")
        if choice == "0":
            res_input = input("Please enter camera resolution as 'AxB' without quotation marks: ")
            try:
                width, height = map(int, res_input.lower().split('x'))
                break
            except Exception as e:
                print("Invalid resolution format, example of valid input value: 640x480")
        elif choice in resolutions:
            width, height = resolutions[choice]
            break
        else:
            continue
    
    print(f"Using resolution: {width}x{height}")
    return csv_path, width, height

def stream_events_fixed(csv_path, fixed_interval_ms, max_events_frame=300000):
    """
    Generator that accumulates events into a pre-allocated NumPy array (buffer)
    and yields the events collected every fixed time interval (in milliseconds).
    """
    # Pre-allocate a buffer for one frame's events.
    event_buffer = np.empty((max_events_frame, 4), dtype=np.int32)
    count = 0
    next_display_time = time.time() + fixed_interval_ms / 1000.0

    with open(csv_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            try:
                t_str, x_str, y_str, p_str = line.split(',')
                t, x, y, p = int(t_str), int(x_str), int(y_str), int(p_str)
            except ValueError:
                continue  # Skip invalid lines

            # If buffer is full, yield what we have and reset count.
            if count >= max_events_frame:
                yield event_buffer[:count]
                count = 0

            event_buffer[count] = [t, x, y, p]
            count += 1

            # Check if it's time to update the display.
            current_time = time.time()
            if current_time >= next_display_time:
                # Yield the current events and reset the buffer.
                yield event_buffer[:count]
                count = 0
                next_display_time = current_time + fixed_interval_ms / 1000.0

        # Yield any remaining events after the file is done.
        if count > 0:
            yield event_buffer[:count]

def main():
    csv_path, width, height = get_config()
    fixed_interval_ms = 33  # Fixed display update interval ~30 FPS

    cv2.namedWindow('Event Video', cv2.WINDOW_NORMAL)

    print("-+-+-+-+- Streaming Captured Events -+-+-+-+-")
    for events_arr in stream_events_fixed(csv_path, fixed_interval_ms):
        # Create a blank frame with the chosen resolution.
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        if events_arr.size > 0:
            # Extract x, y, and polarity (columns 1, 2, 3)
            x = events_arr[:, 1].astype(np.int32)
            y = events_arr[:, 2].astype(np.int32)
            p = events_arr[:, 3].astype(np.int32)
            
            # Filter valid coordinates.
            valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
            x = x[valid]
            y = y[valid]
            p = p[valid]
            
            # Process events: positive events in white, negative events in red.
            pos_mask = (p == 1)
            neg_mask = (p == 0)
            frame[y[pos_mask], x[pos_mask]] = (255, 255, 255)
            frame[y[neg_mask], x[neg_mask]] = (255, 0, 0)

        cv2.imshow('Event Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()