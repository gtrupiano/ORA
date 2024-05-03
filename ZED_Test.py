import pyzed.sl as sl

def main():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Failed to open the camera")
        return

    # Enable positional tracking
    tracking_params = sl.PositionalTrackingParameters()
    err = zed.enable_positional_tracking(tracking_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Failed to enable positional tracking")
        zed.close()
        return

    # Capture a new frame
    runtime_parameters = sl.RuntimeParameters()
    zed.grab(runtime_parameters)

    # Retrieve the depth map
    depth = sl.Mat()
    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

    # Get the closest object distance
    closest_distance = depth.get_value(zed.get_width() // 2, zed.get_height() // 2)

    # Print the closest object distance
    print("Closest object distance: {:.2f} meters".format(closest_distance))

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()