void ODriveMovement(double verticalVelocity, double horizontalVelocity) {
    // Convert joystick inputs to motor velocities
    double leftMotorVel = verticalVelocity - horizontalVelocity;
    double rightMotorVel = verticalVelocity + horizontalVelocity;

    // Scale the velocities to fit within the acceptable range
    const double maxVelocity = 1.0; // Define your maximum velocity here
    const double scale = max(abs(leftMotorVel), abs(rightMotorVel)) / maxVelocity;
    if (scale > 1.0) {
        // Scale down both velocities to fit within the range
        leftMotorVel /= scale;
        rightMotorVel /= scale;
    }

    // Convert velocities to appropriate units for ODrive control (e.g., RPM or encoder ticks per second)
    // Assuming leftMotorVel and rightMotorVel are in the range [-1, 1]

    // Example conversion to RPM:
    int leftMotorRPM = leftMotorVel * maxVelocity * MAX_RPM; // MAX_RPM is the maximum RPM of your motor
    int rightMotorRPM = rightMotorVel * maxVelocity * MAX_RPM;

    // Now send the velocities to the motors using CAN messages
    // Assuming you have functions to send velocity commands via CAN messages
    // Make sure to check the direction of rotation and adjust if needed based on your motor configuration
    // You may need to invert the velocities depending on how your motors are oriented
    // For instance, if a positive velocity corresponds to forward motion for the left motor,
    // but backward motion for the right motor, you'll need to invert the velocity for the right motor.

    // Example:
    // Send velocity commands to left and right motors
    // Assuming ODRV0_NODE_ID and ODRV1_NODE_ID are correctly defined
    CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x0D), 0, 4, (byte*)&leftMotorRPM);
    CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x0D), 0, 4, (byte*)&rightMotorRPM);
}
