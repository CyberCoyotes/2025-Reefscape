// WristConstants.java
package frc.robot.subsystems.wrist;

public final class WristConstants {
    // CAN ID and hardware config

    public static final double GEAR_RATIO = 50.0; // TODO Determine the actual gear ratio; suggested 75.0
    public static final double ENCODER_TO_MECHANISM_RATIO = 1.0; 
    
    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 10.0;
    public static final double SUPPLY_CURRENT_LIMIT = 10.0;

    // Motion constraints
    public static final double MAX_POSITION = 0.50;
    public static final double MIN_POSITION = 0.0;
    public static final double POSE_TOLERANCE = 0.02;

    // Constants 
    public static final double MOTION_MAGIC_VELOCITY = 10.0; // rotations per second
    public static final double MOTION_MAGIC_ACCELERATION = 20.0; // rotations per second^2
    public static final double MOTION_MAGIC_JERK = 100.0; // rotations per second^3
    public static final double VOLTAGE_FEEDFORWARD = 0.0; // Volts to add to overcome gravity
    
    public static final double REVERSE_LIMIT = 0.00;
    public static final double FORWARD_LIMIT = 0.50;  // 90 degrees
        
    // PID and FF Gains
    public static final class Gains {
        public static final double kP = 4.8;
        public static final double kI = 0.0;
        public static final double kD = 0.01;
        public static final double kG = 0.00;
        public static final double kV = 0.12;
        public static final double kS = 0.25;
        public static final double kA = 0.01;
    }



    // Named Motor positions
    public static final class Positions {

        public static final double SAFE = 0.26;
        public static final double LOAD_CORAL = 0.0;
        public static final double GRAB_ALGAE = 0.40;
        public static final double L1 = 0.05;
        public static final double L2 = 0.10; // 0.20
        public static final double L3 = 0.30;
        public static final double L4 = 0.40;

    }

    public static final class EncoderPose {
        public static final double SAFE = 0.26;
        public static final double LOAD_CORAL = 0.0;
        public static final double GRAB_ALGAE = 0.40;
        public static final double L1 = 0.05;
        public static final double L2 = 0.20;
        public static final double L3 = 0.30;
        public static final double L4 = 0.40;
    }
}