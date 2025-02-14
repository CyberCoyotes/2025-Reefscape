// WristConstants.java
package frc.robot.subsystems.wrist;

public final class WristConstants {
    // CAN ID and hardware config
    public static final int WRIST_ID = 20;
    public static final int WRIST_ENCODER_ID = 14;
    public static final String canBus = "rio";
    public static final double GEAR_RATIO = 50.0; // TODO Determine the actual gear ratio; suggested 75.0
    public static final double ENCODER_TO_MECHANISM_RATIO = 1.0; // TODO Determine the actual ratio

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 10.0;
    public static final double SUPPLY_CURRENT_LIMIT = 10.0;

    // Motion constraints
    public static final double MAX_POSITION = 0.50;
    public static final double MIN_POSITION = 0.0;
    public static final double WRIST_POSE_TOLERANCE = 0.02;

    // Constants 
    public static final double MOTION_MAGIC_VELOCITY = 10.0; // rotations per second
    public static final double MOTION_MAGIC_ACCELERATION = 20.0; // rotations per second^2
    public static final double MOTION_MAGIC_JERK = 100.0; // rotations per second^3
    public static final double VOLTAGE_FEEDFORWARD = 0.0; // Volts to add to overcome gravity
    
    // Wrist angle limits in rotations // Move these to WristCconstants
    public static final double MIN_ROTATION = -0.25; // -90 degrees
    public static final double MAX_ROTATION = 0.50;  // 90 degrees

    
    public static final class Hardware {
        public static final int WRIST_ID = 20;
        public static final int WRIST_ENCODER_ID = 14;
        public static final String CAN_BUS = "rio";
        public static final double GEAR_RATIO = 50.0;
        public static final double ENCODER_TO_MECHANISM_RATIO = 1.0;
        public static final double STATOR_CURRENT_LIMIT = 10.0;
        public static final double SUPPLY_CURRENT_LIMIT = 10.0;
    }

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



    // Named positions
    public static final class Positions {
        public static final double LOAD_CHORAL = 0.0;
        public static final double L1 = 0.10;
        public static final double L2 = 0.20;
        public static final double L3 = 0.30;
        public static final double L4 = 0.40;
        public static final double ELEVATOR_SAFE = 0.26;
    }
}