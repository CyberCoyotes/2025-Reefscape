package frc.robot.subsystems.elevator;


public final class ElevatorConstants {
    
    // Soft Limits (in rotations)
    public static final double MAX_HEIGHT = 0.53;
    public static final double MIN_HEIGHT = 0;

    
    // Preset Positions (in rotations)
    public static final double BASE_POSE =  0.00;
    public static final double L1_POSE =    0.00;
    public static final double L2_POSE =    0.09;
    public static final double L3_POSE =    0.27;
    public static final double L4_POSE =    0.45;
    
    //
        // Motion Magic
    public class TestMode {
        public static final double CRUISE_VELOCITY = 40; // Rotations per second
        public static final double ACCELERATION = 40; // Rotations per second^2
        public static final double JERK = 100; // Rotations per second^3
        
        // PID Values
        public static final double kP = 4.0;
        public static final double kI = 0.01;
        public static final double kD = 0.10;
        public static final double kS = 0.50;
        public static final double kV = 0.12;
        public static final double kG = 0.50;
    }

    // Control Parameters
    public static final double DEADBAND = 0.02;
    public static final double POSITION_TOLERANCE = 0.02;
    public static final double POSITION_HOME_TOLERANCE = 0.00;
    public static final double MANUAL_MAX_SPEED_TESTING = 0.25;
    public static final double MANUAL_MAX_SPEED_PERFORMANCE = 0.50;
}