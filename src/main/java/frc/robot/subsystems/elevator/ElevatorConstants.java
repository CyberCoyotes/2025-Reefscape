package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
    
    public static final double GEAR_RATIO = 9.0;
    // Motion Magic
    public static final double CRUISE_VELOCITY = 160; // Rotations per second
    public static final double ACCELERATION = 160; // Rotations per second^2
    public static final double JERK = 500; // Rotations per second^3
    
    // PID Values
    public static final double kP = 4.00;
    public static final double kI = 0.01;
    public static final double kD = 0.10;
    public static final double kS = 0.50;
    public static final double kV = 0.12;
    public static final double kG = 0.50;

    
    // Soft Limits (in rotations)
    public static final double FORWARD_LIMIT = 2.50;
    public static final double REVERSE_LIMIT = 0;
    
    // Preset Positions (in rotations)
    public static final double HOME_POSE =  0.00;
    public static final double L1_POSE =    0.10;
    public static final double L2_POSE =    0.89;
    public static final double L3_POSE =    2.27;
    public static final double L4_POSE =    (FORWARD_LIMIT * 0.95); // TODO: Check this value, likely about 50
    
    //
        // Motion Magic
    public class TestMode {
        public static final double CRUISE_VELOCITY = 20; // Rotations per second
        public static final double ACCELERATION = 20; // Rotations per second^2
        public static final double JERK = 100; // Rotations per second^3
        
        // PID Values
        public static final double kP = 1.0;
        public static final double kI = 0.01;
        public static final double kD = 0.10;
        public static final double kS = 0.50;
        public static final double kV = 0.12;
        public static final double kG = 0.50;
    }

    // Control Parameters
    public static final double DEADBAND = 0.02;
    public static final double POSITION_TOLERANCE = 0.02;
}