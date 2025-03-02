package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
    
    public static final double GEAR_RATIO = 9.0;

    // Control Parameters
    public static final double DEADBAND = 0.02;
    public static final double POSITION_TOLERANCE = 0.02;

    /******************************
     *  Motion Magic 
     ******************************/ 
    // Tested 160 previously
    public static final double CRUISE_VELOCITY = 120; // Rotations per second
    // Tested 160 previously
    public static final double ACCELERATION = 120; // Rotations per second^2
    
    public static final double JERK = 500; // Rotations per second^3
    
    // PID Values
    // kP tested 8 previously
    public static final double kP = 6.00;
    public static final double kI = 0.01;
    public static final double kD = 0.10;
    public static final double kS = 0.50;
    public static final double kV = 0.12;
    public static final double kG = 0.50;

    
    // Soft Limits (in rotations)
    // This will need updating when new build
    public static final double FORWARD_LIMIT = 4.67;
    public static final double REVERSE_LIMIT = 0;   
    
    public class TestMode {
        
        public static final double kP = 1.0;
        public static final double kI = 0.01;
        public static final double kD = 0.10;
        public static final double kS = 0.50;
        public static final double kV = 0.12;
        public static final double kG = 0.50;

        public static final double CRUISE_VELOCITY = 40; // Rotations per second
        public static final double ACCELERATION = 40; // Rotations per second^2
        public static final double JERK = 200; // Rotations per second^3
    }

}