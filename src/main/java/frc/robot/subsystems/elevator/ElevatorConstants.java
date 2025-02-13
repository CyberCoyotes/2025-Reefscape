package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
    public static final int ELEVATOR_LEAD_ID = 24; // Change to your CAN ID
    public static final int ELEVATOR_FOLLOW_ID = 23; // Change to your CAN ID
    
    // Motion Magic
    public static final double CRUISE_VELOCITY = 160; // Rotations per second
    public static final double ACCELERATION = 160; // Rotations per second^2
    public static final double JERK = 500; // Rotations per second^3
    
    // PID Values
    public static final double kP = 4.0;
    public static final double kI = 0.01;
    public static final double kD = 0.10;
    public static final double kS = 0.50;
    public static final double kV = 0.12;
    public static final double kG = 0.50;

    
    // Soft Limits (in rotations)
    public static final double MAX_HEIGHT = 1;
    public static final double MIN_HEIGHT = 0;
    
    // Preset Positions (in rotations)
    public static final double BASE_POSE =  0.00;
    public static final double L1_POSE =    0.10;
    public static final double L2_POSE =    0.15;
    public static final double L3_POSE =    0.27;
    public static final double L4_POSE =    0.27;
    
    // Control Parameters
    public static final double DEADBAND = 0.02;
    public static final double POSITION_TOLERANCE = 0.02;
}