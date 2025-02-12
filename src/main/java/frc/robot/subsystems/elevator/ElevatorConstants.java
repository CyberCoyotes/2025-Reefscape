package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
    public static final int ELEVATOR_LEAD_ID = 23; // Change to your CAN ID
    public static final int ELEVATOR_FOLLOW_ID = 24; // Change to your CAN ID
    
    // Motion Magic
    public static final double CRUISE_VELOCITY = 80; // Rotations per second
    public static final double ACCELERATION = 160; // Rotations per second^2
    public static final double JERK = 1600; // Rotations per second^3
    
    // PID Values
    public static final double kP = 0.0; // 24
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.00;   // 0.12
    public static final double kS = 0.00;   // 0.25
    public static final double kG = 0.12;

    
    // Soft Limits (in rotations)
    public static final double MAX_HEIGHT = 30;
    public static final double MIN_HEIGHT = 0;
    
    // Preset Positions (in rotations)
    public static final double GROUND_POSITION = 0;
    public static final double MIDDLE_POSITION = 10;
    public static final double HIGH_POSITION = 20;
    
    // Control Parameters
    public static final double DEADBAND = 0.1;
    public static final double POSITION_TOLERANCE = 0.05;
}