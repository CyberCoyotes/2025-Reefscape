package frc.robot.subsystems.elevator;


public final class ElevatorConstants {

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
    public static final double POSITION_HOME_TOLERANCE = 0.00;
    public static final double MANUAL_MAX_SPEED_TESTING = 0.25;
    public static final double MANUAL_MAX_SPEED_PERFORMANCE = 0.50;
}