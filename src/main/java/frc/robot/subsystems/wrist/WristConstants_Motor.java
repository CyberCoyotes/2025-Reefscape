package frc.robot.subsystems.wrist;

public final class WristConstants_Motor {
public static final int MOTOR_CAN_ID = 20;
public static final double GEAR_RATIO = 75.0; 

// Current limits
public static final double STATOR_CURRENT_LIMIT = 40.0;
public static final double SUPPLY_CURRENT_LIMIT = 40.0;

// Motion constraints
public static final double MAX_POSITION = 0.20;  // rotations
public static final double MIN_POSITION = 0.0;   // rotations
public static final double POSITION_TOLERANCE = 0.02;  // rotations

// Motion Magic settings
public static final class MotionMagic {
    public static final double CRUISE_VELOCITY = 0.5;    // rotations per second
    public static final double ACCELERATION = 1.0;       // rotations per second^2
    public static final double JERK = 10.0;             // rotations per second^3
}

// PID and FF Gains
public static final class Gains {
    // Start conservative with Motion Magic
    public static final double kP = 5.0;    // Voltage per rotation error
    public static final double kI = 0.0;    // Voltage per rotation-second error
    public static final double kD = 0.2;    // Voltage per rotation/second error
    public static final double kG = 0.25;   // Voltage to overcome gravity (when horizontal)
    public static final double kS = 0.25;   // Voltage to overcome static friction
    public static final double kV = 0.12;   // Voltage per rotation/second
    public static final double kA = 0.01;   // Voltage per rotation/second^2
}

// Named positions in rotations
public static final class Positions {
    public static final double LOAD_CHORAL = 0.0;
    public static final double ELEVATOR_SAFE = 0.02;
    public static final double L1 = 0.05;
    public static final double L2 = 0.10;
    public static final double L3 = 0.15;
    public static final double L4 = 0.20;
}
}