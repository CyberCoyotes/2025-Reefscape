// WristConstants.java
package frc.robot.subsystems.wrist;

public final class WristMotorConstants {
    // CAN ID and hardware config
    public static final int MOTOR_CAN_ID = 20;
    public static final double GEAR_RATIO = 75.0; // TODO Determine the actual gear ratio
    public static final double ENCODER_TO_MECHANISM_RATIO = 1.0; // TODO Determine the actual ratio

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 10.0;
    public static final double SUPPLY_CURRENT_LIMIT = 10.0;

    // Motion constraints
    public static final double MAX_POSITION = 0.20;
    public static final double MIN_POSITION = 0.0;
    public static final double POSITION_TOLERANCE = 0.02;

    // PID and FF Gains
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double kA = 0.0;

    public static final double LOADING = 0.0;
    public static final double SCORE_L1 = 0.05;
    public static final double SCORE_L2 = 0.10;
    public static final double SCORE_L3 = 0.15;
    public static final double SCORE_L4 = 0.20;

}