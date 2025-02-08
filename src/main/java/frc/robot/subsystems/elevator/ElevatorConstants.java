package frc.robot.subsystems.elevator;

public class ElevatorConstants {

    // Initialize motor with device ID and CAN bus name
    public final static int LEFT_ELEVATOR_ID = 24;
    public final static int RIGHT_ELEVATOR_ID = 23;

    // Elevator Current limits
    public static final double STATOR_CURRENT_LIMIT = 80.0; // TODO Adjust to actual values after testing
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; // TODO Adjust to actual values after testing

    // Motion constraints
    public static final double MAX_POSITION = 20.0; // According to initial testing should be 26-30 ish, sorta kinda
    public static final double MIN_POSITION = 0.0;
    public static final double POSITION_TOLERANCE = MAX_POSITION * .05;

    // PID and FF Gains
    public static final class Gains {
        public static final double kP = 0.20;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kS = 0.05;
        public static final double kV = 0.12;
        public static final double kA = 0.00;
        public static final double kG = 0.00;
    }

}
