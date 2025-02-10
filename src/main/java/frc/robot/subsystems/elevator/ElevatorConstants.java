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
        public static final double kP = 0.10;   // 0.80
        public static final double kI = 0.00;
        public static final double kD = 0.00;   // 0.10
        public static final double kS = 0.00;   // 2.50
        public static final double kV = 0.12;   // 0.12
        public static final double kA = 0.00;
        public static final double kG = 0.00;
    }

    /*
    | kP    | kI    | kD    | kS    | kV    | kA    | kG    | Notes
    ----------------------------------------------------------------------------
    | 0.20  | 0.00  | 0.00  | 0.05  | 0.12  | 0.00  | 0.00  | Not enough power

    Adding motion magic CRUISE and ACCELARATION values worked!
    */
    public static final class MotionMagic {
        public static final double CRUISE_VELOCITY = 40;
        public static final double ACCELERATION = 40;
        public static final double JERK = 100; // 400 suggested
    }

}
