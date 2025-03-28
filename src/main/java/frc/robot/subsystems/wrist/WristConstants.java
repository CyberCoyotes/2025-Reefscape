// WristConstants.java
package frc.robot.subsystems.wrist;

/*
The Magnet Encoder Offset represents the rotational offset of the absolute encoder in rotations (not degrees).
What does 0.16 mean?
The offset is given in rotations (where 1.0 = one full revolution = 360 degrees).
Your value of 0.16 means the encoder is offset by: 0.16 × 360 = 57.6

This means that when the encoder reads "0", the actual position of the mechanism is 57.6 degrees ahead.
 */
public final class WristConstants {
    // CAN ID and hardware config
    // If you need to add CAN ID here, uncomment and set value
    // public static final int WRIST_MOTOR_CAN_ID = 0; 

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 70.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    // Configuration constants
    public static final double GEAR_RATIO = 80.0;
    public static final double REVERSE_LIMIT = 0.0; // In rotations
    public static final double FORWARD_LIMIT = 22.0; // In rotations
    public static final double INCREMENT = 0.50; // Using the value from subsystem as it's likely more tested
    public static final double TOLERANCE = 0.04; // Increased from 0.02 -> 0.04 Position Tolerance in rotations

    // Motion Magic parameters
    public static final double VELOCITY = 400.0;
    public static final double ACCELERATION = 100.0;
    public static final double JERK = 600.0;

    // Gravity compensation
    public static final double VOLTAGE_FEEDFORWARD = 0.0; // Volts to add to overcome gravity

    // PID and FF Gains for primary use (Slot0)
    public static final class Slot0 {
        public static final double kP = 10.0;
        public static final double kI = 0.00;
        public static final double kD = 0.20;
        public static final double kV = 0.00;
        public static final double kS = 0.00;
        public static final double kA = 0.00;
        public static final double kG = 0.30;

    }

    // Alternative PID and FF Gains (Slot1) - Keeping for reference or alternative tuning
    public static final class Slot1 {
        public static final double kP = 5.0;
        public static final double kI = 0.0;
        public static final double kD = 0.01;
        public static final double kG = 0.30;
        public static final double kV = 0.12;
        public static final double kS = 0.25;
        public static final double kA = 0.01;
    }

    // Named Motor positions
    public static final class Positions {
        /* 
        * Physically the end effector of the wrist should be resting against the elevator at the start of the match (START)
        * This position should not be used during a match after the start.
        * The STOWED position is currently not in use, but could be used to stow the wrist for transport as it was with previous configiaration
        * L1 is not a position currently in use, but could be used with testing differet scoring heights
        * L2 was set to 2.15 with the previous configuration
        * L3 should be the same as L2
        * L4 Approximately 6.0 and found to be near vertical in classroom
        * INTAKE CORAL is a new pose and estimate between L4 and PICK ALGAE 10.0
        * PICK ALGAE was previously 14.0 
        * SCORE ALGAE was previously 19.0
        */
        
        public static final double START = 0.0;
        public static final double STOWED = 0.0;
        public static final double SAFE = 0.26;
        // public static final double LOAD_CORAL = 0.0;
        public static final double GRAB_ALGAE = 0.40;
        public static final double L1 = 0.5;
        public static final double L2 = 1.75;
        public static final double L3 = 1.75;
        public static final double L4 = 2.94; // Previously 4.2
        public static final double TRAVEL = 5.0; // Halfway between L4 and PICK_ALGAE
        public static final double INTAKE_CORAL = 12.25;
        public static final double PICK_ALGAE = 14.0;
        public static final double SCORE_ALGAE = 18.0;
    }
    
} // end WristConstants