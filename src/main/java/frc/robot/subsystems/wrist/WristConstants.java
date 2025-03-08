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

    // public static final double MAGNET_ENCODER_OFFSET = 0.16064453125; // From
    // Phoenix Tuner X

    // public static final double GEAR_RATIO = 5.0; // Determine the actual gear
    // ratio; // 50
    // public static final double ENCODER_TO_MECHANISM_RATIO = 1.0;

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double VOLTAGE_FEEDFORWARD = 0.0; // Volts to add to overcome gravity

    // Configuration constants
    public static final double GEAR_RATIO = 80.0;
    public static final double REVERSE_LIMIT = 0.0; // In rotations
    public static final double FORWARD_LIMIT = 20.0; // In rotations
    public static final double INCREMENT = 1.00;
    public static final double TOLERANCE = 0.02; // Position Tolerance in rotations
    public static final double VELOCITY = 80.0; // Motion Magic rotations per second
    public static final double ACCELERATION = 80.0; // Motion Magic rotations per second squared
    public static final double JERK = 300.0; // Motion Magic rotations per second cubed

    public static final class Slot0 {
    public static final double kP = 8.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.05;
    }

    // PID and FF Gains
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

        public static final double SAFE = 0.26;
        public static final double LOAD_CORAL = 0.0;
        public static final double GRAB_ALGAE = 0.40;
        public static final double L1 = 0.05;
        public static final double L2 = 0.10; // 0.20
        public static final double L3 = 0.30;
        public static final double L4 = 0.40;

    }

    public static final class EncoderPose {
        public static final double SAFE = 0.26;
        public static final double LOAD_CORAL = 0.0;
        public static final double GRAB_ALGAE = 0.40;
        public static final double L1 = 0.05;
        public static final double L2 = 0.20;
        public static final double L3 = 0.30;
        public static final double L4 = 0.40;
    }
}