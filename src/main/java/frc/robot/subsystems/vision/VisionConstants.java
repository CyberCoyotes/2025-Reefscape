package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

/**
 * Constants for the vision subsystem
 */
public final class VisionConstants {
    // Prevent instantiation
    private VisionConstants() {}
    
    // Default Limelight table name
    public static final String DEFAULT_TABLE_NAME = "limelight";
    
    // Vision subsystem constants
    public static final double TARGET_LOCK_THRESHOLD_DEG = 2.0; // Degrees
    public static final double TARGET_LOCK_THRESHOLD_RAD = Units.degreesToRadians(TARGET_LOCK_THRESHOLD_DEG);
    public static final double VALID_TARGET_AREA = 0.1; // % of image
    
    // AprilTag alignment PID constants
    public static final double X_REEF_ALIGNMENT_P = 0.1;
    public static final double Y_REEF_ALIGNMENT_P = 0.1;
    public static final double ROT_REEF_ALIGNMENT_P = 0.1;
    
    // Alignment setpoints
    public static final double X_SETPOINT_REEF_ALIGNMENT = 0.0; // meters
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.0; // meters
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.0; // radians
    
    // Alignment tolerances
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.1; // meters
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.1; // meters
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.1; // radians
    
    // Alignment timing constants
    public static final double DONT_SEE_TAG_WAIT_TIME = 0.3; // seconds
    public static final double POSE_VALIDATION_TIME = 0.3; // seconds
    
    // Limelight LED modes
    public static final int LED_MODE_PIPELINE = 0; // Use pipeline default
    public static final int LED_MODE_OFF = 1; // Force off
    public static final int LED_MODE_BLINK = 2; // Force blink
    public static final int LED_MODE_ON = 3; // Force on
    
    // Limelight pipelines
    public static final int APRILTAG_PIPELINE = 0; // Default AprilTag pipeline
    public static final int RETROREFLECTIVE_PIPELINE = 1; // Default retroreflective pipeline
}