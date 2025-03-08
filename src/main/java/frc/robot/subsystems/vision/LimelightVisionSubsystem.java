package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Vision subsystem using Limelight for AprilTag detection and pose estimation.
 */
public class LimelightVisionSubsystem extends SubsystemBase {
    private final String limelightName;
    private final CommandSwerveDrivetrain drivetrain;
    
    // Cache for vision data
    private boolean hasValidTarget = false;
    private double lastValidTimestamp = 0;
    private int currentTagId = -1;
    private Pose2d robotPose = new Pose2d();
    
    // Constants for alignment
    private static final double OFFSET_DISTANCE_METERS = 0.35; // Distance to offset from AprilTag center
    private static final double MAX_TARGET_AGE_SECONDS = 0.5; // How long targets remain valid
    private static final double TARGET_ACQUIRED_THRESHOLD = 2.0; // Degrees
    
    // Current vision state
    private VisionState currentState = VisionState.NO_TARGET;
    
    /**
     * Creates a new LimelightVisionSubsystem.
     * 
     * @param limelightName The NetworkTable name of the Limelight
     * @param drivetrain The swerve drivetrain for pose updates
     */
    public LimelightVisionSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
        
        // Configure Limelight for AprilTag detection 
        configureLimelight();
    }
    
    /**
     * Configure the Limelight for AprilTag detection.
     */
    private void configureLimelight() {
        // Set to AprilTag pipeline (assuming pipeline 0 is configured for AprilTags)
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        
        // Turn on LEDs
        setLeds(true);
    }
    
    @Override
    public void periodic() {
        updateVisionData();
        updateVisionState();
        logData();
        
        // Optional: Add vision measurements to drivetrain odometry
        if (hasValidTarget && Timer.getFPGATimestamp() - lastValidTimestamp < MAX_TARGET_AGE_SECONDS) {
            // Use botpose from the appropriate alliance perspective
            Pose2d visionPose;
            if (DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                visionPose = LimelightHelpers.getBotPose2d_wpiRed(limelightName);
            } else {
                visionPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
            }
            
            // Calculate latency (pipeline + capture latency in seconds)
            double latencySeconds = (LimelightHelpers.getLatency_Pipeline(limelightName) + 
                                     LimelightHelpers.getLatency_Capture(limelightName)) / 1000.0;
            
            // Add vision measurement to drivetrain
            double timestamp = Timer.getFPGATimestamp() - latencySeconds;
            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }
    }
    
    /**
     * Update cached vision data from Limelight.
     */
    private void updateVisionData() {
        // Check if Limelight has a valid target
        hasValidTarget = LimelightHelpers.getTV(limelightName);
        
        if (hasValidTarget) {
            // Update last valid timestamp and tag ID
            lastValidTimestamp = Timer.getFPGATimestamp();
            currentTagId = (int) LimelightHelpers.getFiducialID(limelightName);
            
            // Get robot pose from Limelight (uses the appropriate alliance-relative pose)
            if (DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                robotPose = LimelightHelpers.getBotPose2d_wpiRed(limelightName);
            } else {
                robotPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
            }
        }
    }
    
    /**
     * Update the current vision state based on target data.
     */
    private void updateVisionState() {
        if (!hasValidTarget || Timer.getFPGATimestamp() - lastValidTimestamp > MAX_TARGET_AGE_SECONDS) {
            currentState = VisionState.NO_TARGET;
            return;
        }
        
        // Check horizontal offset to target for alignment state
        double horizontalOffset = LimelightHelpers.getTX(limelightName);
        if (Math.abs(horizontalOffset) <= TARGET_ACQUIRED_THRESHOLD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
    }
    
    /**
     * Log vision data to SmartDashboard.
     */
    private void logData() {
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", currentTagId);
        SmartDashboard.putNumber("Vision/TX", LimelightHelpers.getTX(limelightName));
        SmartDashboard.putNumber("Vision/TY", LimelightHelpers.getTY(limelightName));
        SmartDashboard.putNumber("Vision/TA", LimelightHelpers.getTA(limelightName));
        SmartDashboard.putBoolean("Vision/HasTarget", hasValidTarget);
        
        // Log vision pose components
        if (hasValidTarget) {
            SmartDashboard.putNumber("Vision/PoseX", robotPose.getX());
            SmartDashboard.putNumber("Vision/PoseY", robotPose.getY());
            SmartDashboard.putNumber("Vision/PoseRot", robotPose.getRotation().getDegrees());
        }
    }
    
    /**
     * Turn the Limelight LEDs on or off.
     * 
     * @param enabled True to turn LEDs on, false to turn them off
     */
    public void setLeds(boolean enabled) {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        }
    }
    
    /**
     * Get the current vision state.
     * 
     * @return The current VisionState
     */
    public VisionState getState() {
        return currentState;
    }
    
    /**
     * Check if the vision system currently has a target.
     * 
     * @return True if a valid target exists, false otherwise
     */
    public boolean hasTarget() {
        return hasValidTarget && (Timer.getFPGATimestamp() - lastValidTimestamp < MAX_TARGET_AGE_SECONDS);
    }
    
    /**
     * Get the ID of the currently detected AprilTag.
     * 
     * @return The AprilTag ID, or -1 if no tag is detected
     */
    public int getTagId() {
        if (hasTarget()) {
            return currentTagId;
        }
        return -1;
    }
    
    /**
     * Get the horizontal offset to the target in degrees.
     * 
     * @return The horizontal offset in degrees
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName);
    }
    
    /**
     * Get the vertical offset to the target in degrees.
     * 
     * @return The vertical offset in degrees
     */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(limelightName);
    }
    
    /**
     * Get the current robot pose as estimated by the Limelight.
     * 
     * @return The estimated robot pose
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }
    
    /**
     * Get the pose of the currently detected AprilTag.
     * 
     * @return The AprilTag pose, or null if no tag is detected
     */
    public Pose2d getTagPose() {
        if (!hasTarget()) {
            return null;
        }
        
        // Calculate tag pose based on robot pose and camera-to-target transform
        double[] targetPoseArray = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (targetPoseArray.length < 6) {
            return null;
        }
        
        // Convert camera-relative coordinates to field coordinates
        Translation2d tagTranslation = new Translation2d(targetPoseArray[0], targetPoseArray[1]);
        Rotation2d tagRotation = Rotation2d.fromDegrees(targetPoseArray[5]);
        
        return new Pose2d(tagTranslation, tagRotation);
    }
    
    /**
     * Calculate an offset target pose left of the AprilTag.
     * 
     * @return The offset pose to the left of the tag, or null if no tag is detected
     */
    public Pose2d getLeftOffsetFromTag() {
        return getOffsetFromTag(true);
    }
    
    /**
     * Calculate an offset target pose right of the AprilTag.
     * 
     * @return The offset pose to the right of the tag, or null if no tag is detected
     */
    public Pose2d getRightOffsetFromTag() {
        return getOffsetFromTag(false);
    }
    
    /**
     * Calculate an offset target pose from the AprilTag.
     * 
     * @param isLeft True for left offset, false for right offset
     * @return The offset pose, or null if no tag is detected
     */
    private Pose2d getOffsetFromTag(boolean isLeft) {
        if (!hasTarget()) {
            return null;
        }
        
        // Get tag pose in field coordinates
        Pose2d tagPose = getTagPose();
        if (tagPose == null) {
            return null;
        }
        
        // Calculate perpendicular direction for offset
        Rotation2d tagDirection = tagPose.getRotation();
        Rotation2d perpendicular = tagDirection.plus(Rotation2d.fromDegrees(isLeft ? 90 : -90));
        
        // Apply offset in the perpendicular direction
        Translation2d offsetTranslation = new Translation2d(OFFSET_DISTANCE_METERS, 0).rotateBy(perpendicular);
        Translation2d targetTranslation = tagPose.getTranslation().plus(offsetTranslation);
        
        // Return the offset pose with the same rotation as the tag
        return new Pose2d(targetTranslation, tagDirection);
    }
}