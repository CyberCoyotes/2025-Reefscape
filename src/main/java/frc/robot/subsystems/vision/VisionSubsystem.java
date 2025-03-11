package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private final String tableName;
    private final NetworkTable limelightTable;
    private final CommandSwerveDrivetrain drivetrain;
    
    // Vision processing constants
    private static final double TARGET_LOCK_THRESHOLD = 2.0; // Degrees
    private static final double VALID_TARGET_AREA = 0.1; // % of image
    
    private VisionState currentState = VisionState.NO_TARGET;
    private boolean ledsEnabled = false;
    
    // Constants for tag alignment
    private static final double TAG_LATERAL_OFFSET = 0.20; // 20 cm in meters
    
    // New members to track target information
    private double[] botpose_targetspace = new double[6];
    private double[] targetpose_robotspace = new double[6];
    private double distanceToTarget = 0.0;
    private int lastTagId = -1;
    
    // LimelightHelpers instance
    private LimelightHelpers.PoseEstimate lastPoseEstimate;

    public VisionSubsystem(String tableName, CommandSwerveDrivetrain drivetrain) {
        this.tableName = tableName;
        this.drivetrain = drivetrain;
        
        // Initialize NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        
        // Configure Limelight
        configureLimelight();
        setLeds(true);
        ledsEnabled = true;
    }

    private void configureLimelight() {
        // Set to AprilTag pipeline
        LimelightHelpers.setPipelineIndex(tableName, 0);
        // Only use AprilTags for localization
        //LimelightHelpers.setLeds(tableName, true);
    }

    @Override
    public void periodic() {
        updateVisionState();
        updateTargetInfo();
        logData();
    }

    private void updateVisionState() {
        boolean hasTarget = LimelightHelpers.getTV(tableName);
        double horizontalOffset = LimelightHelpers.getTX(tableName);
        double area = LimelightHelpers.getTA(tableName);

        if (!hasTarget || area < VALID_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(horizontalOffset) <= TARGET_LOCK_THRESHOLD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
    }
    
    private void updateTargetInfo() {
        if (hasTarget()) {
            // Get the current tag ID
            lastTagId = (int) LimelightHelpers.getFiducialID(tableName);
            
            // Get the pose of the robot relative to the target
            botpose_targetspace = LimelightHelpers.getBotPose_TargetSpace(tableName);
            
            // Get the pose of the target relative to the robot
            targetpose_robotspace = LimelightHelpers.getTargetPose_RobotSpace(tableName);
            
            // Calculate distance to target (using Pythagorean theorem on x and z values)
            if (targetpose_robotspace.length >= 3) {
                distanceToTarget = Math.sqrt(
                    Math.pow(targetpose_robotspace[0], 2) + 
                    Math.pow(targetpose_robotspace[2], 2)
                );
            }
            
            // Get the latest pose estimate
            lastPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(tableName);
        }
    }

    public void setLeds(boolean enabled) {
        ledsEnabled = enabled;
        LimelightHelpers.setLEDMode_ForceOn(tableName);
    }

    private void logData() {
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", getTagId());
        SmartDashboard.putNumber("Vision/TX", LimelightHelpers.getTX(tableName));
        SmartDashboard.putNumber("Vision/TY", LimelightHelpers.getTY(tableName));
        SmartDashboard.putNumber("Vision/TA", LimelightHelpers.getTA(tableName));
        SmartDashboard.putNumber("Vision/DistanceToTarget", distanceToTarget);
        
        // Log more detailed pose information
        if (hasTarget()) {
            SmartDashboard.putNumberArray("Vision/BotPose_TargetSpace", botpose_targetspace);
            SmartDashboard.putNumberArray("Vision/TargetPose_RobotSpace", targetpose_robotspace);
        }
    }

    // Getter methods for use in commands
    public VisionState getState() {
        return currentState;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(tableName);
    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(tableName);
    }

    public double getVerticalOffset() {
        return LimelightHelpers.getTY(tableName);
    }

    public int getTagId() {
        return (int) LimelightHelpers.getFiducialID(tableName);
    }
    
    public double getDistanceToTarget() {
        return distanceToTarget;
    }
    
    /**
     * Calculates a target pose offset for alignment with the current AprilTag
     * @param isRightSide If true, offsets to the right of the tag. If false, offsets to the left.
     * @return Transform2d representing the offset to apply to the target pose
     */
    public Transform2d calculateTagAlignment(boolean isRightSide) {
        double offsetX = 0;
        double offsetY = isRightSide ? TAG_LATERAL_OFFSET : -TAG_LATERAL_OFFSET;
        
        return new Transform2d(new Translation2d(offsetX, offsetY), drivetrain.getState().Pose.getRotation());
    }
    
    /**
     * Returns the current pose of the robot in field coordinates
     * @return The robot's current pose
     */
    public Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }
    
    /**
     * Gets the pose of the most recently detected AprilTag in field coordinates
     * @return Pose2d of the tag, or null if no tag is detected
     */
    public Pose2d getTagPose() {
        if (!hasTarget() || lastPoseEstimate == null) {
            return null;
        }
        
        // Use the pose from the Limelight's PoseEstimate
        Pose2d robotPose = lastPoseEstimate.pose;
        
        // Construct the tag pose by adding the target position relative to the robot
        // to the robot's position in field space
        Translation2d tagTranslation = robotPose.getTranslation().plus(
            new Translation2d(targetpose_robotspace[0], targetpose_robotspace[1])
                .rotateBy(robotPose.getRotation())
        );
        
        return new Pose2d(tagTranslation, robotPose.getRotation());
    }
    
    /**
     * Updates the robot's odometry using the AprilTag vision data
     */
    public void updateOdometryWithVision() {
        if (hasTarget() && lastPoseEstimate != null && lastPoseEstimate.tagCount > 0) {
            // Only update if we have a reliable pose estimate (more than one tag is better)
            if (lastPoseEstimate.pose != null) {
                drivetrain.addVisionMeasurement(
                    lastPoseEstimate.pose,
                    lastPoseEstimate.timestampSeconds
                );
            }
        }
    }
}