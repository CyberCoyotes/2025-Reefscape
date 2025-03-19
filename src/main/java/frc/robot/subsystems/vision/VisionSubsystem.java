package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

/**
 * Subsystem for vision processing using Limelight
 */
public class VisionSubsystem extends SubsystemBase {
    private final VisionIO visionIO;
    private final VisionIOInputs inputs = new VisionIOInputs();
    private final CommandSwerveDrivetrain drivetrain;
    
    private VisionState currentState = VisionState.NO_TARGET;
    private boolean ledsEnabled = false;

    /**
     * Creates a new VisionSubsystem with the specified VisionIO implementation
     * 
     * @param visionIO The VisionIO implementation to use
     * @param drivetrain The drive subsystem for pose estimation updates
     */
    public VisionSubsystem(VisionIO visionIO, CommandSwerveDrivetrain drivetrain) {
        this.visionIO = visionIO;
        this.drivetrain = drivetrain;
        
        // Configure Limelight
        configureLimelight();
    }

    /**
     * Configures the Limelight with initial settings
     */
    private void configureLimelight() {
        // Set to AprilTag pipeline
        visionIO.setPipeline(VisionConstants.APRILTAG_PIPELINE);
        setLeds(true);
        ledsEnabled = true;
    }

    @Override
    public void periodic() {
        // Update vision inputs
        visionIO.updateInputs(inputs);
        
        // Process vision data
        updateVisionState();
        
        // Log data to SmartDashboard
        logData();
        
        // Update pose estimator with vision data if enabled
        if (drivetrain != null && inputs.hasTargets) {
            updateOdometryWithVision();
        }
    }

    /**
     * Updates the vision state based on current inputs
     */
    private void updateVisionState() {
        if (!inputs.hasTargets || inputs.targetArea < VisionConstants.VALID_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(inputs.horizontalAngleRadians) <= VisionConstants.TARGET_LOCK_THRESHOLD_RAD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
    }

    /**
     * Updates the drivetrain odometry with vision data
     */
    private void updateOdometryWithVision() {
        // Only update if we have valid pose data
        if (inputs.botPoseBlue.length >= 6) {
            double latency = 0.0;
            if (inputs.botPoseBlue.length >= 7) {
                // If botpose includes latency as 7th element
                latency = inputs.botPoseBlue[6] / 1000.0; // Convert from ms to seconds
            } else {
                // Otherwise use the pipeline and capture latency
                latency = (inputs.pipelineLatency + inputs.captureLatency) / 1000.0;
            }
            
            // Convert botpose to Pose2d
            Pose2d robotPose = LimelightHelpers.toPose2D(inputs.botPoseBlue);
            
            // Calculate the timestamp when the image was captured
            double timestamp = Timer.getFPGATimestamp() - latency;
            
            // Add vision measurement to drivetrain pose estimator
            drivetrain.addVisionMeasurement(robotPose, timestamp);
        }
    }

    /**
     * Sets the LED state of the Limelight
     * 
     * @param enabled True to turn LEDs on, false to turn them off
     */
    public void setLeds(boolean enabled) {
        ledsEnabled = enabled;
        visionIO.setLeds(enabled);
    }

    /**
     * Sets the pipeline of the Limelight
     * 
     * @param pipelineIndex Pipeline index to use
     */
    public void setPipeline(int pipelineIndex) {
        visionIO.setPipeline(pipelineIndex);
    }

    /**
     * Sets the camera mode for driver view or vision processing
     * 
     * @param driverMode True for driver mode, false for vision processing
     */
    public void setDriverMode(boolean driverMode) {
        visionIO.setDriverMode(driverMode);
    }

    /**
     * Logs vision data to SmartDashboard
     */
    private void logData() {
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", inputs.tagId);
        SmartDashboard.putNumber("Vision/TX", Units.radiansToDegrees(inputs.horizontalAngleRadians));
        SmartDashboard.putNumber("Vision/TY", Units.radiansToDegrees(inputs.verticalAngleRadians));
        SmartDashboard.putNumber("Vision/TA", inputs.targetArea);
        SmartDashboard.putNumber("Vision/Pipeline", inputs.currentPipeline);
        SmartDashboard.putNumber("Vision/Latency", inputs.pipelineLatency + inputs.captureLatency);
        SmartDashboard.putBoolean("Vision/HasTarget", inputs.hasTargets);
    }

    /**
     * Gets the current vision state
     * 
     * @return The current vision state
     */
    public VisionState getState() {
        return currentState;
    }

    /**
     * Checks if a target is currently visible
     * 
     * @return True if a target is visible, false otherwise
     */
    public boolean hasTarget() {
        return inputs.hasTargets;
    }

    /**
     * Gets the horizontal angle to the target in radians
     * 
     * @return The horizontal angle in radians
     */
    public double getHorizontalAngleRadians() {
        return inputs.horizontalAngleRadians;
    }

    /**
     * Gets the horizontal angle to the target in degrees
     * 
     * @return The horizontal angle in degrees
     */
    public double getHorizontalAngleDegrees() {
        return Units.radiansToDegrees(inputs.horizontalAngleRadians);
    }

    /**
     * Gets the vertical angle to the target in radians
     * 
     * @return The vertical angle in radians
     */
    public double getVerticalAngleRadians() {
        return inputs.verticalAngleRadians;
    }

    /**
     * Gets the vertical angle to the target in degrees
     * 
     * @return The vertical angle in degrees
     */
    public double getVerticalAngleDegrees() {
        return Units.radiansToDegrees(inputs.verticalAngleRadians);
    }

    /**
     * Gets the target area as a percentage of the image
     * 
     * @return The target area percentage (0-100)
     */
    public double getTargetArea() {
        return inputs.targetArea;
    }

    /**
     * Gets the ID of the detected AprilTag
     * 
     * @return The AprilTag ID, or -1 if no tag is detected
     */
    public int getTagId() {
        return inputs.tagId;
    }

    /**
     * Gets the robot pose in field space (origin at field center)
     * 
     * @return The robot's pose as a double array [x, y, z, roll, pitch, yaw]
     */
    public double[] getBotPose() {
        return inputs.botPose.clone();
    }

    /**
     * Gets the robot pose in blue alliance field space
     * 
     * @return The robot's pose as a double array [x, y, z, roll, pitch, yaw]
     */
    public double[] getBotPose_wpiBlue() {
        return inputs.botPoseBlue.clone();
    }

    /**
     * Gets the robot pose in red alliance field space
     * 
     * @return The robot's pose as a double array [x, y, z, roll, pitch, yaw]
     */
    public double[] getBotPose_wpiRed() {
        return inputs.botPoseRed.clone();
    }

    /**
     * Gets the robot pose in target space
     * 
     * @return The robot's pose relative to the target as a double array [x, y, z, roll, pitch, yaw]
     */
    public double[] getBotPose_TargetSpace() {
        return inputs.botPoseTargetSpace.clone();
    }

    /**
     * Gets the camera pose in target space
     * 
     * @return The camera's pose relative to the target as a double array [x, y, z, roll, pitch, yaw]
     */
    public double[] getCameraPose_TargetSpace() {
        return inputs.cameraPoseTargetSpace.clone();
    }

    /**
     * Gets the robot pose in field space as a Pose2d
     * 
     * @return The robot's pose as a Pose2d
     */
    public Pose2d getBotPose2d() {
        return LimelightHelpers.toPose2D(inputs.botPose);
    }

    /**
     * Gets the robot pose in blue alliance field space as a Pose2d
     * 
     * @return The robot's pose as a Pose2d
     */
    public Pose2d getBotPose2d_wpiBlue() {
        return LimelightHelpers.toPose2D(inputs.botPoseBlue);
    }

    /**
     * Gets the robot pose in red alliance field space as a Pose2d
     * 
     * @return The robot's pose as a Pose2d
     */
    public Pose2d getBotPose2d_wpiRed() {
        return LimelightHelpers.toPose2D(inputs.botPoseRed);
    }

    /**
     * Gets the robot pose in target space as a Pose2d
     * 
     * @return The robot's pose relative to the target as a Pose2d
     */
    public Pose2d getBotPose2d_TargetSpace() {
        return LimelightHelpers.toPose2D(inputs.botPoseTargetSpace);
    }

    /**
     * Gets the camera pose in target space as a Pose2d
     * 
     * @return The camera's pose relative to the target as a Pose2d
     */
    public Pose2d getCameraPose2d_TargetSpace() {
        return LimelightHelpers.toPose2D(inputs.cameraPoseTargetSpace);
    }

    /**
     * Gets the robot pose in field space as a Pose3d
     * 
     * @return The robot's pose as a Pose3d
     */
    public Pose3d getBotPose3d() {
        return LimelightHelpers.toPose3D(inputs.botPose);
    }

    /**
     * Gets the robot pose in blue alliance field space as a Pose3d
     * 
     * @return The robot's pose as a Pose3d
     */
    public Pose3d getBotPose3d_wpiBlue() {
        return LimelightHelpers.toPose3D(inputs.botPoseBlue);
    }

    /**
     * Gets the robot pose in red alliance field space as a Pose3d
     * 
     * @return The robot's pose as a Pose3d
     */
    public Pose3d getBotPose3d_wpiRed() {
        return LimelightHelpers.toPose3D(inputs.botPoseRed);
    }

    /**
     * Gets the robot pose in target space as a Pose3d
     * 
     * @return The robot's pose relative to the target as a Pose3d
     */
    public Pose3d getBotPose3d_TargetSpace() {
        return LimelightHelpers.toPose3D(inputs.botPoseTargetSpace);
    }

    /**
     * Gets the camera pose in target space as a Pose3d
     * 
     * @return The camera's pose relative to the target as a Pose3d
     */
    public Pose3d getCameraPose3d_TargetSpace() {
        return LimelightHelpers.toPose3D(inputs.cameraPoseTargetSpace);
    }
}