package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOLimelight implements VisionIO {
    // NetworkTable entries for Limelight data
    private final NetworkTable limelightTable;
    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry botpose_targetspaceEntry;
    private final NetworkTableEntry targetpose_robotspaceEntry;
    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry ledEntry;
    private final NetworkTableEntry pipelineLatencyEntry;
    private final NetworkTableEntry captureLatencyEntry;
    private final NetworkTableEntry pipelineEntry;

    // Cached values to avoid repeated NT queries
    private double lastTimeStamp = 0.0;
    private boolean hasTargets = false;
    private double yawRadians = 0.0;
    private double pitchRadians = 0.0;
    private double[] botpose = new double[6];
    private double[] botpose_targetspace = new double[6];
    private double[] targetpose_robotspace = new double[6];
    private int tagId = -1;
    private double pipelineLatency = 0.0;
    private double captureLatency = 0.0;

    // AprilTag specific values
    private LimelightHelpers.PoseEstimate lastPoseEstimate = null;

    public VisionIOLimelight(String tableName) {
        // Initialize NetworkTable entries
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        botposeEntry = limelightTable.getEntry("botpose");
        botpose_targetspaceEntry = limelightTable.getEntry("botpose_targetspace");
        targetpose_robotspaceEntry = limelightTable.getEntry("targetpose_robotspace");
        validEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        tagIdEntry = limelightTable.getEntry("tid");
        ledEntry = limelightTable.getEntry("ledMode");
        pipelineLatencyEntry = limelightTable.getEntry("tl");
        captureLatencyEntry = limelightTable.getEntry("cl");
        pipelineEntry = limelightTable.getEntry("pipeline");
        
        // Set to AprilTag pipeline
        setPipeline(0);
    }

    private synchronized void updateValues() {
        // Update latency values first to calculate accurate timestamp
        pipelineLatency = pipelineLatencyEntry.getDouble(0.0);
        captureLatency = captureLatencyEntry.getDouble(0.0);
        
        // Calculate timestamp accounting for both pipeline and capture latency
        lastTimeStamp = Timer.getFPGATimestamp() - (pipelineLatency + captureLatency) / 1000.0;

        // Update target detection values
        hasTargets = validEntry.getDouble(0.0) > 0.5; // Use 0.5 as threshold for robustness
        
        if (hasTargets) {
            yawRadians = Units.degreesToRadians(txEntry.getDouble(0.0));
            pitchRadians = Units.degreesToRadians(tyEntry.getDouble(0.0));
            botpose = botposeEntry.getDoubleArray(new double[6]);
            botpose_targetspace = botpose_targetspaceEntry.getDoubleArray(new double[6]);
            targetpose_robotspace = targetpose_robotspaceEntry.getDoubleArray(new double[6]);
            tagId = (int) tagIdEntry.getDouble(-1.0);
            
            // Get the latest pose estimate from LimelightHelpers
            lastPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        } else {
            // Reset values when no target is detected
            yawRadians = 0.0;
            pitchRadians = 0.0;
            botpose = new double[6];
            botpose_targetspace = new double[6];
            targetpose_robotspace = new double[6];
            tagId = -1;
            lastPoseEstimate = null;
        }
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        updateValues(); // Ensure values are fresh

        // Copy cached values to inputs
        inputs.lastTimeStamp = this.lastTimeStamp;
        inputs.horizontalAngleRadians = this.yawRadians;
        inputs.verticalAngleRadians = this.pitchRadians;
        inputs.hasTargets = this.hasTargets;
        inputs.botpose = this.botpose.clone(); // Create defensive copy
        inputs.tagId = this.tagId;
        inputs.pipelineLatency = this.pipelineLatency;
        inputs.captureLatency = this.captureLatency;
    }
    
    /**
     * Gets the pose estimate from the Limelight
     * @return The latest pose estimate or null if not available
     */
    public synchronized LimelightHelpers.PoseEstimate getPoseEstimate() {
        updateValues(); // Ensure values are fresh
        return lastPoseEstimate;
    }
    
    /**
     * Gets the robot pose relative to the target
     * @return Double array with the pose data or null if not available
     */
    public synchronized double[] getBotPose_TargetSpace() {
        updateValues(); // Ensure values are fresh
        return botpose_targetspace.clone();
    }
    
    /**
     * Gets the target pose relative to the robot
     * @return Double array with the pose data or null if not available
     */
    public synchronized double[] getTargetPose_RobotSpace() {
        updateValues(); // Ensure values are fresh
        return targetpose_robotspace.clone();
    }
    
    /**
     * Sets the pipeline index for the Limelight
     * @param pipeline Index of the pipeline to use (0-9)
     */
    public void setPipeline(int pipeline) {
        pipelineEntry.setNumber(pipeline);
    }

    @Override 
    public void setLeds(boolean on) {
        ledEntry.setNumber(on ? 3 : 1); // 3=force on, 1=force off
    }
    
    /**
     * Converts the 6-element botpose array to a Pose3d
     * @return A Pose3d object representing the robot's position in 3D space
     */
    public Pose3d getBotPose3d() {
        updateValues(); // Ensure values are fresh
        return LimelightHelpers.toPose3D(botpose);
    }
    
    /**
     * Converts the 6-element botpose array to a Pose2d
     * @return A Pose2d object representing the robot's position in 2D space
     */
    public Pose2d getBotPose2d() {
        updateValues(); // Ensure values are fresh
        return LimelightHelpers.toPose2D(botpose);
    }
}