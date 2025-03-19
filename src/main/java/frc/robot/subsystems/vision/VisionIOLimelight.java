package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.Units;

/**
 * Implementation of VisionIO for Limelight hardware
 */
public class VisionIOLimelight implements VisionIO {
    // NetworkTable entries for Limelight data
    private final String tableName;
    private final NetworkTable limelightTable;
    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry botposeBlueEntry;
    private final NetworkTableEntry botposeRedEntry;
    private final NetworkTableEntry botposeTargetSpaceEntry;
    private final NetworkTableEntry cameraPoseTargetSpaceEntry;
    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry ledEntry;
    private final NetworkTableEntry pipelineEntry;
    private final NetworkTableEntry camModeEntry;
    private final NetworkTableEntry currentPipelineEntry;
    private final NetworkTableEntry pipelineLatencyEntry;
    private final NetworkTableEntry captureLatencyEntry;

    // Cached values to avoid repeated NT queries
    private double lastTimeStamp = 0.0;
    private boolean hasTargets = false;
    private double yawRadians = 0.0;
    private double pitchRadians = 0.0;
    private double targetArea = 0.0;
    private double[] botpose = new double[6];
    private double[] botposeBlue = new double[6];
    private double[] botposeRed = new double[6];
    private double[] botposeTargetSpace = new double[6];
    private double[] cameraPoseTargetSpace = new double[6];
    private int tagId = -1;
    private int currentPipeline = 0;
    private double pipelineLatency = 0.0;
    private double captureLatency = 0.0;

    /**
     * Creates a new VisionIOLimelight with the specified table name
     * 
     * @param tableName The NetworkTables name of the Limelight
     */
    public VisionIOLimelight(String tableName) {
        this.tableName = tableName;
        
        // Initialize NetworkTable entries
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        botposeEntry = limelightTable.getEntry("botpose");
        botposeBlueEntry = limelightTable.getEntry("botpose_wpiblue");
        botposeRedEntry = limelightTable.getEntry("botpose_wpired");
        botposeTargetSpaceEntry = limelightTable.getEntry("botpose_targetspace");
        cameraPoseTargetSpaceEntry = limelightTable.getEntry("camerapose_targetspace");
        validEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tagIdEntry = limelightTable.getEntry("tid");
        ledEntry = limelightTable.getEntry("ledMode");
        pipelineEntry = limelightTable.getEntry("pipeline");
        camModeEntry = limelightTable.getEntry("camMode");
        currentPipelineEntry = limelightTable.getEntry("getpipe");
        pipelineLatencyEntry = limelightTable.getEntry("tl");
        captureLatencyEntry = limelightTable.getEntry("cl");
    }

    /**
     * Updates the cached values from NetworkTables
     */
    private synchronized void updateValues() {
        // Update latency values first to calculate accurate timestamp
        pipelineLatency = pipelineLatencyEntry.getDouble(0.0);
        captureLatency = captureLatencyEntry.getDouble(0.0);
        
        // Calculate timestamp accounting for both pipeline and capture latency
        lastTimeStamp = Timer.getFPGATimestamp() - (pipelineLatency + captureLatency) / 1000.0;

        // Update target detection values
        hasTargets = validEntry.getDouble(0.0) > 0.5; // Use 0.5 as threshold for robustness
        
        // Get current pipeline
        currentPipeline = (int) currentPipelineEntry.getDouble(0.0);
        
        if (hasTargets) {
            yawRadians = Units.degreesToRadians(txEntry.getDouble(0.0));
            pitchRadians = Units.degreesToRadians(tyEntry.getDouble(0.0));
            targetArea = taEntry.getDouble(0.0);
            botpose = botposeEntry.getDoubleArray(new double[6]);
            botposeBlue = botposeBlueEntry.getDoubleArray(new double[6]);
            botposeRed = botposeRedEntry.getDoubleArray(new double[6]);
            botposeTargetSpace = botposeTargetSpaceEntry.getDoubleArray(new double[6]);
            cameraPoseTargetSpace = cameraPoseTargetSpaceEntry.getDoubleArray(new double[6]);
            tagId = (int) tagIdEntry.getDouble(-1.0);
        } else {
            // Reset values when no target is detected
            yawRadians = 0.0;
            pitchRadians = 0.0;
            targetArea = 0.0;
            botpose = new double[6];
            botposeBlue = new double[6];
            botposeRed = new double[6];
            botposeTargetSpace = new double[6];
            cameraPoseTargetSpace = new double[6];
            tagId = -1;
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
        inputs.targetArea = this.targetArea;
        inputs.botPose = this.botpose.clone(); // Create defensive copy
        inputs.botPoseBlue = this.botposeBlue.clone();
        inputs.botPoseRed = this.botposeRed.clone();
        inputs.botPoseTargetSpace = this.botposeTargetSpace.clone();
        inputs.cameraPoseTargetSpace = this.cameraPoseTargetSpace.clone();
        inputs.tagId = this.tagId;
        inputs.currentPipeline = this.currentPipeline;
        inputs.pipelineLatency = this.pipelineLatency;
        inputs.captureLatency = this.captureLatency;
    }

    @Override 
    public void setLeds(boolean on) {
        ledEntry.setNumber(on ? VisionConstants.LED_MODE_ON : VisionConstants.LED_MODE_OFF);
    }
    
    @Override
    public void setPipeline(int index) {
        pipelineEntry.setNumber(index);
    }
    
    @Override
    public void setDriverMode(boolean driverMode) {
        camModeEntry.setNumber(driverMode ? 1 : 0);
    }
}