package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.Units;

public class VisionIOLimelight implements VisionIO {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry botposeTargetSpaceEntry;
    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry ledEntry;
    private final NetworkTableEntry pipelineLatencyEntry;
    private final NetworkTableEntry captureLatencyEntry;
    
    // Cached values
    private double lastTimeStamp = 0.0;
    private boolean hasTargets = false;
    private double yawRadians = 0.0;
    private double pitchRadians = 0.0;
    private double[] botpose = new double[6];
    private double[] botposeTargetSpace = new double[6];
    private int tagId = -1;
    private double pipelineLatency = 0.0;
    private double captureLatency = 0.0;
    
    public VisionIOLimelight(String tableName) {
        // Initialize NetworkTable entries
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        botposeEntry = limelightTable.getEntry("botpose");
        botposeTargetSpaceEntry = limelightTable.getEntry("botpose_targetspace");
        validEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        tagIdEntry = limelightTable.getEntry("tid");
        ledEntry = limelightTable.getEntry("ledMode");
        pipelineLatencyEntry = limelightTable.getEntry("tl");
        captureLatencyEntry = limelightTable.getEntry("cl");
    }
    
    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        // Update latency values first to calculate accurate timestamp
        pipelineLatency = pipelineLatencyEntry.getDouble(0.0);
        captureLatency = captureLatencyEntry.getDouble(0.0);
        
        // Calculate timestamp accounting for both pipeline and capture latency
        lastTimeStamp = Timer.getFPGATimestamp() - (pipelineLatency + captureLatency) / 1000.0;
        
        // Update target detection values
        hasTargets = validEntry.getDouble(0.0) > 0.5;
        
        if (hasTargets) {
            yawRadians = Units.degreesToRadians(txEntry.getDouble(0.0));
            pitchRadians = Units.degreesToRadians(tyEntry.getDouble(0.0));
            botpose = botposeEntry.getDoubleArray(new double[6]);
            botposeTargetSpace = botposeTargetSpaceEntry.getDoubleArray(new double[6]);
            tagId = (int) tagIdEntry.getDouble(-1.0);
        } else {
            // Reset values when no target is detected
            yawRadians = 0.0;
            pitchRadians = 0.0;
            botpose = new double[6];
            botposeTargetSpace = new double[6];
            tagId = -1;
        }
        
        // Copy cached values to inputs
        inputs.lastTimeStamp = this.lastTimeStamp;
        inputs.horizontalAngleRadians = this.yawRadians;
        inputs.verticalAngleRadians = this.pitchRadians;
        inputs.hasTargets = this.hasTargets;
        inputs.botpose = this.botpose.clone();
        inputs.botposeTargetSpace = this.botposeTargetSpace.clone();
        inputs.tagId = this.tagId;
        inputs.pipelineLatency = this.pipelineLatency;
        inputs.captureLatency = this.captureLatency;
    }
    
    @Override 
    public void setLeds(boolean on) {
        ledEntry.setNumber(on ? 3 : 1); // 3=force on, 1=force off
    }
    
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
}