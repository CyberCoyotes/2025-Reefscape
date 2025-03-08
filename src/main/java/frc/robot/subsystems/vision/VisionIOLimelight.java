package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * Implementation of VisionIO for Limelight cameras.
 */
public class VisionIOLimelight implements VisionIO {
    private final String limelightName;
    
    /**
     * Creates a new VisionIOLimelight.
     * 
     * @param limelightName The name of the Limelight camera
     */
    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Get basic data from Limelight
        inputs.hasTargets = LimelightHelpers.getTV(limelightName);
        
        if (inputs.hasTargets) {
            // Get angles in radians
            inputs.horizontalAngleRadians = Units.degreesToRadians(LimelightHelpers.getTX(limelightName));
            inputs.verticalAngleRadians = Units.degreesToRadians(LimelightHelpers.getTY(limelightName));
            
            // Get AprilTag ID
            inputs.tagId = (int) LimelightHelpers.getFiducialID(limelightName);
            
            // Get bot pose data (using default alliance perspective)
            inputs.botpose = LimelightHelpers.getBotPose(limelightName);
            
            // Calculate timestamp with latency adjustment
            double pipelineLatency = LimelightHelpers.getLatency_Pipeline(limelightName);
            double captureLatency = LimelightHelpers.getLatency_Capture(limelightName);
            inputs.pipelineLatency = pipelineLatency;
            inputs.captureLatency = captureLatency;
            
            // Adjust timestamp for latency
            inputs.lastTimeStamp = Timer.getFPGATimestamp() - (pipelineLatency + captureLatency) / 1000.0;
        } else {
            // Reset values when no target is detected
            inputs.horizontalAngleRadians = 0.0;
            inputs.verticalAngleRadians = 0.0;
            inputs.tagId = -1;
            inputs.botpose = new double[6];
            inputs.lastTimeStamp = Timer.getFPGATimestamp();
            inputs.pipelineLatency = 0.0;
            inputs.captureLatency = 0.0;
        }
    }
    
    @Override
    public void setLeds(boolean on) {
        if (on) {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        }
    }
}