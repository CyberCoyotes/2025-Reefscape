/* 
 * Structure inspired by FRC 2910 
 * https://github.com/FRCTeam2910/2024CompetitionRobot-Public/tree/main/src/main/java/frc/robot/subsystems
*/
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Hardware abstraction interface for vision sensors.
 * Implemented by hardware-specific classes like VisionIOLimelight.
 */
public interface VisionIO {
    /**
     * Updates the set of loggable inputs.
     * 
     * @param inputs The input values to update
     */
    public default void updateInputs(VisionIOInputs inputs) {}
    
    /**
     * Sets the LED mode for the vision device.
     * 
     * @param on True to turn LEDs on, false to turn them off
     */
    public default void setLeds(boolean on) {}
    
    /**
     * Sets the pipeline index for the vision device.
     * 
     * @param index Pipeline index to set
     */
    public default void setPipeline(int index) {}
    
    /**
     * Sets the camera mode for the vision device.
     * 
     * @param driverMode True for driver camera mode, false for vision processing mode
     */
    public default void setDriverMode(boolean driverMode) {}

    /**
     * Container for vision sensor inputs
     */
    public static class VisionIOInputs {
        // Timestamp data
        public double lastTimeStamp = 0.0;
        
        // Target data
        public boolean hasTargets = false;
        public double verticalAngleRadians = 0.0;
        public double horizontalAngleRadians = 0.0;
        public double targetArea = 0.0;
        
        // AprilTag specific data  
        public double[] botPose = new double[6];
        public double[] botPoseBlue = new double[6];
        public double[] botPoseRed = new double[6];
        public double[] botPoseTargetSpace = new double[6];
        public double[] cameraPoseTargetSpace = new double[6];
        public int tagId = -1;
        
        // Latency data
        public double pipelineLatency = 0.0;
        public double captureLatency = 0.0;
        
        // Pipeline data
        public int currentPipeline = 0;
    }
}