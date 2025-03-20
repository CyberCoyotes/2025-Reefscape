package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.led.LEDState;
// import frc.robot.subsystems.led.LEDSubsystem;

@SuppressWarnings("unused")

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private final CommandSwerveDrivetrain drivetrain;
    
    // Vision processing constants
    private static final double TARGET_LOCK_THRESHOLD = 2.0; // Degrees
    private static final double VALID_TARGET_AREA = 0.1; // % of image
    
    private VisionState currentState = VisionState.NO_TARGET;
    
    /**
     * Constants for vision alignment
     */
    public static class AlignmentConstants {
        // PID values
        public static final class PID {
            // Forward (X) control
            public static final double FORWARD_P = 0.5;
            public static final double FORWARD_I = 0.0;
            public static final double FORWARD_D = 0.0;
            public static final double FORWARD_TOLERANCE = 0.05; // meters
            
            // Lateral (Y) control
            public static final double LATERAL_P = 0.2;
            public static final double LATERAL_I = 0.0;
            public static final double LATERAL_D = 0.0;
            public static final double LATERAL_TOLERANCE = 1.0; // degrees
            
            // Rotation control
            public static final double ROTATION_P = 0.1;
            public static final double ROTATION_I = 0.0;
            public static final double ROTATION_D = 0.0;
            public static final double ROTATION_TOLERANCE = 2.0; // degrees
        }
        
        // Target values
        public static final double TARGET_DISTANCE = -0.5; // meters
        public static final double MAX_SPEED = 1.0; // maximum speed for alignment
        public static final double MAX_ANGULAR_SPEED = 1.0; // maximum angular speed
    }
    
    public VisionSubsystem(VisionIO io, CommandSwerveDrivetrain drivetrain) {
        this.io = io;
        this.drivetrain = drivetrain;
        
        // Configure Limelight if it's a Limelight implementation
        if (io instanceof VisionIOLimelight) {
            ((VisionIOLimelight) io).setPipeline(0); // Set to AprilTag pipeline
            io.setLeds(true);
        }
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        updateVisionState();
        logData();
    }
    
    private void updateVisionState() {
        if (!inputs.hasTargets) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(Units.radiansToDegrees(inputs.horizontalAngleRadians)) <= TARGET_LOCK_THRESHOLD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
    }
    
    public void setLeds(boolean enabled) {
        io.setLeds(enabled);
    }
    
    private void logData() {
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", inputs.tagId);
        SmartDashboard.putNumber("Vision/TX", Units.radiansToDegrees(inputs.horizontalAngleRadians));
        SmartDashboard.putNumber("Vision/TY", Units.radiansToDegrees(inputs.verticalAngleRadians));
        SmartDashboard.putBoolean("Vision/HasTargets", inputs.hasTargets);
    }
    
    // Getter methods for use in commands
    public VisionState getState() {
        return currentState;
    }
    
    public boolean hasTarget() {
        return inputs.hasTargets;
    }
    
    public double getHorizontalOffsetDegrees() {
        return Units.radiansToDegrees(inputs.horizontalAngleRadians);
    }
    
    public double getVerticalOffsetDegrees() {
        return Units.radiansToDegrees(inputs.verticalAngleRadians);
    }
    
    public int getTagId() {
        return inputs.tagId;
    }
    
    public double[] getBotPose() {
        return inputs.botpose.clone();
    }
    
    public double[] getBotPoseTargetSpace() {
        return inputs.botposeTargetSpace.clone();
    }
    
    public double getTimestamp() {
        return inputs.lastTimeStamp;
    }
    
    /**
     * Creates a command to align to an AprilTag using tag-relative positioning
     * @return A command that aligns the robot to the AprilTag
     */
    public Command createAlignToTagCommand() {
        return new AlignToTagCommand(this, drivetrain);
    }
}