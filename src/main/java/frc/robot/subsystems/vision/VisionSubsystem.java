package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.led.LEDState;
// import frc.robot.subsystems.led.LEDSubsystem;

@SuppressWarnings("unused")

public class VisionSubsystem extends SubsystemBase {
    private final String tableName;
    private final NetworkTable limelightTable;
    private final CommandSwerveDrivetrain drivetrain;
    // private final LEDSubsystem leds;
    
    // NetworkTable entries
    private final NetworkTableEntry tv; // Whether there are valid targets
    private final NetworkTableEntry tx; // Horizontal offset
    private final NetworkTableEntry ty; // Vertical offset
    private final NetworkTableEntry ta; // Target area
    private final NetworkTableEntry tid; // AprilTag ID
    
    // Vision processing constants
    private static final double TARGET_LOCK_THRESHOLD = 2.0; // Degrees
    private static final double VALID_TARGET_AREA = 0.1; // % of image
    
    private VisionState currentState = VisionState.NO_TARGET;
    private boolean ledsEnabled = false;

    public VisionSubsystem(String tableName, CommandSwerveDrivetrain drivetrain/* , LEDSubsystem leds*/) {
        this.tableName = tableName;
        this.drivetrain = drivetrain;
        // this.leds = leds;
        
        // Initialize NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");
        
        // Configure Limelight
        configureLimelight();
        // setLeds(false);
        // ledsEnabled = false;
    }

    private void configureLimelight() {
        // Set to AprilTag pipeline
        limelightTable.getEntry("pipeline").setNumber(0);
        setLeds(true); // Turn off LEDs if false
        ledsEnabled = true;

        // NetworkTableInstance.getDefault().flush();

    }

    @Override
    public void periodic() {
        updateVisionState();
        // updateLEDs();
        logData();
    }

    private void updateVisionState() {
        boolean hasTarget = tv.getDouble(0.0) > 0.5;
        double horizontalOffset = tx.getDouble(0.0);
        double area = ta.getDouble(0.0);

        if (!hasTarget || area < VALID_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(horizontalOffset) <= TARGET_LOCK_THRESHOLD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
    }

    public void setLeds(boolean enabled) {
        ledsEnabled = enabled;
        limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1); // 3=force on, 1=force off
    }

    private void logData() {
        // Current state and basic targeting info
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", tid.getDouble(0));
        SmartDashboard.putNumber("Vision/TX", tx.getDouble(0));
        SmartDashboard.putNumber("Vision/TY", ty.getDouble(0));
        SmartDashboard.putNumber("Vision/TA", ta.getDouble(0));
        
        // Add timestamp and latency information
        SmartDashboard.putNumber("Vision/Timestamp", limelightTable.getEntry("ts").getDouble(0));
        SmartDashboard.putNumber("Vision/Pipeline_Latency_ms", limelightTable.getEntry("tl").getDouble(0));
        SmartDashboard.putNumber("Vision/Capture_Latency_ms", limelightTable.getEntry("cl").getDouble(0));
        
        // Add 3D pose data if available
        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        if (botpose.length >= 6) {
            SmartDashboard.putNumber("Vision/PoseX", botpose[0]);
            SmartDashboard.putNumber("Vision/PoseY", botpose[1]);
            SmartDashboard.putNumber("Vision/PoseZ", botpose[2]);
            SmartDashboard.putNumber("Vision/PoseRoll", botpose[3]);
            SmartDashboard.putNumber("Vision/PosePitch", botpose[4]);
            SmartDashboard.putNumber("Vision/PoseYaw", botpose[5]);
        }
        
        // Detailed target tracking status
        SmartDashboard.putBoolean("Vision/HasTarget", tv.getDouble(0) > 0.5);
        SmartDashboard.putBoolean("Vision/TargetLocked", currentState == VisionState.TARGET_LOCKED);
    }
    

    // Getter methods for use in commands
    public VisionState getState() {
        return currentState;
    }

    public boolean hasTarget() {
        return currentState != VisionState.NO_TARGET;
    }

    public double getHorizontalOffset() {
        return tx.getDouble(0.0);
    }

    public double getVerticalOffset() {
        return ty.getDouble(0.0);
    }

    public int getTagId() {
        return (int) tid.getDouble(0);
    }

}