package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private final String tableName;
    private final NetworkTable limelightTable;
    private final CommandSwerveDrivetrain drivetrain;
    
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

    /**
     * Constants for vision alignment PID controllers and targets
     */
    public static class AlignmentConstants {
        // PID values
        public static final class PID {
            // Forward control
            public static final double FORWARD_P = 0.5;
            public static final double FORWARD_I = 0.0;
            public static final double FORWARD_D = 0.0;
            public static final double FORWARD_TOLERANCE = 0.05; // meters

            // Lateral control
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
        public static final double TARGET_DISTANCE = 1.0; // meters
        public static final double MAX_SPEED = 1.0; // maximum speed for alignment
        public static final double MAX_ANGULAR_SPEED = 1.0; // maximum angular speed
        
        // Camera configuration
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12.5);
        public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(25.5);
        public static final double CAMERA_ANGLE_DEGREES = 0.0;
    }

    public VisionSubsystem(String tableName, CommandSwerveDrivetrain drivetrain) {
        this.tableName = tableName;
        this.drivetrain = drivetrain;
        
        // Initialize NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");
        
        // Configure Limelight
        configureLimelight();
    }

    private void configureLimelight() {
        // Set to AprilTag pipeline
        limelightTable.getEntry("pipeline").setNumber(0);
        setLeds(true);
        ledsEnabled = true;
    }

    @Override
    public void periodic() {
        updateVisionState();
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
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", tid.getDouble(0));
        SmartDashboard.putNumber("Vision/TX", tx.getDouble(0));
        SmartDashboard.putNumber("Vision/TY", ty.getDouble(0));
        SmartDashboard.putNumber("Vision/TA", ta.getDouble(0));
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

    public String getTableName() {
        return tableName;
    }

    /**
     * Creates a command to align to an AprilTag in both X and Y directions
     * 
     * @return A command that aligns the robot to the AprilTag
     */
    public Command createAlignToTagCommand() {
        // PID Controllers for both axes
        final PIDController forwardController = new PIDController(
            AlignmentConstants.PID.FORWARD_P,
            AlignmentConstants.PID.FORWARD_I,
            AlignmentConstants.PID.FORWARD_D
        );
        forwardController.setTolerance(AlignmentConstants.PID.FORWARD_TOLERANCE);

        final PIDController lateralController = new PIDController(
            AlignmentConstants.PID.LATERAL_P,
            AlignmentConstants.PID.LATERAL_I,
            AlignmentConstants.PID.LATERAL_D
        );
        lateralController.setTolerance(AlignmentConstants.PID.LATERAL_TOLERANCE);

        final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

        return new Command() {
            @Override
            public void initialize() {
                setLeds(true);
                logInitialSetup();
            }

            @Override
            public void execute() {
                if (!hasTarget()) {
                    handleNoTarget();
                    return;
                }

                // Get measurements
                double tx = getHorizontalOffset();
                double ty = getVerticalOffset();
                
                // Calculate distance from camera to target
                double currentDistance = calculateDistance(ty);
                
                // Calculate control outputs
                double forwardSpeed = forwardController.calculate(currentDistance, AlignmentConstants.TARGET_DISTANCE);
                double lateralSpeed = lateralController.calculate(tx, 0);
                
                // Apply speed limits
                forwardSpeed = MathUtil.clamp(forwardSpeed, -AlignmentConstants.MAX_SPEED, AlignmentConstants.MAX_SPEED);
                lateralSpeed = MathUtil.clamp(lateralSpeed, -AlignmentConstants.MAX_SPEED, AlignmentConstants.MAX_SPEED);

                // Log data
                logAlignmentData(tx, ty, currentDistance, forwardSpeed, lateralSpeed, 
                    forwardController.atSetpoint(), lateralController.atSetpoint());

                // Apply combined control
                drivetrain.setControl(drive
                    .withVelocityX(forwardSpeed)  // Forward/back
                    .withVelocityY(lateralSpeed)  // Left/right
                    .withRotationalRate(0));      // No rotation
            }

            @Override
            public boolean isFinished() {
                return forwardController.atSetpoint() && lateralController.atSetpoint();
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.stop();
                SmartDashboard.putString("AlignTag/Status", interrupted ? "INTERRUPTED" : "COMPLETED");
            }
        };
    }

    /**
     * Creates a command to align to an AprilTag with full 3-axis control (X, Y, and rotation)
     * 
     * @return A command that aligns the robot to the AprilTag in position and orientation
     */
    public Command createFullAlignToTagCommand() {
        // PID Controllers for all three axes
        final PIDController forwardController = new PIDController(
            AlignmentConstants.PID.FORWARD_P,
            AlignmentConstants.PID.FORWARD_I,
            AlignmentConstants.PID.FORWARD_D
        );
        forwardController.setTolerance(AlignmentConstants.PID.FORWARD_TOLERANCE);

        final PIDController lateralController = new PIDController(
            AlignmentConstants.PID.LATERAL_P,
            AlignmentConstants.PID.LATERAL_I,
            AlignmentConstants.PID.LATERAL_D
        );
        lateralController.setTolerance(AlignmentConstants.PID.LATERAL_TOLERANCE);

        final PIDController rotationController = new PIDController(
            AlignmentConstants.PID.ROTATION_P,
            AlignmentConstants.PID.ROTATION_I,
            AlignmentConstants.PID.ROTATION_D
        );
        rotationController.setTolerance(AlignmentConstants.PID.ROTATION_TOLERANCE);

        final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

        return new Command() {
            @Override
            public void initialize() {
                setLeds(true);
                logInitialSetup();
            }

            @Override
            public void execute() {
                if (!hasTarget()) {
                    handleNoTarget();
                    return;
                }

                // Get measurements
                double tx = getHorizontalOffset();
                double ty = getVerticalOffset();
                
                // Calculate distance from camera to target
                double currentDistance = calculateDistance(ty);
                
                // Calculate control outputs
                double forwardSpeed = forwardController.calculate(currentDistance, AlignmentConstants.TARGET_DISTANCE);
                double lateralSpeed = lateralController.calculate(tx, 0);
                
                // Note: For rotation, we would ideally use the target skew or pose estimation
                // This is a placeholder - you'll need to implement the rotation sensing and control
                double rotationRate = 0; // rotationController.calculate(measuredRotation, 0);
                
                // Apply speed limits
                forwardSpeed = MathUtil.clamp(forwardSpeed, -AlignmentConstants.MAX_SPEED, AlignmentConstants.MAX_SPEED);
                lateralSpeed = MathUtil.clamp(lateralSpeed, -AlignmentConstants.MAX_SPEED, AlignmentConstants.MAX_SPEED);
                rotationRate = MathUtil.clamp(rotationRate, -AlignmentConstants.MAX_ANGULAR_SPEED, AlignmentConstants.MAX_ANGULAR_SPEED);

                // Log data
                logAlignmentData(tx, ty, currentDistance, forwardSpeed, lateralSpeed, 
                    forwardController.atSetpoint(), lateralController.atSetpoint());

                // Apply combined control
                drivetrain.setControl(drive
                    .withVelocityX(forwardSpeed)    // Forward/back
                    .withVelocityY(lateralSpeed)    // Left/right
                    .withRotationalRate(rotationRate)); // Rotation
            }

            @Override
            public boolean isFinished() {
                return forwardController.atSetpoint() && 
                       lateralController.atSetpoint() && 
                       rotationController.atSetpoint();
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.stop();
                SmartDashboard.putString("AlignTag/Status", interrupted ? "INTERRUPTED" : "COMPLETED");
            }
        };
    }

    /**
     * Calculate the distance to the target based on the vertical angle
     * 
     * @param ty The vertical angle to the target in degrees
     * @return The calculated distance in meters
     */
    private double calculateDistance(double ty) {
        double angleToTarget = AlignmentConstants.CAMERA_ANGLE_DEGREES + ty;
        return Math.abs((AlignmentConstants.TARGET_HEIGHT_METERS - AlignmentConstants.CAMERA_HEIGHT_METERS) / 
                       Math.tan(Math.toRadians(angleToTarget)));
    }

    /**
     * Handle the case when no target is visible
     */
    private void handleNoTarget() {
        drivetrain.stop();
        SmartDashboard.putString("AlignTag/Status", "NO TARGET");
    }

    /**
     * Log data during alignment
     */
    private void logAlignmentData(double tx, double ty, double distance, 
                                double forwardSpeed, double lateralSpeed,
                                boolean atForwardSetpoint, boolean atLateralSetpoint) {
        // SmartDashboard logging
        SmartDashboard.putNumber("AlignTag/TX", tx);
        SmartDashboard.putNumber("AlignTag/TY", ty);
        SmartDashboard.putNumber("AlignTag/Distance", distance);
        SmartDashboard.putNumber("AlignTag/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("AlignTag/LateralSpeed", lateralSpeed);
        SmartDashboard.putBoolean("AlignTag/AtForwardSetpoint", atForwardSetpoint);
        SmartDashboard.putBoolean("AlignTag/AtLateralSetpoint", atLateralSetpoint);
    }

    /**
     * Log initial setup info
     */
    private void logInitialSetup() {
        SmartDashboard.putString("AlignTag/Status", "RUNNING");
        SmartDashboard.putNumber("AlignTag/TargetDistance", AlignmentConstants.TARGET_DISTANCE);
    }
}