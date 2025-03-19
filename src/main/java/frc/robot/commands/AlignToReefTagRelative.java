package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Command to align the robot to an AprilTag in target-relative space
 */
public class AlignToReefTagRelative extends Command {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    private final boolean isRightScore;
    private final Timer dontSeeTagTimer;
    private final Timer stopTimer;
    private final CommandSwerveDrivetrain drivebase;
    private final VisionSubsystem visionSubsystem;
    
    // Create SwerveRequest for robot-centric control
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();

    /**
     * Creates a new AlignToReefTagRelative command
     * 
     * @param isRightScore Whether to align to the right side of the target
     * @param drivebase The drivetrain subsystem
     * @param visionSubsystem The vision subsystem
     */
    public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase, 
                                VisionSubsystem visionSubsystem) {
        this.xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0, 0);
        this.yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0, 0);
        this.rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);
        this.isRightScore = isRightScore;
        this.drivebase = drivebase;
        this.visionSubsystem = visionSubsystem;
        this.dontSeeTagTimer = new Timer();
        this.stopTimer = new Timer();
        
        // Require the drivetrain subsystem
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        // Start timers
        this.stopTimer.reset();
        this.stopTimer.start();
        this.dontSeeTagTimer.reset();
        this.dontSeeTagTimer.start();

        // Configure the rotation PID controller
        rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        // Configure the X (forward/back) PID controller
        xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

        // Configure the Y (left/right) PID controller
        // Positive Y for right score, negative Y for left score
        yController.setSetpoint(isRightScore ? 
            VisionConstants.Y_SETPOINT_REEF_ALIGNMENT : 
            -VisionConstants.Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);
        
        // Log initialization
        SmartDashboard.putString("Alignment/Status", "Initialized");
        SmartDashboard.putString("Alignment/Target", isRightScore ? "Right" : "Left");
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {
            // Reset the "don't see tag" timer when we see a tag
            this.dontSeeTagTimer.reset();
            
            // Get the robot pose in target space
            double[] positions = visionSubsystem.getBotPose_TargetSpace();
            
            // Verify we have valid pose data
            if (positions.length < 6) {
                SmartDashboard.putString("Alignment/Error", "Invalid pose data");
                stopRobot();
                return;
            }
            
            // Calculate control outputs
            // X is the Z axis in target space (forward/back)
            double xSpeed = xController.calculate(positions[2]);
            // Y is the X axis in target space (left/right) - negative because of coordinate system
            double ySpeed = -yController.calculate(positions[0]);
            // Rotation is the pitch in target space - negative because of coordinate system
            double rotValue = -rotController.calculate(positions[4]);
            
            // Log debugging data
            SmartDashboard.putNumber("Alignment/X_Position", positions[2]);
            SmartDashboard.putNumber("Alignment/Y_Position", positions[0]);
            SmartDashboard.putNumber("Alignment/Rotation", positions[4]);
            SmartDashboard.putNumber("Alignment/X_Speed", xSpeed);
            SmartDashboard.putNumber("Alignment/Y_Speed", ySpeed);
            SmartDashboard.putNumber("Alignment/Rot_Speed", rotValue);
            SmartDashboard.putNumber("Alignment/X_Error", xController.getPositionError());
            SmartDashboard.putNumber("Alignment/Y_Error", yController.getPositionError());
            SmartDashboard.putNumber("Alignment/Rot_Error", rotController.getPositionError());
            
            // Use robot-centric control for alignment
            if (yController.getPositionError() < VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT) {
                // If Y is aligned, use all axes
                drivebase.setControl(
                    robotCentricRequest
                        .withVelocityX(xSpeed)  // Forward/backward
                        .withVelocityY(ySpeed)  // Left/right 
                        .withRotationalRate(rotValue)  // Rotation rate
                );
            } else {
                // If Y is not aligned, prioritize Y and rotation alignment
                drivebase.setControl(
                    robotCentricRequest
                        .withVelocityX(0)  // No forward/backward motion
                        .withVelocityY(ySpeed)  // Left/right
                        .withRotationalRate(rotValue)  // Rotation rate
                );
            }

            // Reset the stop timer if we're not at the setpoint
            if (!isAtSetpoint()) {
                stopTimer.reset();
            }
            
            // Log alignment status
            SmartDashboard.putBoolean("Alignment/AtSetpoint", isAtSetpoint());
            SmartDashboard.putString("Alignment/Status", "Aligning");
        } else {
            // No target visible
            SmartDashboard.putString("Alignment/Status", "No target");
            stopRobot();
        }
    }

    /**
     * Checks if all controllers are at their setpoints
     * 
     * @return true if all controllers are at their setpoints
     */
    private boolean isAtSetpoint() {
        return rotController.atSetpoint() && 
               yController.atSetpoint() && 
               xController.atSetpoint();
    }
    
    /**
     * Stops the robot movement
     */
    private void stopRobot() {
        drivebase.setControl(
            robotCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure the robot stops when command ends
        stopRobot();
        
        // Log end status
        SmartDashboard.putString("Alignment/Status", 
            interrupted ? "Interrupted" : "Completed");
    }

    @Override
    public boolean isFinished() {
        // Command is finished if:
        // 1. We don't see a tag for DONT_SEE_TAG_WAIT_TIME seconds, or
        // 2. We've been at the setpoint for POSE_VALIDATION_TIME seconds
        return this.dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
               (isAtSetpoint() && 
                stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME));
    }
}