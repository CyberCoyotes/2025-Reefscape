package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlignToReefTagRelative extends Command {
    private boolean isRightScore;
    private Timer dontSeeTagTimer;
    private CommandSwerveDrivetrain drivebase;
    
    // Create SwerveRequest for robot-centric control
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();

    // Simple proportional control with very low values
    private double Y_GAIN = 1;  // Very conservative
    private double ROT_GAIN = 0.5; // Very conservative
    
    // Target position values
    private double Y_SETPOINT = 0.15;  // Target Y offset in meters
    
    // Deadband values to prevent tiny movements
    private double Y_DEADBAND = 0.07;  // 7cm deadband
    private double ROT_DEADBAND = 0.05; // ~3 degrees deadband
    
    // Timers
    private double DONT_SEE_TAG_WAIT_TIME = 0.3;

    public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
        this.isRightScore = isRightScore;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();
        
        // Reset visual indicator
        SmartDashboard.putBoolean("Vision/AlignmentReady", false);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("")) {
            // Reset timer when tag is visible
            this.dontSeeTagTimer.reset();

            // Get current pose data from Limelight
            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            
            // Print raw values to understand what's happening
            SmartDashboard.putNumber("Vision/RawTagX", positions[0]);
            SmartDashboard.putNumber("Vision/RawTagZ", positions[2]);
            SmartDashboard.putNumber("Vision/RawTagPitch", positions[4]);
            
            // Calculate Y error (using positions[0] for side-to-side)
            double yTarget = isRightScore ? Y_SETPOINT : -Y_SETPOINT;
            double yError = yTarget - positions[0];
            
            // Calculate rotation error (using positions[4] for rotation)
            double rotError = -positions[4];  // Target is 0
            
            // Apply deadband - only move if error is significant
            double ySpeed = 0;
            if (Math.abs(yError) > Y_DEADBAND) {
                ySpeed = Y_GAIN * yError;
            }
            
            double rotSpeed = 0;
            if (Math.abs(rotError) > ROT_DEADBAND) {
                rotSpeed = ROT_GAIN * rotError;
            }
            
            // Apply speed limits
            ySpeed = Math.min(Math.max(ySpeed, -0.3), 0.3);     // Limit to ±0.3 m/s
            rotSpeed = Math.min(Math.max(rotSpeed, -0.3), 0.3); // Limit rotation speed
            
            // Display errors and speeds
            SmartDashboard.putNumber("Vision/YError", yError);
            SmartDashboard.putNumber("Vision/RotError", rotError);
            SmartDashboard.putNumber("Vision/YSpeed", ySpeed);
            SmartDashboard.putNumber("Vision/RotSpeed", rotSpeed);
            
            // Apply control - only Y and rotation, no X movement
            drivebase.setControl(
                robotCentricRequest
                    .withVelocityX(0)        // No forward/backward movement
                    .withVelocityY(ySpeed)   // Side-to-side alignment
                    .withRotationalRate(rotSpeed)  // Rotational alignment
            );
            
            // Show if we're within deadband
            boolean yAligned = Math.abs(yError) <= Y_DEADBAND;
            boolean rotAligned = Math.abs(rotError) <= ROT_DEADBAND;
            SmartDashboard.putBoolean("Vision/YAligned", yAligned);
            SmartDashboard.putBoolean("Vision/RotAligned", rotAligned);
            SmartDashboard.putBoolean("Vision/AlignmentReady", yAligned && rotAligned);
            
        } else {
            // Stop movement when no tag is visible
            drivebase.setControl(
                robotCentricRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
            
            // Indicate that alignment is not ready when tag is lost
            SmartDashboard.putBoolean("Vision/AlignmentReady", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure the robot stops when command ends
        drivebase.setControl(
            robotCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        
        // Reset alignment ready indicator
        SmartDashboard.putBoolean("Vision/AlignmentReady", false);
    }

    @Override
    public boolean isFinished() {
        // Only finish if we've lost sight of the tag for too long
        return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME);
    }
}