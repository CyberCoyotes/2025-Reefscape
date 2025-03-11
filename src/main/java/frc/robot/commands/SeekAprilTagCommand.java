package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionState;

/**
 * Command to rotate the robot until it finds an AprilTag, then center on it.
 * This is useful when we know tags are nearby but not in view.
 */
public class SeekAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    
    // PID controller for centering on tag
    private final PIDController horizontalController = new PIDController(0.03, 0, 0.002);
    
    // Request for controlling the swerve drivetrain
    private final SwerveRequest.ApplyChassisSpeeds chassisRequest = 
        new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);
    
    // Constants
    private static final double SEEK_ROTATION_SPEED = 0.6; // rad/s
    private static final double TARGET_THRESHOLD = 1.0; // degrees
    private static final double MAX_SEEK_TIME = 5.0; // seconds
    
    private double startTime;
    private boolean tagFound = false;
    
    /**
     * Creates a new command to seek and center on an AprilTag.
     * 
     * @param vision The vision subsystem
     * @param drivetrain The swerve drivetrain
     */
    public SeekAprilTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.visionSubsystem = vision;
        this.drivetrain = drivetrain;
        
        // Configure the PID controller
        horizontalController.setTolerance(TARGET_THRESHOLD);
        
        addRequirements(vision, drivetrain);
    }
    
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis() / 1000.0;
        tagFound = false;
        horizontalController.reset();
    }
    
    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {
            tagFound = true;
            
            // We have a target, so center on it
            double horizontalOffset = visionSubsystem.getHorizontalOffset();
            double rotationSpeed = horizontalController.calculate(horizontalOffset, 0);
            
            // Limit rotation speed
            rotationSpeed = Math.max(-1.5, Math.min(1.5, rotationSpeed));
            
            drivetrain.setControl(chassisRequest.withSpeeds(new ChassisSpeeds(0, 0, rotationSpeed)));
            
            SmartDashboard.putNumber("SeekTag/HorizontalOffset", horizontalOffset);
            SmartDashboard.putNumber("SeekTag/RotationSpeed", rotationSpeed);
        } else {
            // No target found, so rotate to search
            drivetrain.setControl(chassisRequest.withSpeeds(new ChassisSpeeds(0, 0, SEEK_ROTATION_SPEED)));
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(chassisRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        
        SmartDashboard.putBoolean("SeekTag/TagFound", tagFound);
    }
    
    @Override
    public boolean isFinished() {
        // Command completes when we are centered on a tag or time runs out
        double currentTime = System.currentTimeMillis() / 1000.0;
        boolean timeExpired = (currentTime - startTime) > MAX_SEEK_TIME;
        
        // If we've found a tag and centered on it, or time has expired
        if (tagFound && horizontalController.atSetpoint()) {
            return true;
        }
        
        return timeExpired;
    }
}