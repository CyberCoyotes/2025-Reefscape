package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Command to align the robot relative to an AprilTag.
 * Can align to the left or right of the tag with a specified offset.
 */
public class AlignWithTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignToRightSide;
    
    // PID controllers for alignment
    private final PIDController xController = new PIDController(1.0, 0, 0);
    private final PIDController yController = new PIDController(1.0, 0, 0);
    private final PIDController rotationController = new PIDController(1.5, 0, 0.05);
    
    // Target pose we're trying to reach
    private Pose2d targetPose;
    
    // Request for controlling the swerve drivetrain
    private final SwerveRequest.ApplyChassisSpeeds chassisRequest = 
        new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);
    
    // Alignment thresholds
    private static final double POSITION_TOLERANCE = 0.05; // 5 cm
    private static final double ROTATION_TOLERANCE = 0.05; // ~3 degrees
    
    /**
     * Creates a new command to align with an AprilTag.
     * 
     * @param vision The vision subsystem
     * @param drivetrain The swerve drivetrain
     * @param alignToRightSide If true, align to the right of the tag. If false, align to the left.
     */
    public AlignWithTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, boolean alignToRightSide) {
        this.visionSubsystem = vision;
        this.drivetrain = drivetrain;
        this.alignToRightSide = alignToRightSide;
        
        // Configure PID controllers
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotationController.setTolerance(ROTATION_TOLERANCE);
        
        addRequirements(vision, drivetrain);
    }
    
    @Override
    public void initialize() {
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        // Calculate and set target pose
        calculateTargetPose();
        
        // Log the target pose
        if (targetPose != null) {
            SmartDashboard.putNumber("AlignWithTag/TargetX", targetPose.getX());
            SmartDashboard.putNumber("AlignWithTag/TargetY", targetPose.getY());
            SmartDashboard.putNumber("AlignWithTag/TargetRotation", targetPose.getRotation().getDegrees());
        }
    }
    
    private void calculateTargetPose() {
        // Get the tag pose
        Pose2d tagPose = visionSubsystem.getTagPose();
        
        if (tagPose != null) {
            // Calculate the offset based on whether we're aligning to the right or left
            Transform2d offset = visionSubsystem.calculateTagAlignment(alignToRightSide);
            
            // Apply the offset to get our target pose
            targetPose = tagPose.plus(offset);
        } else {
            targetPose = null;
        }
    }
    
    @Override
    public void execute() {
        // Update odometry with vision information
        visionSubsystem.updateOdometryWithVision();
        
        // If we don't have a target pose or lose vision, do nothing
        if (targetPose == null || !visionSubsystem.hasTarget()) {
            drivetrain.setControl(chassisRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }
        
        // Get current robot pose
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Calculate PID outputs
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotationSpeed = rotationController.calculate(
            currentPose.getRotation().getRadians(), 
            targetPose.getRotation().getRadians()
        );
        
        // Apply speed limits
        xSpeed = limitMagnitude(xSpeed, 2.0); // Max 2 m/s
        ySpeed = limitMagnitude(ySpeed, 2.0); // Max 2 m/s
        rotationSpeed = limitMagnitude(rotationSpeed, 1.5); // Max 1.5 rad/s
        
        // Create field-relative chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotationSpeed, currentPose.getRotation()
        );
        
        // Apply speeds to drivetrain
        drivetrain.setControl(chassisRequest.withSpeeds(speeds));
        
        // Log data
        SmartDashboard.putNumber("AlignWithTag/CurrentX", currentPose.getX());
        SmartDashboard.putNumber("AlignWithTag/CurrentY", currentPose.getY());
        SmartDashboard.putNumber("AlignWithTag/ErrorX", targetPose.getX() - currentPose.getX());
        SmartDashboard.putNumber("AlignWithTag/ErrorY", targetPose.getY() - currentPose.getY());
        SmartDashboard.putNumber("AlignWithTag/XSpeed", xSpeed);
        SmartDashboard.putNumber("AlignWithTag/YSpeed", ySpeed);
        SmartDashboard.putNumber("AlignWithTag/RotationSpeed", rotationSpeed);
    }
    
    private double limitMagnitude(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(chassisRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        
        SmartDashboard.putBoolean("AlignWithTag/Completed", !interrupted);
    }
    
    @Override
    public boolean isFinished() {
        // Command completes when we are at the target pose within tolerance
        if (targetPose == null || !visionSubsystem.hasTarget()) {
            return false;
        }
        
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotationController.atSetpoint();
    }
}