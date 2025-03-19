package frc.robot.commands;

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

public class AlignWithTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignToRightSide;
    private final double distanceFromTag;
    
    // PID controllers for alignment
    private final PIDController xController = new PIDController(0.8, 0, 0.02);
    private final PIDController yController = new PIDController(0.8, 0, 0.02);
    private final PIDController rotationController = new PIDController(1.2, 0, 0.05);
    
    // Target pose we're trying to reach
    private Pose2d targetPose;
    
    // Request for controlling the swerve drivetrain
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    
    // Alignment thresholds
    private static final double POSITION_TOLERANCE = 0.04; // 4 cm
    private static final double ROTATION_TOLERANCE = 0.05; // ~3 degrees
    
    /**
     * Creates a new command to align with an AprilTag.
     * 
     * @param vision The vision subsystem
     * @param drivetrain The swerve drivetrain
     * @param alignToRightSide If true, align to the right of the tag. If false, align to the left.
     * @param distanceFromTag Distance in meters to position in front of the tag (default: 0.7m)
     */
    public AlignWithTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, 
                              boolean alignToRightSide, double distanceFromTag) {
        this.visionSubsystem = vision;
        this.drivetrain = drivetrain;
        this.alignToRightSide = alignToRightSide;
        this.distanceFromTag = distanceFromTag;
        
        // Configure PID controllers
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotationController.setTolerance(ROTATION_TOLERANCE);
        
        addRequirements(drivetrain);
    }
    
    /**
     * Overloaded constructor with default distance
     */
    public AlignWithTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, boolean alignToRightSide) {
        this(vision, drivetrain, alignToRightSide, 0.7);
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
        // Use the Limelight data to determine tag position
        double[] targetpose_robotspace = visionSubsystem.getTargetPoseRobotSpace();
        Pose2d robotPose = drivetrain.getState().Pose;
        
        if (visionSubsystem.hasTarget() && targetpose_robotspace.length >= 6) {
            // Calculate lateral offset (left/right of tag)
            double lateralOffset = alignToRightSide ? 0.35 : -0.35; // 35cm offset
            
            // Create a transform from the robot's current pose
            Transform2d tagToRobotTransform = new Transform2d(
                // Position in front of the tag at the specified distance
                -distanceFromTag, 
                lateralOffset,
                // Face the tag (opposite direction of the tag's orientation)
                robotPose.getRotation()
            );
            
            // Apply the transform to get the target pose
            targetPose = visionSubsystem.getTagPose().plus(tagToRobotTransform);
        } else {
            targetPose = null;
        }
    }
    
    @Override
    public void execute() {
        // Update target pose with latest vision data
        calculateTargetPose();
        
        // If we don't have a target pose or lose vision, stop
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
        xSpeed = limitMagnitude(xSpeed, 1.5); // Max 1.5 m/s
        ySpeed = limitMagnitude(ySpeed, 1.5); // Max 1.5 m/s
        rotationSpeed = limitMagnitude(rotationSpeed, 1.2); // Max 1.2 rad/s
        
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