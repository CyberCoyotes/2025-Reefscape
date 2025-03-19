package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FineTagAlignmentCommand extends Command {
    private final int targetTagId;
    private final VisionSubsystem visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignToRightSide;
    
    // PID controllers
    private final PIDController lateralController = new PIDController(0.7, 0, 0.01);
    private final PIDController distanceController = new PIDController(0.6, 0, 0.01);
    private final PIDController headingController = new PIDController(1.0, 0, 0.02);
    
    // Target values
    private final double targetDistance = 0.65; // meters
    private final double targetLateralOffset; // calculated based on alignToRightSide
    private final double targetHeading = 0.0; // radians
    
    // Request for controlling the swerve drivetrain
    private final SwerveRequest.RobotCentric robotRequest = new SwerveRequest.RobotCentric();
    
    // Timeouts and tolerances
    private static final double MAX_ALIGNMENT_TIME = 2.0; // seconds
    private static final double LATERAL_TOLERANCE = 0.035; // meters (3.5cm)
    private static final double DISTANCE_TOLERANCE = 0.05; // meters (5cm)
    private static final double HEADING_TOLERANCE = 0.03; // radians (~1.7 degrees)
    
    private final Timer alignmentTimer = new Timer();
    private boolean correctTagFound = false;
    
    public FineTagAlignmentCommand(int targetTagId, VisionSubsystem vision, 
                                  CommandSwerveDrivetrain drivetrain, boolean alignToRightSide) {
        this.targetTagId = targetTagId;
        this.visionSubsystem = vision;
        this.drivetrain = drivetrain;
        this.alignToRightSide = alignToRightSide;
        this.targetLateralOffset = alignToRightSide ? 0.3 : -0.3; // 30cm offset
        
        // Configure PID controllers
        lateralController.setTolerance(LATERAL_TOLERANCE);
        distanceController.setTolerance(DISTANCE_TOLERANCE);
        headingController.setTolerance(HEADING_TOLERANCE);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivetrain, vision);
    }
    
    @Override
    public void initialize() {
        alignmentTimer.reset();
        alignmentTimer.start();
        correctTagFound = false;
        
        lateralController.reset();
        distanceController.reset();
        headingController.reset();
    }
    
    @Override
    public void execute() {
        // Check if we are seeing the correct tag
        if (visionSubsystem.hasTarget() && visionSubsystem.getTagId() == targetTagId) {
            correctTagFound = true;
            
            // Get position data relative to the tag
            double[] botpose_targetspace = LimelightHelpers.getBotPose_TargetSpace(visionSubsystem.getTableName());
            
            if (botpose_targetspace.length >= 6) {
                // Extract position relative to tag
                double lateralPosition = botpose_targetspace[0]; // X in target space
                double distanceFromTag = -botpose_targetspace[2]; // Z in target space (negative because Z points outward from tag)
                double heading = Math.toRadians(botpose_targetspace[5]); // Yaw in target space
                
                // Calculate control outputs
                double lateralSpeed = lateralController.calculate(lateralPosition, targetLateralOffset);
                double forwardSpeed = distanceController.calculate(distanceFromTag, targetDistance);
                double rotationSpeed = headingController.calculate(heading, targetHeading);
                
                // Limit speeds
                lateralSpeed = limitMagnitude(lateralSpeed, 0.8);
                forwardSpeed = limitMagnitude(forwardSpeed, 0.8);
                rotationSpeed = limitMagnitude(rotationSpeed, 0.8);
                
                // Apply robot-centric control (since we're aligning relative to the tag)
                drivetrain.setControl(
                    robotRequest
                        .withVelocityX(forwardSpeed)
                        .withVelocityY(lateralSpeed)
                        .withRotationalRate(rotationSpeed)
                );
                
                // Log data
                SmartDashboard.putNumber("FineAlign/LateralError", targetLateralOffset - lateralPosition);
                SmartDashboard.putNumber("FineAlign/DistanceError", targetDistance - distanceFromTag);
                SmartDashboard.putNumber("FineAlign/HeadingError", targetHeading - heading);
            }
        } else {
            // If we don't see the tag, hold position
            drivetrain.setControl(robotRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }
    }
    
    private double limitMagnitude(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
    
    @Override
    public void end(boolean interrupted) {
        alignmentTimer.stop();
        drivetrain.setControl(robotRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        
        SmartDashboard.putBoolean("FineAlign/Completed", !interrupted && isAligned());
    }
    
    private boolean isAligned() {
        return lateralController.atSetpoint() && 
               distanceController.atSetpoint() && 
               headingController.atSetpoint();
    }
    
    @Override
    public boolean isFinished() {
        // Finish when aligned or if time expires
        return (correctTagFound && isAligned()) || alignmentTimer.hasElapsed(MAX_ALIGNMENT_TIME);
    }
}