package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/**
 * Factory class for creating AprilTag alignment commands.
 */
public class TagAlignmentCommands {
    
    // PID constants for alignment
    private static final double ROTATION_P = 0.04;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.0;
    
    private static final double TRANSLATION_P = 1.0;
    private static final double TRANSLATION_I = 0.0;
    private static final double TRANSLATION_D = 0.0;
    
    // Dead zone for driver inputs
    private static final double DRIVER_DEADBAND = 0.1;
    
    // Maximum distance to consider for alignment
    private static final double MAX_ALIGNMENT_DISTANCE = 3.0; // meters
    
    /**
     * Creates a command to align to the left side of an AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param translationXSupplier Driver input for forward/backward movement
     * @param translationYSupplier Driver input for left/right movement
     * @param rotationSupplier Driver input for rotation
     * @return A command that aligns the robot to the left of the tag
     */
    public static Command alignToTagLeftSide(
            CommandSwerveDrivetrain drivetrain,
            LimelightVisionSubsystem vision,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
                
        return alignToTagWithOffset(
            drivetrain, 
            vision, 
            translationXSupplier, 
            translationYSupplier, 
            rotationSupplier, 
            true);
    }
    
    /**
     * Creates a command to align to the right side of an AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param translationXSupplier Driver input for forward/backward movement
     * @param translationYSupplier Driver input for left/right movement
     * @param rotationSupplier Driver input for rotation
     * @return A command that aligns the robot to the right of the tag
     */
    public static Command alignToTagRightSide(
            CommandSwerveDrivetrain drivetrain,
            LimelightVisionSubsystem vision,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
                
        return alignToTagWithOffset(
            drivetrain, 
            vision, 
            translationXSupplier, 
            translationYSupplier, 
            rotationSupplier, 
            false);
    }
    
    /**
     * Align to an offset position from the tag (left or right side).
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param translationXSupplier Driver input for forward/backward movement
     * @param translationYSupplier Driver input for left/right movement
     * @param rotationSupplier Driver input for rotation
     * @param isLeft True for left side alignment, false for right
     * @return A command for tag-relative alignment
     */
    private static Command alignToTagWithOffset(
            CommandSwerveDrivetrain drivetrain,
            LimelightVisionSubsystem vision,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            boolean isLeft) {
        
        // Create PID controllers for alignment
        PIDController rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        PIDController translationXController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
        PIDController translationYController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
        
        // Set up continuous rotation input
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Reset controllers when command initializes
        Runnable onInit = () -> {
            rotationController.reset();
            translationXController.reset();
            translationYController.reset();
            
            // Set tolerances
            rotationController.setTolerance(Math.toRadians(2.0));
            translationXController.setTolerance(0.05);  // 5cm tolerance
            translationYController.setTolerance(0.05);  // 5cm tolerance
        };
        
        // Supplier that gets the appropriate target pose
        Supplier<Pose2d> targetPoseSupplier = () -> {
            return isLeft ? vision.getLeftOffsetFromTag() : vision.getRightOffsetFromTag();
        };
        
        // Create the alignment command
        return Commands.run(
            () -> {
                // Get current robot pose from odometry
                Pose2d currentPose = drivetrain.getState().Pose;
                
                // Get target pose based on side selection
                Pose2d targetPose = targetPoseSupplier.get();
                
                // If we have a vision target and it's within range
                if (targetPose != null) {
                    // Calculate distance to target
                    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                    
                    // If we're within alignment distance
                    if (distance <= MAX_ALIGNMENT_DISTANCE) {
                        // Calculate alignment corrections
                        double rotationCorrection = rotationController.calculate(
                            currentPose.getRotation().getRadians(),
                            targetPose.getRotation().getRadians());
                            
                        // Get field-relative error vector
                        Translation2d translationError = targetPose.getTranslation().minus(currentPose.getTranslation());
                        
                        // Calculate translation corrections (field-relative)
                        double xCorrection = translationXController.calculate(0, translationError.getX());
                        double yCorrection = translationYController.calculate(0, translationError.getY());
                        
                        // Allow driver to override with significant inputs
                        double driverX = translationXSupplier.getAsDouble();
                        double driverY = translationYSupplier.getAsDouble();
                        double driverRotation = rotationSupplier.getAsDouble();
                        
                        // Blend driver and auto inputs
                        double finalX = Math.abs(driverX) > DRIVER_DEADBAND ? driverX : xCorrection;
                        double finalY = Math.abs(driverY) > DRIVER_DEADBAND ? driverY : yCorrection;
                        double finalRotation = Math.abs(driverRotation) > DRIVER_DEADBAND ? driverRotation : rotationCorrection;
                        
                        // Apply movement to the drivetrain (field-relative)
                        drivetrain.setControl(
                            new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric()
                                .withVelocityX(finalX)
                                .withVelocityY(finalY)
                                .withRotationalRate(finalRotation));
                                
                        return;
                    }
                }
                
                // If we don't have a target or it's too far, use manual control
                drivetrain.setControl(
                    new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric()
                        .withVelocityX(translationXSupplier.getAsDouble())
                        .withVelocityY(translationYSupplier.getAsDouble())
                        .withRotationalRate(rotationSupplier.getAsDouble()));
            },
            drivetrain
        ).beforeStarting(onInit);
    }
    
    /**
     * Creates a command to center on an AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param translationXSupplier Driver input for forward/backward movement
     * @param translationYSupplier Driver input for left/right movement
     * @param rotationSupplier Driver input for rotation
     * @return A command that centers the robot on the tag
     */
    public static Command centerOnTag(
            CommandSwerveDrivetrain drivetrain,
            LimelightVisionSubsystem vision,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        
        // Create PID controller for alignment
        PIDController horizontalController = new PIDController(0.03, 0.0, 0.001);
        
        // Reset controllers when command initializes
        Runnable onInit = () -> {
            horizontalController.reset();
            horizontalController.setTolerance(1.0);  // 1 degree tolerance
        };
        
        // Create the centering command using TX directly from Limelight
        return Commands.run(
            () -> {
                // Use TX directly from the Limelight for faster response
                if (vision.hasTarget()) {
                    double tx = vision.getHorizontalOffset();
                    
                    // Calculate rotation correction
                    double rotationCorrection = -horizontalController.calculate(tx, 0);
                    
                    // Allow driver to override with significant inputs
                    double driverRotation = rotationSupplier.getAsDouble();
                    
                    // Apply rotation and forward/backward from driver
                    drivetrain.setControl(
                        new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric()
                            .withVelocityX(translationXSupplier.getAsDouble())
                            .withVelocityY(translationYSupplier.getAsDouble())
                            .withRotationalRate(Math.abs(driverRotation) > DRIVER_DEADBAND ? 
                                driverRotation : rotationCorrection));
                    
                    return;
                }
                
                // If no target, use manual control
                drivetrain.setControl(
                    new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric()
                        .withVelocityX(translationXSupplier.getAsDouble())
                        .withVelocityY(translationYSupplier.getAsDouble())
                        .withRotationalRate(rotationSupplier.getAsDouble()));
            },
            drivetrain
        ).beforeStarting(onInit);
    }
}