package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Factory for creating alignment commands with different parameters
 */
public class AlignmentCommandFactory {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    
    /**
     * Creates a new AlignmentCommandFactory
     * 
     * @param drivetrain The drivetrain subsystem
     * @param visionSubsystem The vision subsystem
     */
    public AlignmentCommandFactory(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
    }
    
    /**
     * Creates a command to align to an AprilTag relative to the tag
     * 
     * @param isRightScore Whether to align to the right side of the target
     * @return The alignment command
     */
    public Command createAlignToReefTagRelative(boolean isRightScore) {
        return new AlignToReefTagRelative(isRightScore, drivetrain, visionSubsystem);
    }
    
    /**
     * Creates a sequence to enable vision mode, align to a tag, and then return to driver mode
     * 
     * @param isRightScore Whether to align to the right side of the target
     * @return The command sequence
     */
    public Command createVisionAlignSequence(boolean isRightScore) {
        return Commands.sequence(
            // Start with vision processing enabled and LEDs on
            Commands.runOnce(() -> {
                visionSubsystem.setDriverMode(false);
                visionSubsystem.setLeds(true);
            }),
            // Wait for the camera to adjust
            Commands.waitSeconds(0.1),
            // Perform the alignment
            createAlignToReefTagRelative(isRightScore),
            // Return to driver mode with LEDs off when done
            Commands.runOnce(() -> {
                visionSubsystem.setDriverMode(true);
                visionSubsystem.setLeds(false);
            })
        );
    }
    
    /**
     * Creates a command to reset the robot's pose estimate based on vision
     * 
     * @return The command to reset pose
     */
    public Command createResetPoseFromVision() {
        return Commands.runOnce(() -> {
            if (visionSubsystem.hasTarget()) {
                Pose2d visionPose = visionSubsystem.getBotPose2d_wpiBlue();
                drivetrain.resetPose(visionPose);
            }
        });
    }
    
    /**
     * Creates a command that enables auto aim while driving
     * Uses a hybrid of driver controls and vision feedback
     * 
     * @return The command for auto aim while driving
     */
    public Command createDriverAssistedAiming() {
        // This would be implemented with your specific requirements
        // This is just a placeholder example
        return Commands.none();
    }
}