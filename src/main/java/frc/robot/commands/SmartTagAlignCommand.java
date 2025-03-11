package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * A smart command that first seeks an AprilTag, then decides whether to align
 * to the left or right side based on the robot's current position.
 */
public class SmartTagAlignCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private Command activeCommand = null;
    
    /**
     * Creates a new smart tag alignment command.
     * 
     * @param vision The vision subsystem
     * @param drivetrain The drivetrain subsystem
     */
    public SmartTagAlignCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        // No need to add requirements as we'll be scheduling other commands
        // that will add their own requirements
    }
    
    @Override
    public void initialize() {
        // If we don't see a tag yet, first try to find one
        if (!vision.hasTarget()) {
            activeCommand = new SeekAprilTagCommand(vision, drivetrain);
            activeCommand.schedule();
        } else {
            // We already see a tag, so determine alignment side and go
            chooseAndScheduleAlignment();
        }
    }
    
    private void chooseAndScheduleAlignment() {
        // Get the tag pose
        Pose2d tagPose = vision.getTagPose();
        Pose2d robotPose = drivetrain.getState().Pose;
        
        if (tagPose == null) {
            // If we somehow lost the tag, cancel
            cancel();
            return;
        }
        
        // Determine which side to align to based on robot position
        // If the robot is to the right of the tag, align to the right
        // If the robot is to the left of the tag, align to the left
        boolean alignToRight = robotPose.getY() > tagPose.getY();
        
        // Create and schedule the alignment command
        activeCommand = new SequentialCommandGroup(
            new AlignWithTagCommand(vision, drivetrain, alignToRight)
        );
        
        activeCommand.schedule();
    }
    
    @Override
    public void execute() {
        // If the seek command has finished and we now see a tag, start alignment
        if (activeCommand instanceof SeekAprilTagCommand && 
            !activeCommand.isScheduled() && vision.hasTarget()) {
            chooseAndScheduleAlignment();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Cancel the active command if we're interrupted
        if (activeCommand != null && activeCommand.isScheduled()) {
            activeCommand.cancel();
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish when the active command is done
        return activeCommand != null && !activeCommand.isScheduled();
    }
}