package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrontTOFSubsystem;

/**
 * Command for aligning to reef branches using TOF sensor and swerve drivetrain.
 * Uses a two-phase approach:
 * 1. Strafe right until we detect an opening (TOF distance exceeds threshold)
 * 2. Strafe left by a predefined distance based on which branch we're targeting
 */
public class AlignToReefWithEdgeDetection extends SequentialCommandGroup {
    
    private static final double RIGHT_BRANCH_OFFSET =   1.600; // strafe for right branch
    private static final double LEFT_BRANCH_OFFSET =    3.300; // strafe for left branch 
/*
    | YSNP  | RIGHT | LEFT | STRAFE_SPEED |
    |-------|-------|-------|--------------|
    |  375  | 0.3556| 0.3000|     0.5      |
    |  400  | 0.3556| 0.3000|     0.5      |
    |  400  | 1.200 | 1.2858|     0.5      |
    |  400  | 1.300 | 1.2858|     0.75     |
    |  400  | 1.300 | 3.200 |     1.00     |
    |  400  | 1.500 | 3.200 |     1.00     |
    |  400  | 1.600 | 3.300 |     1.00     |
    ----------------------------------------
    Strafe speed is too inaccurate going any faster

 */
    private static final double STRAFE_SPEED = 1.0; // 50% of max speed for strafing
    
    // Request for robot-centric strafing
    private final SwerveRequest.RobotCentric strafeRequest = new SwerveRequest.RobotCentric();
    
    /**
     * Creates a new ReefBranchAlignCommand.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param tofSensor The time of flight sensor subsystem
     * @param isLeftBranch Whether to align to the left branch (true) or right branch (false)
     */
    public AlignToReefWithEdgeDetection(CommandSwerveDrivetrain drivetrain, FrontTOFSubsystem tofSensor, boolean isLeftBranch) {
        // Step 1: Strafe right until we detect an opening
        addCommands(
            // Log the start of alignment process
            Commands.runOnce(() -> System.out.println("Starting reef alignment for " + 
                (isLeftBranch ? "LEFT" : "RIGHT") + " branch")),
            
            // Strafe right until TOF sensor detects an opening
            drivetrain.run(() -> {
                drivetrain.setControl(strafeRequest
                    .withVelocityX(0)
                    .withVelocityY(-STRAFE_SPEED) // Negative Y is right in robot-centric frame
                    .withRotationalRate(0));
            }).until(() -> tofSensor.getFrontDistance() > Constants.YOU_SHALL_NOT_PASS),
            
            // Stop briefly
            drivetrain.runOnce(() -> drivetrain.setControl(strafeRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0))),
                
            // Log detection of opening
            Commands.runOnce(() -> {
                System.out.println("Opening detected at distance: " + tofSensor.getFrontDistance());
            }),
            
            // Strafe left by the appropriate amount based on target branch
            Commands.either(
                // Left branch offset
                createStrafeLeftCommand(drivetrain, LEFT_BRANCH_OFFSET),
                // Right branch offset
                createStrafeLeftCommand(drivetrain, RIGHT_BRANCH_OFFSET),
                // Condition to determine which branch to align with
                () -> isLeftBranch
            ),
            
            // Log completion
            Commands.runOnce(() -> System.out.println("Reef alignment complete"))
        );
        
        addRequirements(drivetrain);
    }
    
    /**
     * Creates a command to strafe left by a specified distance.
     * Uses a time-based approach similar to DriveForwardCommand.
     * 
     * @param drivetrain The swerve drivetrain
     * @param distanceMeters Distance to strafe left in meters
     * @return Command to execute the strafe
     */
    private Command createStrafeLeftCommand(CommandSwerveDrivetrain drivetrain, double distanceMeters) {
        // Calculate time needed based on strafe speed (similar to DriveForwardCommand)
        // This assumes the STRAFE_SPEED is in percentage of max speed
        double maxSpeed = 4.73; // Based on TunerConstants.kSpeedAt12Volts in your code
        double actualSpeed = STRAFE_SPEED * maxSpeed;
        double timeToStrafe = Math.abs(distanceMeters / actualSpeed);
        
        return Commands.sequence(
            // Log starting the strafe
            Commands.runOnce(() -> System.out.println("Strafing left " + distanceMeters + "m")),
            
            // Strafe left for calculated time
            drivetrain.run(() -> {
                drivetrain.setControl(strafeRequest
                    .withVelocityX(0)
                    .withVelocityY(STRAFE_SPEED) // Positive Y is left in robot-centric frame
                    .withRotationalRate(0));
            }).withTimeout(timeToStrafe),
            
            // Stop after strafing
            drivetrain.runOnce(() -> drivetrain.setControl(strafeRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)))
        );
    }
    
    /**
     * Factory method to create a command for left branch alignment.
     */
    public static Command alignToLeftBranch(CommandSwerveDrivetrain drivetrain, FrontTOFSubsystem tofSensor) {
        return new AlignToReefWithEdgeDetection(drivetrain, tofSensor, true);
    }
    
    /**
     * Factory method to create a command for right branch alignment.
     */
    public static Command alignToRightBranch(CommandSwerveDrivetrain drivetrain, FrontTOFSubsystem tofSensor) {
        return new AlignToReefWithEdgeDetection(drivetrain, tofSensor, false);
    }
}