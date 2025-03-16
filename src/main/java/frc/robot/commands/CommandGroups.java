package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.FrontTOFSubsystem;
import frc.robot.subsystems.endEffector.EffectorConstants;
import frc.robot.subsystems.endEffector.EffectorSubsystem;

@SuppressWarnings("unused") // Suppress warnings for unused imports and methods

public class CommandGroups {

    private final WristCommands wristCommands;
    private final ElevatorCommands elevatorCommands;
    private final EndEffectorCommands effectorCommands;
    private final EffectorSubsystem effector;
    private final FrontTOFSubsystem frontToF;

    /**
     * Creates a new CommandGroups instance with the necessary command factories.
     * 
     * @param wristCommands    The wrist command factory
     * @param elevatorCommands The elevator command factory
     * @param effectorCommands The end effector command factory
     */
    public CommandGroups(
            WristCommands wristCommands,
            ElevatorCommands elevatorCommands,
            EffectorSubsystem effector,
            EndEffectorCommands effectorCommands,
            FrontTOFSubsystem frontToF) {
        this.wristCommands = wristCommands;
        this.elevatorCommands = elevatorCommands;
        this.effector = effector;
        this.effectorCommands = effectorCommands;
        this.frontToF = frontToF;
    }

    public Command releaseKickSetWrist(WristCommands wristCommands, ClimberCommands climbCommands) {
        return Commands.sequence(
                // Moves the wrist out of the way
                wristCommands.setL2(),
                // Releases the kickstand
                climbCommands.toggleServo())
                .withName("ReleaseKickSetWristSequence");
    }

    public Command moveToHome(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move wrist to safe travel position if not already
                wristCommands.setL2(), // Add timeout to prevent hanging
                // Add small delay to ensure wrist command has started
                // new WaitCommand(0.1),
                // Move the elevator to home position
                elevatorCommands.setHome()).withName("MoveToHomeSequence");
    }

    public Command moveToL2(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // First move wrist to L2 - this must complete
                wristCommands.setL2(), // Add timeout to prevent hanging
                // Add small delay to ensure wrist command has started
                // new WaitCommand(0.1),
                // Then move elevator - this will wait for the wrist
                elevatorCommands.setL2()).withName("MoveToL2Sequence");
    }

    public Command moveToL3(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Set the wrist to L3, i.e. a safe position
                wristCommands.setL3(), // Add timeout to prevent hanging
                // Add small delay to ensure wrist command has started
                // new WaitCommand(0.1),
                // Move the elevator to L3
                elevatorCommands.setL3()).withName("MoveToL3Sequence");
    }

    public Command moveToL4(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Set the wrist to L2, i.e. a safe position
                wristCommands.setL2(), // Add timeout to prevent hanging
                // Add small delay to ensure wrist command has started
                // new WaitCommand(0.1),
                // Move the elevator to L4
                elevatorCommands.setL4(),
                // Set wrist to L4
                wristCommands.setL4()).withName("MoveToL4Sequence");
    }

    public Command moveToPickAlgae2(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move to safe wrist position first
                wristCommands.setL2(), // Add timeout to prevent hanging
                // Move elevator up to L2 position
                // elevatorCommands.setL2(),
                // Move elevator up to algae position
                elevatorCommands.setAlgae2(),
                // Move the wrist to the picking position
                wristCommands.pickAlgae()).withName("MoveToAlgae2Sequence");
    }

    public Command moveToPickAlgae3(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move to safe wrist position
                wristCommands.setL3(), // Add timeout to prevent hanging
                // Move elevator up to algae position
                elevatorCommands.setAlgae3(),
                // Move the wrist to the picking position
                wristCommands.pickAlgae()).withName("MoveToAlgae3Sequence");
    }

    public Command moveToScoreAlgae(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // First move the wrist to the scoring position
                wristCommands.scoreAlgae(),
                // Add small delay to ensure wrist command has started
                // new WaitCommand(0.1),
                // Then move the elevator to the algae scoring position
                elevatorCommands.setScoreAlgae()).withName("ScoringAlgaeSequence");
    }

    public Command moveToTravel(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Add a small delay to give robot time to clear Reef Branches
                new WaitCommand(0.3),

                // Move wrist to travel position
                wristCommands.setTravelPose(),

                // Move the elevator to travel position, same as CORAL INTAKE currently
                elevatorCommands.setTravelPose()
                
                ).withName("TravelPoseBetweenStations");
    }

    /**********************************
     * Intake Coral Commands
     * 
     * @return
     */
    public Command intakeBasicCoral(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move wrist to L2 position
                wristCommands.setL2(),

                // Move elevator to intake position
                elevatorCommands.setIntakeCoral(),

                // After elevator is at positioned, move wrist to intake position
                wristCommands.setIntakeCoral(),

                // Finally activate the intake
                effectorCommands.intakeCoral()).withName("IntakeCoralSequence");
    }

    // Test Implement this method
    public Command moveToIntakeCoral(WristCommands wristCommands, ElevatorCommands elevatorCommands,
            WristSubsystem wrist) {
        return Commands.sequence(

                // First check if wrist needs to be moved to safe position
                Commands.either(
                        // If wrist is already at L2 or beyond, do nothing
                        Commands.none(),
                        // Otherwise, move wrist to L2 position
                        wristCommands.setL2(),
                        // This is the condition - use approximate comparison with tolerance
                        () -> Math.abs(wrist.getPosition()
                                - WristSubsystem.WristPositions.L2.getRotations()) < WristConstants.TOLERANCE),

                // Move elevator to intake position
                elevatorCommands.setIntakeCoral(),

                /*
                 * Check the ToF distance from the loading station as a safety check
                 * If the ToF distance is between within a certain range (LOADING_RANGE 720 -
                 * 730 mm), continue to set wrist position
                 * ELSE move the robot away from the station -or- just do nothing until
                 * condition is met
                 */
                // Print the current ToF distance to the console
                // Commands.runOnce(() -> System.out.println("ToF Distance: " +
                // frontToF.getDistance())),

                // Move wrist to intake position
                wristCommands.setIntakeCoral(),

                // Activate the intake end effector
                effectorCommands.intakeCoral(),

                // The wrist and elevator should stay here **until** the coral is detected
                // Move the wrist back to L2 position
                wristCommands.setL2(),

                // Move the elevator back to L2 position
                elevatorCommands.setL2()

        ).withName("IntakeCoralSequence");
    }

    /*******************************
     * Auto Command Groups
     ******************************/
    public Command autoScoreL1() {
        return Commands.sequence(
                // Move wrist to L2 position for scoring
                wristCommands.setL2(),

                // Move elevator to L2 height
                elevatorCommands.setL2(),

                // Score the coral with timing appropriate for autonomous
                effectorCommands.scoreCoralWithTimeout()).withName("AutoScoreL2Sequence");
    }

    public Command autoScoreL2() {
        return Commands.sequence(
                // Move wrist to L2 position for scoring
                wristCommands.setL2(),

                // Move elevator to L2 height
                elevatorCommands.setL2(),

                // Short delay to stabilize
                // Commands.waitSeconds(0.2),

                // Score the coral with timing appropriate for autonomous
                effectorCommands.scoreCoralWithTimeout()).withName("AutoScoreL2Sequence");
    }

    public Command autoScoreL4() {
        return Commands.sequence(
                // Set the wrist to L2, i.e. a safe position
                wristCommands.setL2(),
                // Move the elevator to L4
                elevatorCommands.setL4(),
                // Set wrist to L4
                wristCommands.setL4(),
                // Short delay to stabilize
                Commands.waitSeconds(0.05),
                // Score the coral with timing appropriate for autonomous
                effectorCommands.scoreCoralWithTimeout(),
                // Move wrist to safe travel position if not already
                wristCommands.setL2(), // .withTimeout(1.5), // Add timeout to prevent hanging
                // Move the elevator to home position
                elevatorCommands.setHome()).withName("scoreL4Sequence");
    }

    // Untestest version
    public Command autoIntakeCoral(WristCommands wristCommands, ElevatorCommands elevatorCommands,
            WristSubsystem wrist) {
 
                return Commands.sequence(
        
                        // Move wrist to L2 position
                        wristCommands.setL2(),
        
                        // Move elevator to intake position
                        elevatorCommands.setIntakeCoral(),
        
                        // Move wrist to intake position
                        wristCommands.setIntakeCoral(),
        
                        // Activate the intake end effector
                        effectorCommands.intakeCoral(),

                // Move the wrist back to L2 position
                wristCommands.setL2(),

                // Move the elevator back to L2 position
                elevatorCommands.setL2()

        ).withName("IntakeCoralSequence");

    }

    /* // TODO Test this advanced version on autoIntakeCoral
    public Command autoIntakeCoral(WristCommands wristCommands, ElevatorCommands elevatorCommands, 
                              WristSubsystem wrist) {
    
    // Create loading range checker
    LoadingRangeChecker rangeChecker = new LoadingRangeChecker(frontToF);
    
    // First create the base sequence without range checking
    Command baseIntakeSequence = Commands.sequence(
        // Move wrist to L2 position
        wristCommands.setL2(),
        
        // Move elevator to intake position
        elevatorCommands.setIntakeCoral(),
        
        // Log the current distance
        Commands.runOnce(() -> System.out.println("Auto Intake Distance: " + frontToF.getFrontDistance() + "mm")),
        
        // Move wrist to intake position
        wristCommands.setIntakeCoral(),
        
        // Activate the intake end effector
        effectorCommands.intakeCoral(),
        
        // Move the wrist back to L2 position
        wristCommands.setL2(),
        
        // Move the elevator back to L2 position
        elevatorCommands.setL2()
    ).withName("AutoIntakeCoralSequence");
    
    // For autonomous, we want to timeout the waiting so the routine doesn't get stuck
    Command rangeCheckWithTimeout = Commands.race(
        // Wait until in range or timeout after 2 seconds
        Commands.waitUntil(rangeChecker::isInLoadingRange).withTimeout(2.0),
        
        // While waiting, periodically log the distance
        Commands.repeatingSequence(
            Commands.runOnce(() -> System.out.println("Waiting for loading range. Current: " + 
                                                     frontToF.getFrontDistance() + "mm")),
            Commands.waitSeconds(0.5)
        )
    );
    
    // Complete command with range checking for autonomous
    return Commands.sequence(
        // Start by moving into position
        wristCommands.setL2(),
        elevatorCommands.setIntakeCoral(),
        
        // Check if in range (with timeout to prevent blocking auto)
        rangeCheckWithTimeout,
        
        // Then execute the rest regardless (since we're in autonomous)
        baseIntakeSequence
    );
}
*/



/* Deprecated 
    // Version from Choreo branch
    public Command autoIntakeCoralChoreo() {
        return Commands.sequence(
                // Move wrist to L2 position
                wristCommands.setL2(),

                // Move elevator to intake position
                elevatorCommands.setIntakeCoral(),

                // After elevator is at positioned, move wrist to intake position
                wristCommands.setIntakeCoral(),

                // Finally activate the intake
                effectorCommands.intakeCoral(),

                Commands.waitSeconds(0.2),

                wristCommands.setL2(),

                elevatorCommands.setHome()).withName("IntakeCoralSequence");
    }
    */

    // Add this new method to CommandGroups.java
/**
 * Creates a command that checks if the robot is within loading range before
 * attempting to intake coral. If the robot is in the correct distance range,
 * it will run the intake sequence. Otherwise, it will log a message and wait.
 * 
 * @return A command that safely intakes coral when properly positioned
 */
public Command intakeCoralMinimum() {
        // Create a loading range checker instance
        LoadingRangeChecker rangeChecker = new LoadingRangeChecker(frontToF);
        
        // Create the command to execute when in range
        Command intakeSequence = Commands.sequence(
        
            wristCommands.setL2(),
                // This is the condition - use approximate comparison with tolerance
            
            // Move elevator to intake position
            elevatorCommands.setIntakeCoral(),
            
            // Move wrist to intake position
            wristCommands.setIntakeCoral(),
            
            // Activate the intake end effector
            effectorCommands.intakeCoral(),
            
            // After intaking, move the wrist back to L2 position
            wristCommands.setL2(),
            
            // Move the elevator back to L2 position
            elevatorCommands.setL2()
        ).withName("IntakeCoralSequence");
        
        // Create feedback command for when not in range
        Command outOfRangeFeedback = Commands.sequence(
            Commands.runOnce(() -> {
                double currentDistance = frontToF.getFrontDistance();
                System.out.println("Cannot intake coral: Robot not in loading position.");
                System.out.println("Current distance: " + currentDistance + " mm");
                System.out.println("Required range: " + LoadingRangeChecker.LOADING_RANGE_MIN + 
                                   " to " + LoadingRangeChecker.LOADING_RANGE_MAX + " mm");
            }),
            Commands.waitSeconds(1.0) // Short delay to prevent message spam
        ).withName("OutOfLoadingRange");
        
        // Return conditional command that checks range first
        return rangeChecker.whenInLoadingRange(intakeSequence, outOfRangeFeedback);
    }

} // End of CommandGroups class