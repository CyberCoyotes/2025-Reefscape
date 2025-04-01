package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrontTOFSubsystem;
import frc.robot.subsystems.ReefTOFSubsystem;
import frc.robot.subsystems.endEffector.EffectorConstants;
import frc.robot.subsystems.endEffector.EffectorSubsystem;

@SuppressWarnings("unused") // Suppress warnings for unused imports and methods

public class CommandGroups {

    private final WristCommands wristCommands;
    private final ElevatorCommands elevatorCommands;
    private final EndEffectorCommands effectorCommands;
    private final EffectorSubsystem effector;
    private final FrontTOFSubsystem frontToF;
    private final CommandSwerveDrivetrain drivetrain;

    /**
     * Creates a new CommandGroups instance with the necessary command factories.
     * @param wristCommands    The wrist command factory
     * @param elevatorCommands The elevator command factory
     * @param effectorCommands The end effector command factory
     */
    public CommandGroups(
            WristCommands wristCommands,
            ElevatorCommands elevatorCommands,
            EffectorSubsystem effector,
            EndEffectorCommands effectorCommands,
            FrontTOFSubsystem frontToF,
            CommandSwerveDrivetrain drivetrain) {
        this.wristCommands = wristCommands;
        this.elevatorCommands = elevatorCommands;
        this.effector = effector;
        this.effectorCommands = effectorCommands;
        this.frontToF = frontToF;
        this.drivetrain = drivetrain;
    }

    /*******************************
     ** END GAME 
     ******************************/

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
                // Move the elevator to home position
                elevatorCommands.setHome()).withName("MoveToHomeSequence");
    }

    public Command moveToL1(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // First move wrist to L2 - this must complete
                wristCommands.setL1(), // Add timeout to prevent hanging
                // Then move elevator - this will wait for the wrist
                elevatorCommands.setL1()).withName("MoveToL1Sequence");
    }

    public Command moveToL2(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // First move wrist to L2 - this must complete
                wristCommands.setL2(), // Add timeout to prevent hanging
                // Then move elevator - this will wait for the wrist
                elevatorCommands.setL2()).withName("MoveToL2Sequence");
    }

    public Command moveToL3(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Set the wrist to L3, i.e. a safe position
                wristCommands.setL3(), // Add timeout to prevent hanging
                // Move the elevator to L3
                elevatorCommands.setL3()).withName("MoveToL3Sequence");
    }

    public Command moveToL4(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Set the wrist to L2, i.e. a safe position
                wristCommands.setL2(), // Add timeout to prevent hanging
                // Move the elevator to L4
                elevatorCommands.setL4(),
                // Set wrist to L4
                wristCommands.setL4()).withName("MoveToL4Sequence");
    }

    public Command moveToPickAlgae2(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move to safe wrist position first
                wristCommands.setL2(), // Add timeout to prevent hanging
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
                // Then move the elevator to the algae scoring position
                elevatorCommands.setScoreAlgae());
    }

    public Command moveToTravel(/*WristCommands wristCommands, ElevatorCommands elevatorCommands*/) {
        return Commands.sequence(
                // Add a small delay to give robot time to clear Reef Branches
                // new WaitCommand(0.3),

                // Move wrist to travel position
                wristCommands.setTravel(),

                // Move the elevator to travel position, same as CORAL INTAKE currently
                elevatorCommands.setTravel()
                
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

    /**
 * Creates a command that checks if the robot is within loading range before
 * attempting to intake coral. If the robot is in the correct distance range,
 * it will run the intake sequence. Otherwise, it will log a message and wait.
 * 
 * @return A command that safely intakes coral when properly positioned
 */
/* DEPRECATED use autoIntakeCoral
public Command intakeCoralMinimum(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    // Create a loading range checker instance
    LoadingRangeChecker rangeChecker = new LoadingRangeChecker(frontToF);
    
    // Create the command to execute when in range
    Command intakeSequence = Commands.sequence(
    
        wristCommands.setL2(),
        
        // Move elevator to intake position
        elevatorCommands.setIntakeCoral(),
        
        // Move wrist to intake position
        wristCommands.setIntakeCoral(),
        
        // Activate the end effector for intake
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
    */

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

                // Run effector command until coral leaves the end effector
                effectorCommands.autoScoreCoral(),
                
                wristCommands.setL2()
                ).withName("AutoScoreL2Sequence");


    }

    public Command autoScoreL3() {
        return Commands.sequence(
                // Move wrist to L3 position for scoring
                wristCommands.setL3(),

                // Move elevator to L2 height
                elevatorCommands.setL3(),

                // Run effector command until coral leaves the end effector
                effectorCommands.autoScoreCoral(), 

                // Move wrist to L3 position
                wristCommands.setL3(),
                
                elevatorCommands.setL2()
                ).withName("AutoScoreL3Sequence");

    }

    public Command autoScoreL4() {
        return Commands.sequence(
                // Set the wrist to L2
                wristCommands.setL2(),
                
                // Move the elevator to L4
                elevatorCommands.setL4(),
                
                // Set wrist to L4
                wristCommands.setL4(),

                // Short delay to stabilize
                Commands.waitSeconds(0.05),
                
                // Score the coral with timing appropriate for autonomous
                effectorCommands.autoScoreCoral(),
                
                wristCommands.setL2(),
                
                elevatorCommands.setL2()
                ).withName("AutoScoreL4Sequence");
    }

    public Command autoRoadRunnerL4() {
        return Commands.sequence(

                // moveToTravel(wristCommands, elevatorCommands),
                wristCommands.setL3(),
                // Test to make sure this does not hit reef going up - appears to hit the reef!
                elevatorCommands.setL4(),
                // Set wrist to L4
                wristCommands.setL4(),
                // Short delay to stabilize
                Commands.waitSeconds(0.05),
                effectorCommands.autoScoreCoral(),
                wristCommands.setL3(),
                elevatorCommands.setL2()
                // moveToTravel(wristCommands, elevatorCommands)
                    .withName("scoreL4Sequence"));
    }

    public Command autoBeepBeepL4() {
        return Commands.sequence(
                moveToTravel(),

                // Test to make sure this does not hit reef going up - Appears to hit the reef!
                elevatorCommands.setL4(),

                // Set wrist to L4
                wristCommands.setL4(),

                // Short delay to stabilize
                Commands.waitSeconds(0.05),

                // Score the coral with timing appropriate for autonomous
                effectorCommands.autoScoreCoral(), 

                moveToTravel()
                    .withName("scoreL4Sequence"));
    }

    public Command autoIntakeCoral() {
        return Commands.sequence(
            // Move wrist to L2 position
            // wristCommands.setIntakeCoral(),
            
            // Move elevator to intake position
            elevatorCommands.setIntakeCoral(),
            
            // Move wrist to intake position
            wristCommands.setIntakeCoral(),
            
            // Activate the end effector for intake
            effectorCommands.intakeCoral(),
            
            // Wait for coral detection or timeout
            // effectorCommands.waitForCoralLoadWithTimeout(20),
            
            // Once loaded or timed out, move to safe position
            Commands.parallel(
                wristCommands.setL3(),
                Commands.sequence(
                    Commands.waitSeconds(0.05), // Give wrist time to start moving
                    elevatorCommands.setL3() // Changed from L2
                )
            )
        ).withName("AutoIntakeCoralSequence");
    }

    public Command autoPreScore() {
        return Commands.sequence(
                wristCommands.setL3(),
                elevatorCommands.setTravel()
                .withName("AutoScoreL2Sequence"));
    }

    /**
     * Creates a command that stops the drivetrain until coral is loaded or timeout expires.
     * @param timeoutSeconds Maximum time to wait for coral loading
     * @return Command that completes when coral is loaded or timeout expires
     */
    public Command stopUntilCoralLoaded(double timeoutSeconds) {
        return Commands.race(
            // Stop drivetrain
            drivetrain.run(() -> {
                drivetrain.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(0)
                    .withVelocityY(0) 
                    .withRotationalRate(0));
            }),
            // Wait until coral loaded OR timeout expires (whichever comes first)
            Commands.race(
                Commands.waitUntil(() -> effector.isCoralLoaded()),
                Commands.waitSeconds(timeoutSeconds)
            )
        ).withName("StopUntilCoralLoaded");
    }

        /**
     * Creates a command that stops the drivetrain until coral is loaded or timeout expires.
     * @param timeoutSeconds Maximum time to wait for coral loading
     * @return Command that completes when coral is loaded or timeout expires
     */
    public Command stopUntilCoralReleased(double timeoutSeconds) {
        return Commands.race(
            // Stop drivetrain
            drivetrain.run(() -> {
                drivetrain.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(0)
                    .withVelocityY(0) 
                    .withRotationalRate(0));
            }),
            // Wait until coral NOT loaded OR timeout expires (whichever comes first)
            Commands.race(
                Commands.waitUntil(() -> !effector.isCoralLoaded()),
                Commands.waitSeconds(timeoutSeconds)
            )
        ).withName("StopUntilCoralReleased");
    }

} // End of CommandGroups class