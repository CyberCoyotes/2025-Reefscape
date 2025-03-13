package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.FrontTOFSubsystem;

public class CommandGroups {

    private final WristCommands wristCommands;
    private final ElevatorCommands elevatorCommands;
    private final EndEffectorCommands effectorCommands;
    private final FrontTOFSubsystem frontToF;

    /**
     * Creates a new CommandGroups instance with the necessary command factories.
     * 
     * @param wristCommands The wrist command factory
     * @param elevatorCommands The elevator command factory
     * @param effectorCommands The end effector command factory
     */
    public CommandGroups(
        WristCommands wristCommands, 
        ElevatorCommands elevatorCommands, 
        EndEffectorCommands effectorCommands,
        FrontTOFSubsystem frontToF
    ) {
        this.wristCommands = wristCommands;
        this.elevatorCommands = elevatorCommands;
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
            elevatorCommands.setHome()
        ).withName("MoveToHomeSequence");
    }

    public Command moveToL2(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // First move wrist to L2 - this must complete
            wristCommands.setL2(), // Add timeout to prevent hanging 
            // Add small delay to ensure wrist command has started
            // new WaitCommand(0.1),
            // Then move elevator - this will wait for the wrist
            elevatorCommands.setL2() 
        ).withName("MoveToL2Sequence");
    }

    public Command moveToL3(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Set the wrist to L3, i.e. a safe position
            wristCommands.setL3(), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            // new WaitCommand(0.1),
            // Move the elevator to L3
            elevatorCommands.setL3()
        ).withName("MoveToL3Sequence");
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
            wristCommands.setL4()
        ).withName("MoveToL4Sequence");
    }

    public Command moveToPickAlgae2(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Move to safe wrist position first
            wristCommands.setL2(), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            // new WaitCommand(0.1),
            // Move elevator up to L2 position
            elevatorCommands.setL2(),
            // Move elevator up to algae position
            elevatorCommands.setAlgae2(),
            // Move the wrist to the picking position
            wristCommands.pickAlgae()
        ).withName("MoveToAlgae2Sequence");
    }

    public Command moveToPickAlgae3(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Move to safe wrist position
            wristCommands.setL3(), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            // new WaitCommand(0.1),
            // Move elevator up to algae position
            elevatorCommands.setAlgae3(),
            // Move the wrist to the picking position
            wristCommands.pickAlgae()
        ).withName("MoveToAlgae3Sequence");
    }

    public Command moveToScoreAlgae(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // First move the wrist to the scoring position
            wristCommands.scoreAlgae(),
            // Add small delay to ensure wrist command has started
            // new WaitCommand(0.1),
            // Then move the elevator to the algae scoring position
            elevatorCommands.setScoreAlgae()
        ).withName("ScoringAlgaeSequence");
    }

    /**********************************
     * Intake Coral Commands
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
            effectorCommands.intakeCoral()
        ).withName("IntakeCoralSequence");
    }

    // FIXME: Test Implement this method
    public Command moveToIntakeCoral(WristCommands wristCommands, ElevatorCommands elevatorCommands, WristSubsystem wrist) {
        return Commands.sequence(

        // First check if wrist needs to be moved to safe position
        Commands.either(
            // If wrist is already at L2 or beyond, do nothing
            Commands.none(),
            // Otherwise, move wrist to L2 position
            wristCommands.setL2(),
            // This is the condition - use approximate comparison with tolerance
            () -> Math.abs(wrist.getPosition() - WristSubsystem.WristPositions.L2.getRotations()) < WristConstants.TOLERANCE
        ),
        
        // Move elevator to intake position
        elevatorCommands.setIntakeCoral(),

         /* TODO Check the ToF distance from the loading station as a safety check
         * If the ToF distance is between within a certain range (LOADING_RANGE 720 - 730 mm), continue to set wrist position
         * ELSE move the robot away from the station -or- just do nothing until condition is met
         */
        // Print the current ToF distance to the console
        // Commands.runOnce(() -> System.out.println("ToF Distance: " + frontToF.getDistance())),

        // Move wrist to intake position
        wristCommands.setIntakeCoral(),
        
        // Activate the intake end effector
        effectorCommands.intakeCoralWithSensor(),

        // TODO The wrist and elevator should stay here **until** the coral is detected
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
            
            // Add small delay to ensure wrist command has started
            // new WaitCommand(0.1),
            
            // Move elevator to L2 height
            elevatorCommands.setL2(),
            
            // Short delay to stabilize
            Commands.waitSeconds(0.2),
            
            // Score the coral with timing appropriate for autonomous
            effectorCommands.autoScoreCoral()
        ).withName("AutoScoreL2Sequence");
    }

    public Command autoScoreL2() {
        return Commands.sequence(
            // Move wrist to L2 position for scoring
            wristCommands.setL2(),
                    
            // Move elevator to L2 height
            elevatorCommands.setL2(),
            
            // Short delay to stabilize
            Commands.waitSeconds(0.2),
            
            // Score the coral with timing appropriate for autonomous
            effectorCommands.autoScoreCoral()
        ).withName("AutoScoreL2Sequence");
    }

    public Command autoScoreL4() {
        return Commands.sequence(
            // Set the wrist to L2, i.e. a safe position
            wristCommands.setL2(),//.withTimeout(1.5), // Add timeout to prevent hanging
            // Move the elevator to L4
            elevatorCommands.setL4(),//.withTimeout(2.0),
            // Set wrist to L4
            wristCommands.setL4(),
            // Short delay to stabilize
            Commands.waitSeconds(0.05), // Changed from 0.2 to 0.05
            // Score the coral with timing appropriate for autonomous
            effectorCommands.autoScoreCoral(),
            // Move wrist to safe travel position if not already
            wristCommands.setL2(),//.withTimeout(1.5), // Add timeout to prevent hanging
            // Move the elevator to home position
            elevatorCommands.setHome()
        ).withName("scoreL4Sequence");
    }

    public Command autoIntakeCoral(WristCommands wristCommands, ElevatorCommands elevatorCommands, WristSubsystem wrist) {
        return Commands.sequence(

            wristCommands.setL2(),
        // Move elevator to intake position
        elevatorCommands.setIntakeCoral(),
    
         /* TODO Check the ToF distance from the loading station as a safety check
         * If the ToF distance is between within a certain range (LOADING_RANGE 720 - 730 mm), continue to set wrist position
         * ELSE move the robot away from the station -or- just do nothing until condition is met
         */
        // Print the current ToF distance to the console
        // Commands.runOnce(() -> System.out.println("ToF Distance: " + frontToF.getDistance())),
    
        // Move wrist to intake position
        wristCommands.setIntakeCoral(),
        
        // Activate the intake end effector
        effectorCommands.intakeCoralWithSensor(),
    
        // TODO The wrist and elevator should stay here **until** the coral is detected
        // Activate the intake end effector and wait until coral is loaded
        // effectorCommands.intakeCoralWithSensor().until(() -> effectorCommands.isCoralLoaded()),

        // Move the wrist back to L2 position
        wristCommands.setL2(),
    
        // Move the elevator back to L2 position
        elevatorCommands.setL2()
    
    ).withName("IntakeCoralSequence");

    }


} // End of CommandGroups class