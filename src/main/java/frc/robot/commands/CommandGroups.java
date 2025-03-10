package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class CommandGroups {

    private final WristCommands wristCommands;
    private final ElevatorCommands elevatorCommands;
    private final EndEffectorCommands effectorCommands;

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
        EndEffectorCommands effectorCommands
    ) {
        this.wristCommands = wristCommands;
        this.elevatorCommands = elevatorCommands;
        this.effectorCommands = effectorCommands;
    }

    public Command releaseKickSetWrist(WristCommands wristCommands, ClimberCommands climbCommands) {
        return Commands.sequence(
            // Moves the wrist out of the way
            wristCommands.setL2(),
            // Releases the kickstand
            climbCommands.toggleServo())
        .withName("ReleaseKickSetWristSequence");
    }

    public Command moveToHomeGroup(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Move wrist to safe travel position if not already
            wristCommands.setL2().withTimeout(1.5), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            // Move the elevator to home position
            elevatorCommands.setHomeNoCheck()
        ).withName("MoveToHomeSequence");
    }

    public Command moveToL2Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // First move wrist to L2 - this must complete
            wristCommands.setL2().withTimeout(1.5), // Add timeout to prevent hanging 
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            // Then move elevator - this will wait for the wrist
            elevatorCommands.setL2NoCheck() 
        ).withName("MoveToL2Sequence");
    }

    public Command moveToL3Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Set the wrist to L3, i.e. a safe position
            wristCommands.setL3().withTimeout(1.5), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            // Move the elevator to L3
            elevatorCommands.setL3NoCheck()
        ).withName("MoveToL3Sequence");
    }

    public Command moveToL4Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Set the wrist to L2, i.e. a safe position
            wristCommands.setL2().withTimeout(1.5), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            // Move the elevator to L4
            elevatorCommands.setL4NoCheck().withTimeout(2.0),
            // Set wrist to L4
            wristCommands.setL4()
        ).withName("MoveToL4Sequence");
    }

    public Command moveToPickAlgae2Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Move to safe wrist position first
            wristCommands.setL2().withTimeout(1.5), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            // Move elevator up to L2 position
            elevatorCommands.setL2NoCheck().withTimeout(1.5),
            // Move elevator up to algae position
            elevatorCommands.setAlgae2NoCheck().withTimeout(1.5),
            // Move the wrist to the picking position
            wristCommands.pickAlgae()
        ).withName("MoveToAlgae2Sequence");
    }

    public Command moveToPickAlgae3Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Move to safe wrist position
            wristCommands.setL3().withTimeout(1.5), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            // Move elevator up to algae position
            elevatorCommands.setAlgae3NoCheck().withTimeout(2.0),
            // Move the wrist to the picking position
            wristCommands.pickAlgae()
        ).withName("MoveToAlgae3Sequence");
    }

    public Command moveToScoreAlgaeGroup(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // First move the wrist to the scoring position
            wristCommands.scoreAlgae().withTimeout(1.5), // Add timeout to prevent hanging
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            // Then move the elevator to the algae scoring position
            elevatorCommands.setScoreAlgaeNoCheck().withTimeout(1.5)
        ).withName("ScoringAlgaeSequence");
    }

    // FIXME: Test Implement this method
public Command intakeCoralGroup(WristCommands wristCommands, ElevatorCommands elevatorCommands, WristSubsystem wrist) {
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
        
        // NOT WORKING AS EXPECTED -Once wrist is in safe position, move elevator
        // Commands.waitUntil(() -> 
        //     Math.abs(wrist.getPosition() - WristSubsystem.WristPositions.L2.getRotations()) < WristConstants.TOLERANCE
        // ),
        elevatorCommands.setIntakeCoral(),
        
        // After elevator is positioned, move wrist to intake position
        wristCommands.setIntakeCoral(),
        
        // Finally activate the intake
        effectorCommands.intakeCoralWithSensor()
    ).withName("IntakeCoralSequence");
}

    /*******************************
     * Auto Command Groups
     ******************************/

    public Command autoScoreL2() {
        return Commands.sequence(
            // Move wrist to L2 position for scoring
            wristCommands.setL2().withTimeout(1.0),
            
            // Add small delay to ensure wrist command has started
            new WaitCommand(0.1),
            
            // Move elevator to L2 height
            elevatorCommands.setL2NoCheck().withTimeout(1.0),
            
            // Short delay to stabilize
            Commands.waitSeconds(0.2),
            
            // Score the coral with timing appropriate for autonomous
            effectorCommands.scoreCoralAuto()
        ).withName("AutoScoreL2Sequence");
    }
}