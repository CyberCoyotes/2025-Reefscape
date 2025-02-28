package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class CommandGroups {

    public Command moveToHomeGroup(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move wrist to safe travel position if not already
                wristCommands.setL2(),
                // Move the elevator to home position
                elevatorCommands.setHomeNoCheck(),
                // TODO Add as a check that elevator has completed its command AND reached its target
                // Removed a wait command here <--. Hopefully this will allow wrist to move to stow

                // Stow the elevator
                wristCommands.setStowed()

        ).withName("MoveToHomeSequence");
    }

    public Command moveToL1Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // wristCommands.setL1(), // First move wrist to L2
                // elevatorCommands.moveToL2NoCheck() // Then move elevator (this already checks wrist.inSafePosition())
        ).withName("MoveToL2Sequence");
    }

    public Command moveToL2Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                wristCommands.setL2(), // First move wrist to L2
                elevatorCommands.setL2NoCheck() // Then move elevator (this already checks wrist.inSafePosition())
        ).withName("MoveToL2Sequence");
    }

    public Command moveToL3Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Set the wrist to L3, i.e. a safe position
                wristCommands.setL3(),
                // Move the elevator to L3
                elevatorCommands.setL3NoCheck()
        ).withName("MoveToL3Sequence");
    }

    public Command moveToL4Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Set the wrist to L2, i.e. a safe position
                wristCommands.setL2(),
                // Move the elevator to L4
                elevatorCommands.setL4NoCheck(),
                // TODO check this position
                wristCommands.setL4()
        ).withName("MoveToL4Sequence");
    }

    public Command moveToPickAlgae2Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move to safe wrist position
                wristCommands.setL2(),
                // Move elevator up and underneath the algae
                elevatorCommands.setL2NoCheck(),
                // Move elevator up && run coral?
                elevatorCommands.setAlgae2NoCheck(),
                // Move elevator up to final picking position
                wristCommands.pickAlgae()
        ).withName("MoveToAlgaeSequence");
    }

    public Command moveToPickAlgae3Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move to safe wrist position
                wristCommands.setL3(),
                // Move elevator up and underneath the algae
                elevatorCommands.setAlgae3NoCheck(),

                // Move elevator up && run coral?

                // Move elevator up to final picking position
                wristCommands.pickAlgae()
        ).withName("MoveToAlgaeSequence");
    }

    public Command moveToScoreAlgaeGroup(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
        return Commands.sequence(
                // Move the wrist to a down and "out" position
                wristCommands.scoreAlgae(),
                
                // Move the elevator to the algae scoring position;
                /*
                 * TODO Position needs to be verified
                 * Previously this was set to L2, based on observation at practice field it should be less than L2
                 */
                elevatorCommands.setScoreAlgaeNoCheck()

        ).withName("ScoringAlgaeSequence");
    }

}
