package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandGroups {
 
// TODO Check if this is the correct
public Command moveToL2Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        wristCommands.setL2(),           // First move wrist to L2
        elevatorCommands.moveToL2()      // Then move elevator (this already checks wrist.inSafePosition())
    ).withName("MoveToL2Sequence");
}

// TODO Check if this is the correct
public Command moveToL3Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        wristCommands.setL2(),           // First move wrist to L2
        elevatorCommands.moveToL3()      // Then move elevator (this already checks wrist.inSafePosition())
    ).withName("MoveToL3Sequence");
}

// TODO Check if this is the correct
public Command moveToL4Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        wristCommands.setL2(),           // First move wrist to L2
        elevatorCommands.moveToL4(),
        wristCommands.setL4()      // Then move elevator (this already checks wrist.inSafePosition())
    ).withName("MoveToL4Sequence");
}

// TODO Check if this is the correct
public Command moveToAlgae2Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        wristCommands.setL2(),           // First move wrist to L2
        elevatorCommands.moveToPickAlgae2Raw(),
        wristCommands.setAlgae()      // Then move elevator (this already checks wrist.inSafePosition())
    ).withName("MoveToAlgaeSequence");
}

// TODO Check if this is the correct
public Command moveToAlgae3Group(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        wristCommands.setL2(),           // First move wrist to L2
        elevatorCommands.moveToPickAlgae3Raw(),
        wristCommands.setAlgae()      // Then move elevator (this already checks wrist.inSafePosition())
    ).withName("MoveToAlgaeSequence");
}

// TODO Check if this is the correct
public Command moveToHomeGroup(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        wristCommands.setL2(),           // First move wrist to L2
        elevatorCommands.moveToHome()      // Then move elevator (this already checks wrist.inSafePosition())
    ).withName("MoveToHomeSequence");
}

// TODO Check if this is the correct
public Command moveToScoreAlgae(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        wristCommands.setAlgae(),           // First move wrist to L2
        elevatorCommands.moveToL2()      // Then move elevator (this already checks wrist.inSafePosition())
    ).withName("MoveToHomeSequence");
}

// Example 2 template - awaiting your specific requirements
public Command customSequence(WristCommands wristCommands, ElevatorCommands elevatorCommands) {
    return Commands.sequence(
        // Add your desired command sequence here
    ).withName("CustomSequence");
}
}
