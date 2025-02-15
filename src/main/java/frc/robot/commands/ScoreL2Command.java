package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.wrist.WristSubsystem;

public final class ScoreL2Command {
    private ScoreL2Command() {}

    /**
     * Returns a command sequence:
     * 1) Wrist --> SAFE
     * 2) Elevator --> L2
     * 3) Wrist --> L2
     */
    public static Command createScoreL2Command(WristSubsystem wrist,
                                               ElevatorSubsystem elevator,
                                               ElevatorCommands elevatorCommands) {
        return Commands.sequence(
            // Step 1: Wrist to "safe"
            WristCommands.setSafePose(wrist),

            // Step 2: Elevator to L2
            elevatorCommands.moveToL2(),

            // Step 3: Wrist to L2
            WristCommands.setL2(wrist)
        ).withName("ScoreL2Sequence");
    }
}
