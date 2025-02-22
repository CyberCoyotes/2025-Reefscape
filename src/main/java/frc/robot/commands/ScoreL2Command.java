package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
// import frc.robot.commands.WristCommands;
import frc.robot.subsystems.wrist.WristSubsystem;

public final class ScoreL2Command {
    private ScoreL2Command() {}

    /**
     * Returns a command sequence:
     * 1) Wrist --> SAFE
     * 2) Elevator --> L2
     * 3) Wait for a specified time
     * 4) Wrist --> L2
     * 5) Effector --> SCORE
     * 6) Elevator --> HOME
     */
    public static Command createScoreL2Command(WristSubsystem wrist,
                                               ElevatorSubsystem elevator,
                                               ElevatorCommands elevatorCommands,
                                               EffectorSubsystem endEffector) {
        return Commands.sequence(
            // Step 1: Wrist to "safe"
            WristCommands.setSafePose(wrist),

            // Step 2: Elevator to L2
            elevatorCommands.moveToL2(),

            // Step 3: Wait for a specified time (e.g., 0.5 second)
            new WaitCommand(0.5),

            // Step 4: Wrist to L2
            // WristCommands.setL2(wrist)

            // Step 5: Effector to score
            // new SetEndEffectorCommand(EffectorState.SCORE_CORAL, endEffector)

            // Step 6: Elevator to home
            elevatorCommands.moveToHome()
            
        ).withName("ScoreL2Sequence");

    }
}
