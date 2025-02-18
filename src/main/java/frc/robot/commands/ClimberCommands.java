package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.climber.ClimberVoltageSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

import java.util.function.DoubleSupplier;

public class ClimberCommands {
    private final ClimberVoltageSubsystem climber;
    private final WristSubsystem wrist;

    public ClimberCommands(ClimberVoltageSubsystem climber, WristSubsystem wrist) {
        this.climber = climber;
        this.wrist = wrist;
    }
    public Command climbUpCommand() {
        return Commands.run(
            () -> climber.climbUp())
        // TODO Consider adding a wrist set position command here
        
        // TODO Add a "max" climb position
        // .until()(() -> climber.getWristPosition() > 0.5)
        
        // When command ends, stop the motor by setting 0 V:
        .finallyDo((boolean interrupted) -> climber.stopClimb())
        .withName("ClimbUp");
    }

    /**
     * Returns a Command that drives the motor at -6 V until canceled or interrupted.
     */
    public Command climbDownCommand() {
        return Commands.run(
            () -> climber.climbDown())
        // When command ends, stop the motor by setting 0 V:
        .finallyDo((boolean interrupted) -> climber.stopClimb())
        .withName("ClimbDown");
    }

    /**
     * Returns a Command that immediately stops the climb motor (0 V).
     * This can be used in “instant” scenarios.
     */
    public Command stopClimbCommand() {
        return Commands.runOnce(
            () -> climber.stopClimb()
        ).withName("StopClimb");
    }

}
