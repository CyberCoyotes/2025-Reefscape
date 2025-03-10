package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimbConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

@SuppressWarnings("unused")

public class ClimberCommands {
    private final ClimberSubsystem climber;
    private final WristSubsystem wrist;

    public ClimberCommands(ClimberSubsystem climber, WristSubsystem wrist) {
        this.climber = climber;
        this.wrist = wrist;
    }
    public Command incrementUp() {
        return Commands.run(
            () -> climber.climbUp())

            // .until()(() -> climber.getWristPosition() > 0.5)
        
        // When command ends, stop the motor by setting 0 V:
        .finallyDo((boolean interrupted) -> climber.stopClimb())
        .withName("ClimbUp");
    }

    /**
     * Returns a Command that drives the motor at -6 V until canceled or interrupted.
     */
    public Command incrementDown() {
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
    public Command stopClimb() {
        return Commands.runOnce(
            () -> climber.stopClimb()
        ).withName("StopClimb");
    }

    
    // Command to set servo to specific angle
    public Command setServoAngle() {
        return Commands.runOnce(
            () -> climber.setServoAngle(ClimbConstants.SERVO_ANGLE)
        ).withName("SetServoAngle");
    }
    
    // Servo commands
    public Command toggleServo() {
        return climber.runOnce(() -> climber.rotateServoRange())
            .withName("ToggleServo");

    }

}
