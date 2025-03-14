package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.endEffector.EffectorConstants;
import frc.robot.subsystems.endEffector.EffectorSubsystem;

/**
 * Command factory class for the EndEffector subsystem.
 * This class contains methods that create and return commands for the end effector.
 */
public class EndEffectorCommands {
    private final EffectorSubsystem effector;

    /**
     * Constructs a new EndEffectorCommands with the specified subsystem.
     * 
     * @param effector The end effector subsystem
     */
    public EndEffectorCommands(EffectorSubsystem effector) {
        this.effector = effector;
    }

    /**
     * Creates a basic command to intake coral at the default speed.
     * This command does not check sensors and runs until canceled.
     * 
     * @return A command to run the intake
     */
    public Command intakeCoralBasic() {
        return effector.run(() -> 
            effector.setEffectorOutput(EffectorConstants.INTAKE_CORAL)
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("IntakeCoralBasic");
    }

    /**
     * Creates a command to intake coral without sensor feedback.
     * The motor will run until the command is canceled.
     * 
     * @return A command to intake coral
     */
    public Command intakeCoralNoSensor() {
        return new RunCommand(() -> 
            effector.setEffectorOutput(EffectorConstants.INTAKE_CORAL),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("IntakeCoralNoSensor");
    }

    /**
     * Creates a command that runs the intake until a coral is detected by the sensor.
     * When a coral is detected, the motor will automatically stop.
     * 
     * @return A command to intake coral with sensor feedback
     */
    public Command intakeCoralWithSensor() {
        return new RunCommand(() -> {
            // Check coral sensor
            if (effector.isCoralLoaded()) {
                effector.stopMotor();
            } else {
                // No coral detected, run the intake as normal
                effector.setEffectorOutput(EffectorConstants.INTAKE_CORAL);
            }
        }, effector)
        .finallyDo((interrupted) -> effector.stopMotor())
        .withName("IntakeCoralWithSensor");
    }

    // Write a command or helper that returns the status of isCoralLoaded
    // and use it in the intakeCoralWithSensor command to determine if the motor should stop
    // or continue running. This will help in making the command more efficient and responsive.
    // The isCoralLoaded method should be implemented in the EffectorSubsystem class.
    // This method will check the status of the coral sensor and return true if a coral is detected.
    // The command should be structured to run the motor until a coral is detected, at which point
    // the motor will stop automatically. This will ensure that the intake process is efficient
    // and responsive to the presence of coral, reducing the risk of jamming or overloading the system.

    // https://claude.ai/chat/29b91c46-0b27-4294-a3a5-3358d8bfeb98

        /**
     * Creates a command that runs the intake until a coral is detected by the sensor.
     * When a coral is detected, the motor will automatically stop.
     * 
     * @return A command to intake coral with sensor feedback
     */
    public Command intakeCoral() {
return new FunctionalCommand(
        // init
        () -> {},
        // execute
        () -> {
            if (effector.isCoralLoaded()) {
                effector.stopMotor();
            } else {
                effector.setEffectorOutput(EffectorConstants.INTAKE_CORAL);
            }
        },
        // end
        (interrupted) -> effector.stopMotor(),
        // isFinished
        () -> effector.isCoralLoaded(),
        effector
    ).withName("IntakeCoralWithSensor");
    }

    /**
     * Creates a command to slowly reverse the coral out of the mechanism.
     * 
     * @return A command to reverse the coral
     */
    public Command reverseCoralNoSensor() {
        return new RunCommand(() -> 
            effector.setEffectorOutput(-EffectorConstants.INTAKE_CORAL * 0.55),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("ReverseCoralNoSensor");
    }

    /**
     * Creates a command to score/eject coral at the standard speed.
     * 
     * @return A command to score coral
     */
    public Command scoreCoral() {
        return new RunCommand(() -> 
            effector.setEffectorOutput(EffectorConstants.SCORE_CORAL),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("ScoreCoral");
    }
    
    /**
     * Creates a command to score/eject coral with a specified timeout.
     * Useful for autonomous sequences where precise timing is needed.
     * 
     * @param timeoutSeconds The timeout in seconds
     * @return A command to score coral that times out after specified duration
     */
    public Command scoreCoralWithTimeout(double timeoutSeconds) {
        return new RunCommand(() -> 
            effector.setEffectorOutput(EffectorConstants.SCORE_CORAL),
            effector
        ).withTimeout(timeoutSeconds)
         .finallyDo((interrupted) -> effector.stopMotor())
         .withName("ScoreCoral(" + timeoutSeconds + "s)");
    }
    
    /**
     * Creates a command to score/eject coral with a default timeout of 0.75 seconds.
     * This is particularly useful for autonomous routines.
     * 
     * @return A command to score coral that times out after 0.75 seconds
     */
    public Command autoScoreCoral() {
        return scoreCoralWithTimeout(0.75);
    }

    /**
     * Creates a command to score/eject coral at a slower speed.
     * 
     * @return A command to score coral slowly
     */
    public Command slowCoral() {
        return new RunCommand(() -> 
            effector.setEffectorOutput(EffectorConstants.lSCORE_CORAL),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("SlowCoral");
    }

    /**
     * Creates a command to intake algae.
     * 
     * @return A command to intake algae
     */
    public Command intakeAlgae() {
        return new RunCommand(() -> 
            effector.setEffectorOutput(EffectorConstants.INTAKE_ALGAE),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("IntakeAlgae");
    }

    /**
     * Creates a command to score/eject algae.
     * 
     * @return A command to score algae
     */
    public Command scoreAlgae() {
        return new RunCommand(() -> 
            effector.setEffectorOutput(EffectorConstants.SCORE_ALGAE),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("ScoreAlgae");
    }

    /**
     * Creates a command that immediately stops the effector motor.
     * 
     * @return A command to stop the motor
     */
    public Command stopEffector() {
        return Commands.runOnce(() -> effector.stopMotor(), effector)
                .withName("StopEffector");
    }

    /**
     * Creates a command to hold algae using minimal power to maintain grip.
     * 
     * @return A command to hold algae
     */
    public Command holdAlgae() {
        return new RunCommand(() -> 
            effector.setEffectorOutput(EffectorConstants.HOLD_ALGAE),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("HoldAlgae");
    }

    /**
     * Creates a command to run the effector motor at the specified output.
     * 
     * @param output The duty cycle output to apply (-1.0 to 1.0)
     * @return A command to run the motor at the specified output
     */
    public Command runAtOutput(double output) {
        return new RunCommand(() -> 
            effector.setEffectorOutput(output),
            effector
        ).finallyDo((interrupted) -> effector.stopMotor())
         .withName("RunAtOutput(" + output + ")");
    }
}