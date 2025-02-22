package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.endEffector.EffectorConstants;

public class MonitorEffectorSafetyCommand extends Command {
    private final EffectorSubsystem effector;
    private final Joystick controller;
    private final int resumeButton;
    private final int overrideButton;

    public MonitorEffectorSafetyCommand(EffectorSubsystem effector, Joystick controller, int resumeButton, int overrideButton) {
        this.effector = effector;
        this.controller = controller;
        this.resumeButton = resumeButton;
        this.overrideButton = overrideButton;
        addRequirements(effector);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(overrideButton)) {
            // Manual override: always allow motor operation
            effector.setEffectorOutput(EffectorConstants.INTAKE_CORAL);
        } else if (effector.getCoralDistanceMillimeters() < 30) {
            // Stop motor if object detected
            effector.setEffectorOutput(EffectorConstants.STOP);
        } else if (controller.getRawButtonReleased(resumeButton)) {
            // Resume normal operation on button release
            effector.setEffectorOutput(EffectorConstants.SCORE_CORAL);
        } else {
            // Default to intaking coral
            effector.setEffectorOutput(EffectorConstants.INTAKE_CORAL);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Always running
    }
}
