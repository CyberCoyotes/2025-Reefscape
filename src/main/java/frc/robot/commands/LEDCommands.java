package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LEDControlSystem;
import frc.robot.subsystems.led.LEDControlSystem.LEDColor;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.TOFSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

/**
 * Factory for creating LED feedback commands for different subsystems.
 * This class centralizes all the LED command creation to make it easier to
 * maintain and update.
 */
public class LEDCommands {
    private final LEDControlSystem ledControl;
    
    public LEDCommands(LEDSubsystem ledSubsystem) {
        this.ledControl = new LEDControlSystem(ledSubsystem);
    }
    
    /**
     * Creates a command that displays feedback for the end effector's coral detection.
     * Green solid light when coral is detected, red strobe when not detected.
     */
    public Command effectorCoralDetectionCommand(EffectorSubsystem effector) {
        return ledControl.coralDetectionFeedback(effector);
    }
    
    /**
     * Creates a command that displays feedback for the effector's state during manual control.
     * Changes LED color based on the effector's state (intaking, scoring, etc.)
     */
    public Command effectorStateFeedbackCommand(EffectorSubsystem effector) {
        return ledControl.simpleLEDFeedback(
            effector::isCoralDetected,
            LEDColor.GREEN,
            LEDColor.ORANGE
        );
    }
    
    /**
     * Creates a command that displays coral intaking feedback.
     * Cyan pulsing lights.
     */
    public Command coralIntakingCommand() {
        return ledControl.coralIntakingFeedback();
    }
    
    /**
     * Creates a command that displays coral scoring feedback.
     * Pink color flow animation.
     */
    public Command coralScoringCommand() {
        return ledControl.coralScoringFeedback();
    }
    
    /**
     * Creates a command for elevator position feedback.
     * Green when at target, yellow larson scanner when moving, red strobe for error.
     */
    public Command elevatorPositionCommand(ElevatorSubsystem elevator) {
        return ledControl.elevatorPositionFeedback(elevator);
    }
    
    /**
     * Creates a command that shows elevator progress towards target as a color blend.
     */
    public Command elevatorProgressCommand(ElevatorSubsystem elevator) {
        return ledControl.progressBarLEDs(
            () -> {
                double target = elevator.getTargetPosition();
                double current = elevator.getPosition();
                // If we're at zero position and target is zero, we're at 100%
                if (target == 0 && Math.abs(current) < 0.01) return 1.0;
                // Otherwise calculate progress
                if (target == 0) return 0.0;
                double progress = current / target;
                return progress > 1.0 ? 1.0 : progress < 0.0 ? 0.0 : progress;
            },
            LEDColor.GREEN,
            LEDColor.RED
        );
    }
    
    /**
     * Creates a command for TOF sensor distance feedback.
     * Green when distance is less than threshold, red when greater.
     */
    public Command tofDistanceFeedbackCommand(TOFSubsystem tofSubsystem, double thresholdDistance) {
        return ledControl.tofSensorFeedback(tofSubsystem, thresholdDistance);
    }
    
    /**
     * Creates a command that blinks LEDs when the wrist is moving.
     */
    public Command wristMovementCommand(WristSubsystem wristSubsystem) {
        return ledControl.simpleLEDFeedback(
            () -> wristSubsystem.atPosition(wristSubsystem.getPosition(), 0.05),
            LEDColor.GREEN,
            LEDColor.YELLOW
        );
    }
    
    /**
     * Creates a command that shows a rainbow celebration animation when a target is reached.
     */
    public Command targetReachedCelebrationCommand(double durationSeconds) {
        return ledControl.celebrationLEDs(durationSeconds);
    }
    
    /**
     * Creates a command that turns off all LEDs.
     */
    public Command turnOffLEDsCommand() {
        return ledControl.turnOffLEDs();
    }
    
    /**
     * Creates a teleoperation default LED command that provides feedback
     * about the system's state.
     */
    public Command createTeleopDefaultCommand(
            EffectorSubsystem effector, 
            ElevatorSubsystem elevator) {
        
        return Commands.parallel(
            // First priority: Coral detection if effector is active
            Commands.either(
                ledControl.coralDetectionFeedback(effector),
                // Second priority: Elevator position feedback
                ledControl.elevatorPositionFeedback(elevator),
                // Condition: Is the effector active?
                () -> Math.abs(effector.getCoralDistanceMillimeters()) < 200
            )
        ).withName("Default LED Feedback");
    }
}

// https://claude.ai/chat/786aa812-ecea-4d23-a82d-5fbee0ef9bb2