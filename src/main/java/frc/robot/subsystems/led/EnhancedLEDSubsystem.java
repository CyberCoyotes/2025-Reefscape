package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.TOFSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class EnhancedLEDSubsystem {
    private final LEDSubsystem ledSubsystem;
    
    // LED Patterns
    private static class LEDPattern {
        public final LEDColor color;
        public final LEDSubsystem.AnimationTypes animationType;
        
        public LEDPattern(LEDColor color, LEDSubsystem.AnimationTypes animationType) {
            this.color = color;
            this.animationType = animationType;
        }
    }
    
    // Pattern definitions for different feedback types
    private static final LEDPattern CORAL_DETECTED = 
        new LEDPattern(LEDColor.GREEN, LEDSubsystem.AnimationTypes.SolidColor);
    private static final LEDPattern CORAL_NOT_DETECTED = 
        new LEDPattern(LEDColor.RED, LEDSubsystem.AnimationTypes.Strobe);
    private static final LEDPattern CORAL_INTAKING = 
        new LEDPattern(LEDColor.CYAN, LEDSubsystem.AnimationTypes.SingleFade);
    private static final LEDPattern CORAL_SCORING = 
        new LEDPattern(LEDColor.PINK, LEDSubsystem.AnimationTypes.ColorFlow);
    
    private static final LEDPattern ELEVATOR_AT_TARGET = 
        new LEDPattern(LEDColor.GREEN, LEDSubsystem.AnimationTypes.SolidColor);
    private static final LEDPattern ELEVATOR_MOVING = 
        new LEDPattern(LEDColor.YELLOW, LEDSubsystem.AnimationTypes.Larson);
    private static final LEDPattern ELEVATOR_ERROR = 
        new LEDPattern(LEDColor.RED, LEDSubsystem.AnimationTypes.Strobe);
    
    private static final LEDPattern ROBOT_ENABLED = 
        new LEDPattern(LEDColor.GREEN, LEDSubsystem.AnimationTypes.SolidColor);
    private static final LEDPattern ROBOT_DISABLED = 
        new LEDPattern(LEDColor.RED, LEDSubsystem.AnimationTypes.SolidColor);
    private static final LEDPattern AUTONOMOUS_MODE = 
        new LEDPattern(LEDColor.BLUE, LEDSubsystem.AnimationTypes.Rainbow);
    
    // Current active pattern and section
    private LEDPattern activePattern;
    private LEDSection activeSection = LEDSection.ALL;
    
    // LED Sections for better control
    public enum LEDSection {
        ALL(0, 30), 
        FRONT(0, 10),
        MIDDLE(10, 10),
        BACK(20, 10);
        
        public final int startIndex;
        public final int length;
        
        LEDSection(int startIndex, int length) {
            this.startIndex = startIndex;
            this.length = length;
        }
    }
    
    // LED Colors for easier reference
    public enum LEDColor {
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        CYAN(0, 255, 255),
        ORANGE(255, 165, 0),
        YELLOW(255, 255, 0),
        PURPLE(128, 0, 128),
        WHITE(255, 255, 255),
        PINK(255, 20, 147),
        OFF(0, 0, 0);
        
        public final int r, g, b;
        
        LEDColor(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
    
    // Constructor
    public EnhancedLEDSubsystem(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.activePattern = ROBOT_DISABLED;
    }
    
    // Apply a pattern to the LEDs
    private void applyPattern(LEDPattern pattern, LEDSection section) {
        activePattern = pattern;
        activeSection = section;
        
        if (pattern.animationType == LEDSubsystem.AnimationTypes.SolidColor) {
            ledSubsystem.setColor(pattern.color);
        } else {
            ledSubsystem.setAnimation(pattern.animationType);
        }
    }
    
    // Apply a pattern to all LEDs
    private void applyPattern(LEDPattern pattern) {
        applyPattern(pattern, LEDSection.ALL);
    }
    
    // Basic system state feedback commands
    public Command robotEnabledLEDs() {
        return Commands.runOnce(() -> applyPattern(ROBOT_ENABLED));
    }
    
    public Command robotDisabledLEDs() {
        return Commands.runOnce(() -> applyPattern(ROBOT_DISABLED));
    }
    
    public Command autonomousModeLEDs() {
        return Commands.runOnce(() -> applyPattern(AUTONOMOUS_MODE));
    }
    
    // End Effector LaserCan feedback commands
    public Command coralDetectionFeedback(EffectorSubsystem effector) {
        return new RunCommand(() -> {
            if (effector.isCoralDetected()) {
                applyPattern(CORAL_DETECTED);
            } else {
                applyPattern(CORAL_NOT_DETECTED);
            }
        }, ledSubsystem);
    }
    
    public Command coralIntakingFeedback() {
        return Commands.runOnce(() -> applyPattern(CORAL_INTAKING));
    }
    
    public Command coralScoringFeedback() {
        return Commands.runOnce(() -> applyPattern(CORAL_SCORING));
    }
    
    // TOF (Time-of-Flight) sensor feedback
    public Command tofSensorFeedback(TOFSubsystem tofSubsystem, double thresholdDistance) {
        return new RunCommand(() -> {
            if (tofSubsystem.isElevatorRangeValid() && 
                tofSubsystem.getElevatorDistanceMillimeters() < thresholdDistance) {
                applyPattern(CORAL_DETECTED);
            } else {
                applyPattern(CORAL_NOT_DETECTED);
            }
        }, ledSubsystem);
    }
    
    // Elevator position feedback
    public Command elevatorPositionFeedback(ElevatorSubsystem elevator) {
        return new RunCommand(() -> {
            double targetPosition = elevator.getTargetPosition();
            double currentPosition = elevator.getPosition();
            boolean atTarget = elevator.isAtPosition(targetPosition);
            
            if (atTarget) {
                applyPattern(ELEVATOR_AT_TARGET);
            } else {
                // If we're within 20% of target, show yellow LEDs
                double error = Math.abs(currentPosition - targetPosition);
                double totalDistance = Math.abs(targetPosition);
                if (totalDistance > 0 && error / totalDistance < 0.2) {
                    applyPattern(ELEVATOR_MOVING);
                } else {
                    applyPattern(ELEVATOR_ERROR);
                }
            }
        }, ledSubsystem);
    }
    
    // Generic conditional LED feedback
    public Command conditionalLEDFeedback(
            BooleanSupplier condition, 
            LEDColor trueColor, 
            LEDColor falseColor,
            LEDSubsystem.AnimationTypes trueAnimation,
            LEDSubsystem.AnimationTypes falseAnimation) {
        
        return new RunCommand(() -> {
            if (condition.getAsBoolean()) {
                if (trueAnimation == LEDSubsystem.AnimationTypes.SolidColor) {
                    ledSubsystem.setColor(trueColor);
                } else {
                    ledSubsystem.setAnimation(trueAnimation);
                }
            } else {
                if (falseAnimation == LEDSubsystem.AnimationTypes.SolidColor) {
                    ledSubsystem.setColor(falseColor);
                } else {
                    ledSubsystem.setAnimation(falseAnimation);
                }
            }
        }, ledSubsystem);
    }
    
    // Simplified conditional LED feedback
    public Command simpleLEDFeedback(BooleanSupplier condition, LEDColor trueColor, LEDColor falseColor) {
        return conditionalLEDFeedback(
            condition, 
            trueColor, 
            falseColor,
            LEDSubsystem.AnimationTypes.SolidColor,
            LEDSubsystem.AnimationTypes.SolidColor
        );
    }
    
    // Custom animations
    public Command blinkLEDs(LEDColor color, double onTime, double offTime) {
        return new FunctionalCommand(
            () -> {}, // Init
            () -> {
                double timestamp = Timer.getFPGATimestamp();
                if ((timestamp % (onTime + offTime)) < onTime) {
                    ledSubsystem.setColor(color);
                } else {
                    ledSubsystem.setColor(LEDColor.OFF);
                }
            }, // Execute
            (interrupted) -> ledSubsystem.stopLEDs(), // End
            () -> false, // isFinished - runs until interrupted
            ledSubsystem
        );
    }
    
    // Progress bar effect - useful for showing progress towards target
    public Command progressBarLEDs(Supplier<Double> progressSupplier, LEDColor fillColor, LEDColor emptyColor) {
        return new RunCommand(() -> {
            double progress = progressSupplier.get();
            progress = Math.min(Math.max(progress, 0.0), 1.0); // Clamp between 0 and 1
            
            int totalLEDs = 30; // Assuming 30 LEDs total
            int litLEDs = (int)(progress * totalLEDs);
            
            // TODO: This would need to be implemented if your LEDSubsystem supports setting individual LEDs
            // For now, we'll just change color based on progress
            if (progress > 0.95) {
                ledSubsystem.setColor(fillColor);
            } else if (progress > 0.0) {
                // Blend between colors based on progress
                int r = (int)(emptyColor.r + progress * (fillColor.r - emptyColor.r));
                int g = (int)(emptyColor.g + progress * (fillColor.g - emptyColor.g));
                int b = (int)(emptyColor.b + progress * (fillColor.b - emptyColor.b));
                ledSubsystem.setSolidColor(r, g, b);
            } else {
                ledSubsystem.setColor(emptyColor);
            }
        }, ledSubsystem);
    }
    
    // Rainbow animation when a target is reached
    public Command celebrationLEDs(double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(() -> ledSubsystem.setAnimation(LEDSubsystem.AnimationTypes.Rainbow)),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(() -> ledSubsystem.stopLEDs())
        );
    }
    
    // Command to turn off all LEDs
    public Command turnOffLEDs() {
        return Commands.runOnce(() -> ledSubsystem.stopLEDs());
    }
}