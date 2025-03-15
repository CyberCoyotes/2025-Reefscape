package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

@SuppressWarnings("unused") // Suppress warnings for unused imports and methods

/**
 * Command factory class for the Elevator subsystem.
 * This class contains methods that create and return commands for the elevator.
 */

public class ElevatorCommands {
    private final ElevatorSubsystem elevator;
    // private final WristSubsystem wrist; // Uncomment if wrist functionality is needed
    
    

    public ElevatorCommands(ElevatorSubsystem elevator /*, WristSubsystem wrist */) {
        this.elevator = elevator;
        // this.wrist = wrist;
    }

    /**
     * Creates a command to continuously increment elevator up while button is held
     */
    public Command incrementUp() {
        return elevator.run(() -> {
            double currentPos = elevator.getPosition();
            double newTarget = currentPos + ElevatorConstants.INCREMENT_VALUE;
            
            // Make sure we don't exceed soft limits
            if (newTarget <= ElevatorConstants.FORWARD_LIMIT) {
                elevator.setPosition(newTarget);
            }
        }).withName("ElevatorIncrement(up)");
    }

    /**
     * Creates a command to continuously increment elevator down while button is held
     */
    public Command incrementDown() {
        return elevator.run(() -> {
            double currentPos = elevator.getPosition();
            double newTarget = currentPos - ElevatorConstants.INCREMENT_VALUE;
            
            // Make sure we don't exceed soft limits
            if (newTarget >= ElevatorConstants.REVERSE_LIMIT) {
                elevator.setPosition(newTarget);
            }
        }).withName("ElevatorIncrement(down)");
    }

    /**
     * Creates a command that moves the elevator to a target position
     * This version does not check wrist position
     */
    public Command setPosition(double targetPosition) {
        return new FunctionalCommand(
            () -> {
                // Log that we're starting the elevator movement
                System.out.println("Starting elevator movement to " + targetPosition);
            },
            // Move the elevator to the target position using the elevator subsystem
            () -> elevator.setPosition(targetPosition),
            interrupted -> {
                if (interrupted) {
                    System.out.println("Elevator movement interrupted");
                } else {
                    System.out.println("Elevator reached position " + targetPosition);
                }
            },
            () -> elevator.isAtPosition(targetPosition),
            elevator
        ).withName("MoveElevatorTo(" + targetPosition + ")");
    }

    // Preset position commands
    public Command setHome() {
        return setPosition(ElevatorPosition.HOME.getPosition()).withName("MoveElevatorToHome");
    }

    public Command setL1() {
        return setPosition(ElevatorPosition.L1.getPosition()).withName("MoveElevatorToL1Pose");
    }

    public Command setL2() {
        return setPosition(ElevatorPosition.L2.getPosition()).withName("MoveElevatorToL2Pose");
    }

    public Command setL3() {
        return setPosition(ElevatorPosition.L3.getPosition()).withName("MoveElevatorToL3Pose");
    }

    public Command setL4() {
        return setPosition(ElevatorPosition.L4.getPosition()).withName("MoveElevatorToL4Pose");
    }

    public Command setAlgae2() {
        return setPosition(ElevatorPosition.ALGAE2.getPosition()).withName("MoveElevatorToAlgae2Pose");
    }

    public Command setAlgae3() {
        return setPosition(ElevatorPosition.ALGAE3.getPosition()).withName("MoveElevatorToAlgae3Pose");
    }

    public Command setScoreAlgae() {
        return setPosition(ElevatorPosition.SCORE_ALGAE.getPosition()).withName("MoveElevatorToScoreAlgaePose");
    }

    public Command setIntakeCoral() {
        return setPosition(ElevatorPosition.INTAKE_CORAL.getPosition()).withName("SafeMoveElevatorToIntakeCoralPose");
    }
}