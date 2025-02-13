package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristConstants.Positions;

/**
 * Factory for creating wrist-related commands.
 * This class provides static methods to create commands for controlling the wrist subsystem.
 */
public final class WristCommands {
    private WristCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Creates a command to move the wrist to a specific position
     */
    public static Command setPosition(WristSubsystem wrist, double targetRotations) {
        return Commands.run(() -> wrist.setPosition(targetRotations), wrist)
                      .until(() -> wrist.atTargetPosition(0.02))
                      .withName("Wrist To " + targetRotations);
    }

    /**
     * Creates a command to clear sticky faults
     */
    /* 
    public static Command clearFaults(WristSubsystem wrist) {
        return Commands.runOnce(wrist::clearStickyFaults, wrist)
                      .withName("Clear Wrist Faults");
    }*/

    /**
     * Common wrist positions as static command generators
     */
    // public static final class Positions {
        // private Positions() {}

        public static Command loadCoral(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.LOAD_CHORAL).withName("LoadCoral");
        }
        
        public static Command elevatorSafe(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.ELEVATOR_SAFE).withName("WristSafeForElevator");
        }

        public static Command L1(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L1).withName("WristL1");
        }

        public static Command L2(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L2).withName("WristL2");
        }

        public static Command L3(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L3).withName("WristL3");
        }
        public static Command L4(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L4).withName("WristL4");
        }
        // Add more preset positions as needed
    // }

    /**
     * Complex command sequences combining multiple wrist movements
     */
    public static final class Sequences {
        private Sequences() {}

        public static Command loadToSafe(WristSubsystem wrist) {
            return loadCoral(wrist)
                    .andThen(elevatorSafe(wrist))
                    .withName("Stow To Ground");
        }

        // Add more sequences as needed
    }
}