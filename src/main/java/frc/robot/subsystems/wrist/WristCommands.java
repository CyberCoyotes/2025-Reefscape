package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.WristSubsystem;

/**
 * Factory for creating wrist-related commands.
 * This class provides static methods to create commands for controlling the wrist subsystem.
 */
public final class WristCommands {
    // private WristCommands() {
    //     throw new UnsupportedOperationException("This is a utility class!");
    // }

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

        public static Command loadChoral(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.LOAD_CHORAL).withName("Load Choral");
        }
        
        public static Command elevatorSafe(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.ELEVATOR_SAFE).withName("Elevator Safe");
        }

        public static Command L1(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L1).withName("L1 Wrist");
        }

        public static Command L2(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L2).withName("L2 Wrist");
        }

        public static Command L3(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L3).withName("L3 Wrist");
        }
        public static Command L4(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L4).withName("L4 Wrist");
        }
        // Add more preset positions as needed
    // }

    /**
     * Complex command sequences combining multiple wrist movements
     */
    public static final class Sequences {
        private Sequences() {}

        public static Command loadToSafe(WristSubsystem wrist) {
            return loadChoral(wrist)
                    .andThen(elevatorSafe(wrist))
                    .withName("Stow To Ground");
        }

        // Add more sequences as needed
    }
}