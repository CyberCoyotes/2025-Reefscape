package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristMotorSubsystem;
import frc.robot.subsystems.wrist.WristConstants.Positions;

/**
 * Factory for creating wrist-related commands.
 * This class provides static methods to create commands for controlling the wrist subsystem.
 */
public class WristCommands {

    public WristCommands(WristSubsystem wrist) {
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

        public static Command setSafePose(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.SAFE).withName("WristSafeForElevator");
        }
        public static Command setLoadCoral(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.LOAD_CORAL).withName("LoadCoral");
        }
    
        public static Command setGrabAlgae(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.GRAB_ALGAE).withName("LoadCoral");
        }

        public static Command setL1(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L1).withName("WristL1");
        }

        public static Command setL2(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L2).withName("WristL2");
        }

        public static Command setL3(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L3).withName("WristL3");
        }
        public static Command setL4(WristSubsystem wrist) {
            return setPosition(wrist, WristConstants.Positions.L4).withName("WristL4");
        }



    
}