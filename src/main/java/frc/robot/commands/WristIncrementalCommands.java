package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristConstants;

public class WristIncrementalCommands {
    private static final double INCREMENTAL_STEP = 0.05; // Rotations per step

    /**
     * Creates a command that allows incremental control of the wrist using D-pad while a button is held.
     * POV right increases position, POV left decreases position.
     * 
     * @param wrist The wrist subsystem
     * @param controller The Xbox controller
     * @param enableButton The button that must be held to enable incremental control
     * @return A command that runs the incremental control
     */
    public static Command createIncrementalCommand(
            WristSubsystem wrist, 
            CommandXboxController controller,
            int enableButton) {
        
        return new Command() {
            private double lastPosition;
            private boolean isFirstRun = true;

            @Override
            public void initialize() {
                lastPosition = wrist.getPosition();
                isFirstRun = true;
            }

            @Override
            public void execute() {
                // Only update if this is the first run or the POV has changed
                if (isFirstRun || controller.getHID().getPOV() != -1) {
                    isFirstRun = false;
                    
                    // Get current POV value
                    int pov = controller.getHID().getPOV();
                    
                    // Calculate new position based on POV
                    double newPosition = lastPosition;
                    if (pov == 90) { // RIGHT on D-pad
                        newPosition += INCREMENTAL_STEP;
                    } else if (pov == 270) { // LEFT on D-pad
                        newPosition -= INCREMENTAL_STEP;
                    }
                    
                    // Clamp the position within valid range
                    newPosition = MathUtil.clamp(
                        newPosition,
                        WristConstants.REVERSE_LIMIT,
                        WristConstants.FORWARD_LIMIT
                    );
                    
                    // Update position if it changed
                    if (newPosition != lastPosition) {
                        wrist.setPosition(newPosition);
                        lastPosition = newPosition;
                    }
                }
            }

            @Override 
            public boolean isFinished() {
                return false; // Run until interrupted
            }

            @Override
            public InterruptionBehavior getInterruptionBehavior() {
                return InterruptionBehavior.kCancelSelf;
            }
        }
        // Command will run only while the enable button is held
        .ignoringDisable(true)
        .unless(() -> !controller.getHID().getRawButton(enableButton));
    }
}