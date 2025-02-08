package frc.robot.commands;

public class ZeroWristCommand {
    
}

/*
 * public class ZeroWristCommand extends Command {
    private final WristSubsystem wrist;
    private final DigitalInput limitSwitch;
    private static final double ZEROING_SPEED = -0.1; // Slow speed
    
    public ZeroWristCommand(WristSubsystem wrist, DigitalInput limitSwitch) {
        this.wrist = wrist;
        this.limitSwitch = limitSwitch;
        addRequirements(wrist);
    }
    
    @Override
    public void initialize() {
        // Start moving slowly towards limit switch
        wrist.setManualOutput(ZEROING_SPEED);
    }
    
    @Override
    public boolean isFinished() {
        // Stop when limit switch is hit
        return limitSwitch.get();
    }
    
    @Override
    public void end(boolean interrupted) {
        wrist.stop();
        if (!interrupted) {
            wrist.setWristZero(); // Zero encoder at known position
        }
    }
}
 */

 // https://claude.ai/chat/e4dc3456-f3f1-4b08-bfdf-d133a2d2e628