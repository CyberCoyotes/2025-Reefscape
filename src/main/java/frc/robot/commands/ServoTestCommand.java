package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ServoTestCommand extends Command {
    private final ClimberSubsystem climber;
    private boolean testComplete = false;
    private long startTime;
    private int testPhase = 0;

    public ServoTestCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        testPhase = 0;
        testComplete = false;
        SmartDashboard.putString("ServoTest", "Started");
    }

    @Override
    public void execute() {
        long elapsedTime = System.currentTimeMillis() - startTime;
        
        // Phase 0: Move to min angle
        if (testPhase == 0 && elapsedTime > 1000) {
            climber.setServoAngle(0.0);
            SmartDashboard.putString("ServoTest", "Min Angle (0°)");
            testPhase = 1;
            startTime = System.currentTimeMillis();
        }
        // Phase 1: Move to 90 degrees
        else if (testPhase == 1 && elapsedTime > 2000) {
            climber.setServoAngle(90.0);
            SmartDashboard.putString("ServoTest", "Middle (90°)");
            testPhase = 2;
            startTime = System.currentTimeMillis();
        }
        // Phase 2: Move to max angle
        else if (testPhase == 2 && elapsedTime > 2000) {
            climber.setServoAngle(180.0);
            SmartDashboard.putString("ServoTest", "Max Angle (180°)");
            testPhase = 3;
            startTime = System.currentTimeMillis();
        }
        // Phase 3: Toggle
        else if (testPhase == 3 && elapsedTime > 2000) {
            climber.rotateServo90Degrees();
            SmartDashboard.putString("ServoTest", "Toggle Test");
            testPhase = 4;
            startTime = System.currentTimeMillis();
        }
        // Complete test
        else if (testPhase == 4 && elapsedTime > 2000) {
            testComplete = true;
            SmartDashboard.putString("ServoTest", "Complete");
        }
        
        // Display current angle
        SmartDashboard.putNumber("ServoCurrentAngle", climber.getServoAngle());
    }

    @Override
    public boolean isFinished() {
        return testComplete;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putString("ServoTest", "Interrupted");
        }
    }
}