package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignToTagCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController forwardController;
    private final PIDController lateralController;
    private final PIDController rotationController;
    private final Timer validationTimer = new Timer();
    private final Timer lostTargetTimer = new Timer();
    
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    
    public AlignToTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        // Initialize PID controllers
        forwardController = new PIDController(
            VisionSubsystem.AlignmentConstants.PID.FORWARD_P,
            VisionSubsystem.AlignmentConstants.PID.FORWARD_I,
            VisionSubsystem.AlignmentConstants.PID.FORWARD_D
        );
        forwardController.setTolerance(VisionSubsystem.AlignmentConstants.PID.FORWARD_TOLERANCE);
        
        lateralController = new PIDController(
            VisionSubsystem.AlignmentConstants.PID.LATERAL_P,
            VisionSubsystem.AlignmentConstants.PID.LATERAL_I,
            VisionSubsystem.AlignmentConstants.PID.LATERAL_D
        );
        lateralController.setTolerance(VisionSubsystem.AlignmentConstants.PID.LATERAL_TOLERANCE);
        
        rotationController = new PIDController(
            VisionSubsystem.AlignmentConstants.PID.ROTATION_P,
            VisionSubsystem.AlignmentConstants.PID.ROTATION_I,
            VisionSubsystem.AlignmentConstants.PID.ROTATION_D
        );
        rotationController.setTolerance(VisionSubsystem.AlignmentConstants.PID.ROTATION_TOLERANCE);
        
        addRequirements(vision, drivetrain);
    }
    
    @Override
    public void initialize() {
        vision.setLeds(true);
        
        // Configure setpoints
        forwardController.setSetpoint(VisionSubsystem.AlignmentConstants.TARGET_DISTANCE);
        lateralController.setSetpoint(0.0);
        rotationController.setSetpoint(0.0);
        
        validationTimer.restart();
        lostTargetTimer.restart();
        
        logInitialSetup();
    }
    
    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            handleNoTarget();
            return;
        }
        
        // Reset the lost target timer since we have a target
        lostTargetTimer.restart();
        
        // Get position data in target space
        double[] targetSpace = vision.getBotPoseTargetSpace();
        
        // Calculate control outputs
        double forwardSpeed = forwardController.calculate(targetSpace[2]); // Z in target space
        double lateralSpeed = lateralController.calculate(targetSpace[0]); // X in target space
        double rotationRate = rotationController.calculate(targetSpace[4]); // Pitch in target space
        
        // Apply speed limits
        forwardSpeed = MathUtil.clamp(forwardSpeed, 
            -VisionSubsystem.AlignmentConstants.MAX_SPEED, 
            VisionSubsystem.AlignmentConstants.MAX_SPEED);
            
        lateralSpeed = MathUtil.clamp(lateralSpeed, 
            -VisionSubsystem.AlignmentConstants.MAX_SPEED, 
            VisionSubsystem.AlignmentConstants.MAX_SPEED);
            
        rotationRate = MathUtil.clamp(rotationRate, 
            -VisionSubsystem.AlignmentConstants.MAX_ANGULAR_SPEED, 
            VisionSubsystem.AlignmentConstants.MAX_ANGULAR_SPEED);
        
        // Apply control
        drivetrain.setControl(drive
            .withVelocityX(forwardSpeed)
            .withVelocityY(lateralSpeed)
            .withRotationalRate(rotationRate));
        
        // Log data
        logAlignmentData(targetSpace, forwardSpeed, lateralSpeed, rotationRate);
        
        // Reset validation timer if not at setpoint
        if (!atSetpoint()) {
            validationTimer.restart();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
            
        SmartDashboard.putString("TagAlign/Status", interrupted ? "INTERRUPTED" : "COMPLETED");
    }
    
    @Override
    public boolean isFinished() {
        // We're done if:
        // 1. We lost the target for too long, or
        // 2. We're at the setpoint and have been for the validation period
        return lostTargetTimer.hasElapsed(0.3) || 
               (atSetpoint() && validationTimer.hasElapsed(0.3));
    }
    
    private boolean atSetpoint() {
        return forwardController.atSetpoint() && 
               lateralController.atSetpoint() && 
               rotationController.atSetpoint();
    }
    
    private void handleNoTarget() {
        // Stop the robot when target is lost
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
            
        SmartDashboard.putString("TagAlign/Status", "NO TARGET");
    }
    
    private void logInitialSetup() {
        SmartDashboard.putString("TagAlign/Status", "RUNNING");
    }
    
    private void logAlignmentData(double[] position, double forwardSpeed, 
                                double lateralSpeed, double rotationRate) {
        SmartDashboard.putNumber("TagAlign/X", position[0]);
        SmartDashboard.putNumber("TagAlign/Y", position[1]);
        SmartDashboard.putNumber("TagAlign/Z", position[2]);
        SmartDashboard.putNumber("TagAlign/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("TagAlign/LateralSpeed", lateralSpeed);
        SmartDashboard.putNumber("TagAlign/RotationRate", rotationRate);
        SmartDashboard.putBoolean("TagAlign/AtSetpoint", atSetpoint());
    }
}