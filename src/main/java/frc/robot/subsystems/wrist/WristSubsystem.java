package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final MotionMagicVoltage motionMagic;
    
    // Configuration constants
    private static final double GEAR_RATIO = 80.0;
    private static final double WRIST_MIN_POSITION = 0.0; // In rotations
    private static final double WRIST_MAX_POSITION = 20.0; // In rotations
    private static final double INCREMENT_AMOUNT = 0.5; // 0.5 rotations per button press
    
    // Motion Magic Constants (from config file)
    private static final double kP = 8.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.0;
    private static final double kS = 0.0;
    private static final double kG = 0.05;
    
    // Motion Profile Constraints (from config file)
    private static final double MOTION_MAGIC_VELOCITY = 80.0; // rotations per second
    private static final double MOTION_MAGIC_ACCELERATION = 80.0; // rotations per second squared
    private static final double MOTION_MAGIC_JERK = 300.0; // rotations per second cubed
    
    private double targetPosition = 0.0;
    private boolean hasBeenReset = false;
    
    public WristSubsystem() {
        wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
        motionMagic = new MotionMagicVoltage(0).withSlot(0);
        
        configureMotor();
        resetWrist();
    }
    
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure Motion Magic and PID
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kS = kS;
        config.Slot0.kG = kG;
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
        
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;
        
        // Configure soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WRIST_MAX_POSITION;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WRIST_MIN_POSITION;
        
        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Set brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Apply configuration
        wristMotor.getConfigurator().apply(config);
    }
    
    public void resetWrist() {
        wristMotor.setPosition(0.0);
        targetPosition = 0.0;
        hasBeenReset = true;
    }
    
    public double getPosition() {
        return wristMotor.getPosition().getValueAsDouble();
    }
    
    public void setPosition(double targetRotations) {
        targetPosition = targetRotations;
        wristMotor.setControl(motionMagic.withPosition(targetRotations));
    }
    
    public void incrementOut() {
        setPosition(Math.min(getPosition() + INCREMENT_AMOUNT, WRIST_MAX_POSITION));
    }
    
    public void incrementIn() {
        setPosition(Math.max(getPosition() - INCREMENT_AMOUNT, WRIST_MIN_POSITION));
    }
    
    public boolean atPosition(double targetRotations, double toleranceRotations) {
        return Math.abs(getPosition() - targetRotations) <= toleranceRotations;
    }
    
    @Override
    public void periodic() {
        // Update SmartDashboard
        SmartDashboard.putNumber("Wrist Position (rot)", getPosition());
        SmartDashboard.putNumber("Wrist Target (rot)", targetPosition);
        SmartDashboard.putBoolean("Wrist Reset Status", hasBeenReset);
        SmartDashboard.putNumber("Wrist Error (rot)", Math.abs(getPosition() - targetPosition));
    
        Logger.recordOutput("Wrist/Position", getPosition());
        Logger.recordOutput("Wrist/Target", targetPosition);
        Logger.recordOutput("Wrist/Reset", hasBeenReset);
        Logger.recordOutput("Wrist Error (deg)", Math.abs(getPosition() - targetPosition));
        Logger.recordOutput("Wrist/Voltage", wristMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Wrist/Position", wristMotor.getPosition().getValue());
        Logger.recordOutput("Wrist/Stator Current", wristMotor.getStatorCurrent().getValue());
    }
    
    // Command Factory methods
    public static class CommandFactory {
        private final WristSubsystem subsystem;
        private static final double DEFAULT_TOLERANCE = 0.05; // 0.05 rotations tolerance
        
        public CommandFactory(WristSubsystem subsystem) {
            this.subsystem = subsystem;
        }
        
        public Command moveToPosition(double targetRotations) {
            return moveToPosition(targetRotations, DEFAULT_TOLERANCE);
        }
        
        public Command moveToPosition(double targetRotations, double toleranceRotations) {
            return this.subsystem.runEnd(
                () -> this.subsystem.setPosition(targetRotations),
                () -> this.subsystem.setPosition(this.subsystem.getPosition())
            )
            .until(() -> this.subsystem.atPosition(targetRotations, toleranceRotations));
        }
        
        public Command incrementOut() {
            return this.subsystem.runOnce(() -> this.subsystem.incrementOut());
        }
        
        public Command incrementIn() {
            return this.subsystem.runOnce(() -> this.subsystem.incrementIn());
        }
        
        public Command moveTo(WristPositions wristPose) {
            return moveToPosition(wristPose.getRotations());
        }
        
        public Command resetWrist() {
            return this.subsystem.runOnce(() -> this.subsystem.resetWrist());
        }
    }
    
    // Preset positions enum
    public enum WristPositions {
        STOWED(0.0),             // Stowed position from config
        SCORE_L2(2.15),          // L2 scoring position from your value
        SCORE_L4(3.5),           // Estimated
        PICK_ALGAE(4.0),         // Estimated
        SCORE_ALGAE(4.5);        // Estimated
        
        private final double rotations;
        
        WristPositions(double rotations) {
            this.rotations = rotations;
        }
        
        public double getRotations() {
            return rotations;
        }
    }
}