package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {

    private final TalonFX wristMotor;
    private final CANcoder wristEncoder;
    private final MotionMagicVoltage wristMotionRequest;

    private static final double INCREMENT_STEP = 2.0; // Adjusted for better visibility

    private static final double HOME_POSITION = 0.0;
    

    private double targetPosition = HOME_POSITION;

    public WristSubsystem() {
        this.wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
        this.wristEncoder = new CANcoder(Constants.WRIST_ENCODER_ID);

        wristMotionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        configureMotor();
        configureCANCoder();
        resetWristMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    
        // PID and FF gains from your constants
        motorConfig.Slot0.kP = WristConstants.Gains.kP;
        motorConfig.Slot0.kI = WristConstants.Gains.kI;
        motorConfig.Slot0.kD = WristConstants.Gains.kD;
        motorConfig.Slot0.kV = WristConstants.Gains.kV;
        motorConfig.Slot0.kS = WristConstants.Gains.kS;
        motorConfig.Slot0.kG = WristConstants.Gains.kG;
        motorConfig.Slot0.kA = WristConstants.Gains.kA;
    
        // Motion Magic settings from constants
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MOTION_MAGIC_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = WristConstants.MOTION_MAGIC_ACCELERATION;
        motorConfig.MotionMagic.MotionMagicJerk = WristConstants.MOTION_MAGIC_JERK;
    
        // Critical: Configure feedback for FusedCANcoder
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID(); // TODO This was added
        motorConfig.Feedback.RotorToSensorRatio = WristConstants.GEAR_RATIO;
        motorConfig.Feedback.SensorToMechanismRatio = WristConstants.ENCODER_TO_MECHANISM_RATIO;
    
        // TODO This was added
        // Current limits
        motorConfig.CurrentLimits.StatorCurrentLimit = WristConstants.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
        // This was added
        // Soft limits
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.FORWARD_LIMIT;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.REVERSE_LIMIT;
    
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
        var result = wristMotor.getConfigurator().apply(motorConfig);
        if (!result.isOK()) {
            System.out.println("Failed to apply motor configuration: " + result.toString());
        }
    }

    private void configureCANCoder() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = WristConstants.MAGNET_ENCODER_OFFSET;
        
        // Important: Configure absolute sensor range
        // encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        
        // New way to configure absolute sensor discontinuity
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;

        var result = wristEncoder.getConfigurator().apply(encoderConfig);

        if (!result.isOK()) {
            System.out.println("Failed to apply encoder configuration: " + result.toString());
        }
    }
    public void resetWristMotor() {
        double absolutePosition = getWristCANCoderAngle() / 360.0;
        wristMotor.setPosition(absolutePosition);
        System.out.println("Wrist motor reset to: " + absolutePosition + " rotations");
    }

    public double getWristMotorAngle() {
        return wristMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public double getWristCANCoderAngle() {
        // return wristEncoder.getPosition().getValueAsDouble() * 360.0;
        return (wristEncoder.getPosition().getValueAsDouble() - WristConstants.MAGNET_ENCODER_OFFSET) * 360.0;

    }

    public void setWristMotorTargetPosition(double positionDegrees) {
        targetPosition = positionDegrees / 360.0;
            wristMotor.setControl(wristMotionRequest.withPosition(targetPosition)); // APPLY IMMEDIATELY

    }

    public void incrementWristPosition(boolean up) {
        double newTarget = getWristMotorAngle() + (up ? INCREMENT_STEP : -INCREMENT_STEP);
        setWristMotorTargetPosition(newTarget);
    }

    /** Command factory to increment the wrist up */
    public Command incrementWristUp() {
        return Commands.runOnce(() -> incrementWristPosition(true))
            .withName("WristIncrementUp");
    }

    /** Command factory to increment the wrist down */
    public Command incrementWristDown() {
        return Commands.runOnce(() -> incrementWristPosition(false))
            .withName("WristIncrementDown");
    }

    // FIXME for debugging
    public void testManualPower() {
        wristMotor.set(0.2);
        System.out.println("Wrist motor test, apply 20% power");
    }

    @Override
    public void periodic() {
        wristMotor.setControl(wristMotionRequest.withPosition(targetPosition));

        /*** Debugging ***/
        double fusedAngle = wristEncoder.getPosition().getValueAsDouble() * 360.0;
        double absoluteAngle = wristEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        double motorAngle = wristMotor.getPosition().getValueAsDouble() * 360.0;

    
        SmartDashboard.putNumber("Wrist/Fused CANCoder Angle", fusedAngle);
        SmartDashboard.putNumber("Wrist/Absolute CANCoder Angle", absoluteAngle);
        SmartDashboard.putNumber("Wrist/Motor Encoder Angle", motorAngle);
        SmartDashboard.putNumber("Wrist/Motor Voltage", wristMotor.getMotorVoltage().getValueAsDouble());
    
        System.out.println("Wrist Encoder Readings:");
        System.out.println("  - Fused CANCoder Angle: " + fusedAngle);
        System.out.println("  - Absolute CANCoder Angle: " + absoluteAngle);
        System.out.println("  - Motor Encoder Angle: " + motorAngle);
        // end of debugging


        
        /*** SMART DASHBOARD TELEMETRY***/
        // Motor
        // SmartDashboard.putNumber("Wrist/Motor/Target Position (deg)", targetPosition * 360.0);
        SmartDashboard.putNumber("Wrist/Motor/Position (deg)", getWristMotorAngle());
        SmartDashboard.putNumber("Wrist/Motor/Voltage (V)", wristMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Wrist/Motor/Current (A)", wristMotor.getStatorCurrent().getValueAsDouble());

        // CANCoder
        SmartDashboard.putNumber("Wrist/CANCoder/Fused Angle (deg)", getWristCANCoderAngle());
        
        /*** ADVANTAGEKIT ***/
        // Motor telemetry
        Logger.recordOutput("Wrist/Motor/Position (deg)", getWristMotorAngle());
        Logger.recordOutput("Wrist/Motor/Target Position (deg)", targetPosition * 360.0);
        Logger.recordOutput("Wrist/Motor/Current (A)", wristMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Wrist/Motor/Output (V)", wristMotor.getMotorVoltage().getValueAsDouble());

        // CANCoder telemetry
        Logger.recordOutput("Wrist/CANCoder/Absolute Encoder (deg)", getWristCANCoderAngle());
        Logger.recordOutput("Wrist/CANCoder/Fused Angle (deg)", getWristCANCoderAngle());

    }

} // end subsystem class