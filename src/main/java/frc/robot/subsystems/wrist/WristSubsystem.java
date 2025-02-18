package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

    private static final double POSITION_TOLERANCE = 0.5;
    private static final double INCREMENT_STEP = 2.0; // Adjusted for better visibility

    private static final double HOME_POSITION = 0.0;
    private static final double L1_POSITION = 5.0;
    private static final double L2_POSITION = 15.0;
    private static final double L4_POSITION = 30.0;

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

        motorConfig.Slot0.kP = 1.0;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.05;
        motorConfig.Slot0.kV = 0.12;
        motorConfig.Slot0.kS = 0.2;
        motorConfig.Slot0.kG = 0.3;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 30.0;
        motorConfig.MotionMagic.MotionMagicJerk = 200.0;

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = 1.0;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply the configuration to the motor when the method is called
        wristMotor.getConfigurator().apply(motorConfig);
    }

    private void configureCANCoder() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = WristConstants.MAGNET_ENCODER_OFFSET;

        // Apply the configuration to the CANCoder when the method is called
        wristEncoder.getConfigurator().apply(encoderConfig);
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