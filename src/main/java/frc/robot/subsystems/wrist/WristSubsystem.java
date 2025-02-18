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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {

    private final TalonFX wristMotor;
    private final CANcoder wristEncoder;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);

    private static final double POSITION_TOLERANCE = 0.5; // Acceptable error in degrees
    private static final double INCREMENT_STEP = 0.05; // Small adjustment step

    private static final double HOME_POSITION = 0.0;
    private static final double L1_POSITION = 5.0;  // Degrees
    private static final double L2_POSITION = 15.0;  // Degrees
    private static final double L4_POSITION = 30.0; // Degrees

    private static final double POSITION_CONVERSION_FACTOR = 1.0 / 360.0; // Converts degrees to rotations
    private double targetPosition = HOME_POSITION;

    public WristSubsystem() {
        this.wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
        this.wristEncoder = new CANcoder(Constants.WRIST_ENCODER_ID);

        configureMotor();
        configureCANCoder();
        resetWristMotor(); // Reset motor position on startup
    }

    /** Configures the TalonFX wrist motor to use Motion Magic and CANCoder feedback. */
    private void configureMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // PID and Motion Magic Settings
        motorConfig.Slot0.kP = 0.3;  // Increase P gain slightly
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.05; // Reduce D gain
        motorConfig.Slot0.kV = 0.12;
        motorConfig.Slot0.kS = 0.2;
        motorConfig.Slot0.kG = 0.3;

        // Motion Magic
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0; // Increase velocity
        motorConfig.MotionMagic.MotionMagicAcceleration = 30.0;
        motorConfig.MotionMagic.MotionMagicJerk = 200.0;

        // Use CANCoder for Position Feedback
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristMotor.getConfigurator().apply(motorConfig);
    }

    /** Configures the CANCoder absolute encoder with magnet offset. */
    private void configureCANCoder() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = WristConstants.MAGNET_ENCODER_OFFSET; // Apply offset
        wristEncoder.getConfigurator().apply(encoderConfig);
    }

    /** Resets the TalonFX encoder to match the absolute CANCoder position. */
    public void resetWristMotor() {
        double absolutePosition = getAbsoluteEncoderAngle() / 360.0; // Convert degrees to rotations
        wristMotor.setPosition(absolutePosition); // Reset motor encoder
        System.out.println("Wrist motor reset to: " + absolutePosition + " rotations");
    }

    /** Returns the TalonFX integrated encoder position (motor rotations → degrees). */
    public double getWristMotorAngle() {
        return wristMotor.getPosition().getValueAsDouble() * 360.0; // Convert rotations to degrees
    }

    /** Returns the CANCoder absolute angle (ignoring motor sensor). */
    public double getAbsoluteEncoderAngle() {
        return wristEncoder.getAbsolutePosition().getValueAsDouble() * 360.0; // Convert from rotations to degrees
    }

    /** Returns the CANCoder fused sensor reading (which includes offset). */
    public double getWristCANCoderAngle() {
        return wristEncoder.getPosition().getValueAsDouble() * 360.0; // Convert rotations to degrees
    }

    /** Sets the wrist motor target position using Motion Magic. */
    public void setWristMotorTargetPosition(double positionDegrees) {
        targetPosition = positionDegrees * POSITION_CONVERSION_FACTOR; // Convert to rotations
        wristMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    /** Checks if the wrist motor is at the target position. */
    public boolean isAtWristMotorTarget() {
        return Math.abs(getWristMotorAngle() - targetPosition * 360.0) < POSITION_TOLERANCE;
    }

    /** Factory Methods for Commands */
    public Command moveToHome() {
        return Commands.runOnce(() -> setWristMotorTargetPosition(HOME_POSITION), this);
    }

    public Command moveToL1() {
        return Commands.runOnce(() -> setWristMotorTargetPosition(L1_POSITION), this);
    }

    public Command moveToL2() {
        return Commands.runOnce(() -> setWristMotorTargetPosition(L2_POSITION), this);
    }

    public Command moveToL4() {
        return Commands.runOnce(() -> setWristMotorTargetPosition(L4_POSITION), this);
    }

    public Command adjustPosition(double delta) {
        return Commands.runOnce(() -> setWristMotorTargetPosition(getWristMotorAngle() + delta), this);
    }

    public Command bindPOVIncrementalControl(XboxController controller) {
        return Commands.run(() -> {
            if (controller.getPOV() == 0) {
                setWristMotorTargetPosition(getWristMotorAngle() + INCREMENT_STEP);
            } else if (controller.getPOV() == 180) {
                setWristMotorTargetPosition(getWristMotorAngle() - INCREMENT_STEP);
            }
        }, this);
    }

    public Command resetWristCommand() {
        return Commands.runOnce(this::resetWristMotor, this);
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with telemetry
        SmartDashboard.putNumber("Wrist/Motor/Target Position (deg)", targetPosition * 360.0);
        SmartDashboard.putNumber("Wrist/Motor/Position (deg)", getWristMotorAngle());
        SmartDashboard.putNumber("Wrist/CANCoder/Absolute Encoder (deg)", getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Wrist/CANCoder/Fused Angle (deg)", getWristCANCoderAngle());
        SmartDashboard.putNumber("Wrist/Motor/Voltage (V)", wristMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Wrist/Motor/Current (A)", wristMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Wrist/Motor/At Target", isAtWristMotorTarget());

        // Update AdvantageKit logging
        Logger.recordOutput("Wrist/Motor/Position (deg)", getWristMotorAngle());
        Logger.recordOutput("Wrist/CANCoder/Absolute Encoder (deg)", getAbsoluteEncoderAngle());
        Logger.recordOutput("Wrist/CANCoder/Fused Angle (deg)", getWristCANCoderAngle());
        Logger.recordOutput("Wrist/Motor/Voltage (V)", wristMotor.getMotorVoltage().getValueAsDouble());
    }


/* 
    @Override
    public void periodic() {
        // Update SmartDashboard with telemetry
        SmartDashboard.putNumber("Wrist/Motor/Reset Position (rotations)", wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist/Motor/Target Position (deg)", targetPosition * 360.0);
        SmartDashboard.putNumber("Wrist/Motor/Position (deg)", getWristMotorAngle());
        SmartDashboard.putNumber("Wrist/Motor/Voltage (V)", wristMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Wrist/Motor/Current (A)", wristMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Wrist/Motor/At Target", isAtWristMotorTarget());
        SmartDashboard.putNumber("Wrist/Motor/At Target", getWristMotorAngle());
        SmartDashboard.putNumber("Wrist/CANCoder/Absolute Encoder (deg)", getAbsoluteEncoderAngle());

        // Update AdvantageKit with telem
        Logger.recordOutput("Wrist/Motor/Reset Position (rotations)", wristMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Wrist/Motor/Target Position (deg)", targetPosition * 360.0);
        Logger.recordOutput("Wrist/Motor/Position (deg)", getWristMotorAngle());
        Logger.recordOutput("Wrist/Motor/Voltage (V)", wristMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Wrist/Motor/Current (A)", wristMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Wrist/Motor/At Target", isAtWristMotorTarget());
        Logger.recordOutput("Wrist/Motor/At Target", getWristMotorAngle());
        Logger.recordOutput("Wrist/CANCoder/Absolute Encoder (deg)", getAbsoluteEncoderAngle());
        
 
        } // end periodic*/

} // end subsystem class