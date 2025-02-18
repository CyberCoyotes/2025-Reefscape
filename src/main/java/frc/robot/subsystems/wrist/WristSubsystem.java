package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;

public class WristSubsystem extends SubsystemBase {

    private final TalonFX wristMotor;
    private final CANcoder wristEncoder;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);

    private static final double POSITION_TOLERANCE = 0.5; // Acceptable error in degrees
    private static final double INCREMENT_STEP = 0.05; // Small adjustment step

    private static final double HOME_POSITION = 0.0;
    private static final double L1_POSITION = 30.0;  // Degrees
    private static final double L2_POSITION = 60.0;  // Degrees
    private static final double L4_POSITION = 120.0; // Degrees

    private static final double POSITION_CONVERSION_FACTOR = 1.0 / 360.0; // Converts degrees to rotations
    private double targetPosition = HOME_POSITION;

    public WristSubsystem() {
        this.wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
        this.wristEncoder = new CANcoder(Constants.WRIST_ENCODER_ID);

        configureMotor();
        configureCANCoder();
    }

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

    private void configureCANCoder() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        wristEncoder.getConfigurator().apply(encoderConfig);
    }

    public double getWristAngle() {
        return wristMotor.getPosition().getValueAsDouble() * 360.0; // Convert rotations to degrees
    }

    public void setTargetPosition(double positionDegrees) {
        targetPosition = positionDegrees * POSITION_CONVERSION_FACTOR; // Convert to rotations
        wristMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    public boolean isAtTarget() {
        return Math.abs(getWristAngle() - targetPosition * 360.0) < POSITION_TOLERANCE;
    }

    // Factory Methods for Commands
    public Command moveToHome() {
        return Commands.runOnce(() -> setTargetPosition(HOME_POSITION), this);
    }

    public Command moveToL1() {
        return Commands.runOnce(() -> setTargetPosition(L1_POSITION), this);
    }

    public Command moveToL2() {
        return Commands.runOnce(() -> setTargetPosition(L2_POSITION), this);
    }

    public Command moveToL4() {
        return Commands.runOnce(() -> setTargetPosition(L4_POSITION), this);
    }

    public Command adjustPosition(double delta) {
        return Commands.runOnce(() -> setTargetPosition(getWristAngle() + delta), this);
    }

    public Command bindPOVIncrementalControl(XboxController controller) {
        return Commands.run(() -> {
            if (controller.getPOV() == 0) {
                setTargetPosition(getWristAngle() + INCREMENT_STEP);
            } else if (controller.getPOV() == 180) {
                setTargetPosition(getWristAngle() - INCREMENT_STEP);
            }
        }, this);
    }
}
