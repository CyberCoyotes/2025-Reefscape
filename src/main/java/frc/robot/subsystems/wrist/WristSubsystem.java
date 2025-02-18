package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

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
    private static final double L1_POSITION = 0.09; // 30
    private static final double L2_POSITION = 0.15; // 60
    private static final double L4_POSITION = 0.25;    // 120

    private double targetPosition = HOME_POSITION;

    public WristSubsystem(TalonFX wristMotor, CANcoder wristEncoder) {
        this.wristMotor = wristMotor;
        this.wristEncoder = wristEncoder;

        configureMotor();
        configureCANCoder();
    }

    private void configureMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.Slot0.kP = 0.2;  // Proportional gain
    motorConfig.Slot0.kI = 0.0;  // Integral gain
    motorConfig.Slot0.kD = 0.1;  // Derivative gain
    motorConfig.Slot0.kV = 0.12; // Velocity feedforward
    motorConfig.Slot0.kS = 0.2;  // Static feedforward (to overcome friction)
    motorConfig.Slot0.kG = 0.3;  // Gravity feedforward (if needed for arms)

    // Motion Magic settings
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 30.0; // Max speed (adjust as needed)
    motorConfig.MotionMagic.MotionMagicAcceleration = 20.0;   // Acceleration rate (tune this)
    motorConfig.MotionMagic.MotionMagicJerk = 100.0;          // Jerk control (smoothing)

    motorConfig.Feedback.SensorToMechanismRatio = 1.0; // Adjust as needed
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Prevent free movement

    wristMotor.getConfigurator().apply(motorConfig);
        // motorConfig.Feedback.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        wristMotor.getConfigurator().apply(motorConfig);
    }

    private void configureCANCoder() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        wristEncoder.getConfigurator().apply(encoderConfig);
    }

    public double getArmAngle() {
        return wristEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        wristMotor.setControl(positionRequest.withPosition(targetPosition));
    }

    public boolean isAtTarget() {
        return Math.abs(getArmAngle() - targetPosition) < POSITION_TOLERANCE;
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
        return Commands.runOnce(() -> setTargetPosition(getArmAngle() + delta), this);
    }

    public Command bindPOVIncrementalControl(XboxController controller) {
        return Commands.run(() -> {
            if (controller.getPOV() == 0) {
                setTargetPosition(getArmAngle() + INCREMENT_STEP);
            } else if (controller.getPOV() == 180) {
                setTargetPosition(getArmAngle() - INCREMENT_STEP);
            }
        }, this);
    }

    @Override
    public void periodic() {
        // Update motor control with position tracking
        wristMotor.setControl(positionRequest.withPosition(targetPosition));
    }
}
