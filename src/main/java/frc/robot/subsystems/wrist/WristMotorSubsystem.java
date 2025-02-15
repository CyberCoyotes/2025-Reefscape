// WristSubsystem.java
package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristMotorSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final PositionVoltage positionRequest;
    
    private String currentPositionName = "Unknown";
    private double targetPosition = 0.0;

    public WristMotorSubsystem() {
        wristMotor = new TalonFX(WristConstants.WRIST_ID, WristConstants.CANBUS);
        positionRequest = new PositionVoltage(0).withSlot(0);
        
        configureMotor();
    }

    private void configureMotor() {
        var motorConfig = new TalonFXConfiguration();
        
        // Current limits
        var currentLimits = motorConfig.CurrentLimits;
        currentLimits.StatorCurrentLimit = WristConstants.STATOR_CURRENT_LIMIT;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_CURRENT_LIMIT;
        currentLimits.SupplyCurrentLimitEnable = true;

        // Motor output configuration
        var motorOutput = motorConfig.MotorOutput;
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Feedback configuration
        var feedback = motorConfig.Feedback;
        feedback.SensorToMechanismRatio = WristConstants.GEAR_RATIO * WristConstants.ENCODER_TO_MECHANISM_RATIO;
        
        // Soft limits
        var softLimits = motorConfig.SoftwareLimitSwitch;
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = WristConstants.MAX_POSITION;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = WristConstants.MIN_POSITION;

        // PID configuration
        var slot0 = motorConfig.Slot0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.kP = WristConstants.Gains.kP;
        slot0.kI = WristConstants.Gains.kI;
        slot0.kD = WristConstants.Gains.kD;
        slot0.kG = WristConstants.Gains.kG;
        slot0.kV = WristConstants.Gains.kV;
        slot0.kS = WristConstants.Gains.kS;
        slot0.kA = WristConstants.Gains.kA;

        wristMotor.getConfigurator().apply(motorConfig);
    }

    // Command factory methods
    public Command goToLoadingPosition() {
        return run(() -> {
            setPosition(WristConstants.Positions.LOAD_CORAL);
            currentPositionName = "Loading";
        }).until(this::atPosition)
          .withName("Wrist To Loading");
    }

    public Command goToScoreL1() {
        return run(() -> {
            setPosition(WristConstants.Positions.L1);
            currentPositionName = "Score L1";
        }).until(this::atPosition)
          .withName("Wrist To L1");
    }

    public Command goToScoreL2() {
        return run(() -> {
            setPosition(WristConstants.Positions.L2);
            currentPositionName = "Score L2";
        }).until(this::atPosition)
          .withName("Wrist To L2");
    }

    public Command goToScoreL3() {
        return run(() -> {
            setPosition(WristConstants.Positions.L3);
            currentPositionName = "Score L3";
        }).until(this::atPosition)
          .withName("Wrist To L3");
    }

    public Command goToScoreL4() {
        return run(() -> {
            setPosition(WristConstants.Positions.L4);
            currentPositionName = "Score L4";
        }).until(this::atPosition)
          .withName("Wrist To L4");
    }

    // Basic control methods
    public void setPosition(double targetPositionRotations) {
        targetPosition = targetPositionRotations;
        wristMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    public double getPosition() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition() {
        return Math.abs(wristMotor.getClosedLoopError().getValue()) < WristConstants.POSE_TOLERANCE;
    }

    public void stop() {
        wristMotor.stopMotor();
    }

    public void setWristZero() {
        wristMotor.setPosition(0);
        System.out.println("Wrist encoder zeroed");
        SmartDashboard.putBoolean("Wrist/WasZeroed", true);
    }

    @Override
    public void periodic() {
        // Basic position telemetry
        SmartDashboard.putString("Wrist/CurrentPosition", currentPositionName);
        SmartDashboard.putNumber("Wrist/CurrentRotations", getPosition());
        SmartDashboard.putNumber("Wrist/TargetRotations", targetPosition);
        SmartDashboard.putBoolean("Wrist/AtPosition", atPosition());
        
        // Diagnostic data
        // SmartDashboard.putNumber("Wrist/StatorCurrent", wristMotor.getStatorCurrent().getValue());
        // SmartDashboard.putNumber("Wrist/SupplyCurrent", wristMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Wrist/ClosedLoopError", wristMotor.getClosedLoopError().getValue());
    }
}