package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignal.*;

// import com.ctre.phoenix6.signals.StatusSignal;
// import com.ctre.phoenix6.measuredvalue.Rotations;
// import com.ctre.phoenix6.measuredvalue.RotationalVelocity;
// import com.ctre.phoenix6.measuredvalue.Current;
// import com.ctre.phoenix6.measuredvalue.Temperature;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration; 

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    // Constants
    public static final int CLIMB_ID = 50;
    public static final String CANBUS = "rio";
    private static final double GEAR_RATIO = 125.0; // 125:1 reduction
    private static final double SPOOL_DIAMETER_METERS = 0.0508; // 2 inch spool diameter
    private static final double METERS_PER_ROTATION = Math.PI * SPOOL_DIAMETER_METERS / GEAR_RATIO;
    private static final double MAX_EXTENSION_METERS = 0.5; // Example: 0.5 meters max extension
    private static final double MIN_EXTENSION_METERS = 0.0;

    // Motor and control request objects
    private final TalonFX climbMotor;
    private final PositionVoltage positionRequest;
    private final VoltageOut percentRequest;

    private final StatusSignal<rot> motorPosition;  // For getRotorPosition() 
    private final StatusSignal<> motorVelocity;  // For getRotorVelocity()
    private final StatusSignal<> motorCurrent;   // For getStatorCurrent()
    private final StatusSignal<> motorTemperature; // For getDeviceTemp()
    


    public ClimbSubsystem() {
        // Initialize motor with device ID and CAN bus name
        climbMotor = new TalonFX(CLIMB_ID, CANBUS); // Adjust ID as needed

        // Initialize control requests
        positionRequest = new PositionVoltage(0).withSlot(0);
        percentRequest = new VoltageOut(0);

        // Configure motor
        configureMotor();

        // Initialize status signals
        motorPosition = climbMotor.getRotorPosition();
        motorVelocity = climbMotor.getRotorVelocity();
        motorCurrent = climbMotor.getStatorCurrent();
        motorTemperature = climbMotor.getDeviceTemp();

        // Optimize signals for logging
        motorPosition.setUpdateFrequency(50);
        motorVelocity.setUpdateFrequency(50);
        motorCurrent.setUpdateFrequency(10);
        motorTemperature.setUpdateFrequency(1);
    }

    private void configureMotor() {
        // Motor output configuration
        var motorConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        // Current limits
        var currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true);

        // Position PID gains
        var slot0 = new Slot0Configs()
                .withKP(5.0)
                .withKI(0.0)
                .withKD(0.1)
                .withKS(0.2) // Static friction compensation
                .withKV(0.12); // Velocity feedforward

        // Software limits (convert meters to rotations)
        var softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(metersToRotations(MAX_EXTENSION_METERS))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(metersToRotations(MIN_EXTENSION_METERS));

        // Apply configurations
        var configurator = climbMotor.getConfigurator();
        configurator.apply(motorConfig);
        configurator.apply(currentLimits);
        configurator.apply(slot0);
        configurator.apply(softLimits);
    }

    private double metersToRotations(double meters) {
        return meters / METERS_PER_ROTATION;
    }

    private double rotationsToMeters(Double rotations) {
        return rotations * METERS_PER_ROTATION;
    }

    // Command Factories
    public Command setPosition(double targetMeters) {
        return run(() -> {
            double targetRotations = metersToRotations(targetMeters);
            climbMotor.setControl(positionRequest.withPosition(targetRotations));
        }).until(() -> isAtPosition(targetMeters))
                .withName("Climb to " + targetMeters + "m");
    }

    public Command manualControl(double percentOutput) {
        return run(() -> {
            // Checks soft limits before applying manual control
            if ((percentOutput > 0 && !isAtUpperLimit()) ||
                    (percentOutput < 0 && !isAtLowerLimit())) {
                climbMotor.setControl(percentRequest.withOutput(percentOutput * 12.0));
            } else {
                climbMotor.setControl(percentRequest.withOutput(0));
            }
        }).withName("Manual Climb Control");
    }

    public Command stopClimb() {
        return runOnce(() -> climbMotor.setControl(percentRequest.withOutput(0)))
                .withName("Stop Climb");
    }

    // Emergency recover/retract command
    public Command emergencyRetract() {
        return setPosition(MIN_EXTENSION_METERS)
                .withName("Emergency Retract");
    }
// Update helper methods to handle new types
public boolean isAtPosition(double targetMeters) {
    double currentMeters = rotationsToMeters(motorPosition.getRotorPosition());
    return Math.abs(currentMeters - targetMeters) < 0.02;
}

public boolean isAtUpperLimit() {
    return rotationsToMeters(motorPosition.getValue().getRotations()) >= MAX_EXTENSION_METERS;
}

public boolean isAtLowerLimit() {
    return rotationsToMeters(motorPosition.getValue().getRotations()) <= MIN_EXTENSION_METERS;
}

public double getCurrentPosition() {
    return rotationsToMeters(motorPosition.getValue().getRotations());
}

private double rotationsToMeters(double rotations) {
    return rotations * METERS_PER_ROTATION;
}


    @Override
    public void periodic() {
        // Log data using Logger
        Logger.recordOutput("Climb/Position_Meters", getCurrentPosition());
        Logger.recordOutput("Climb/Velocity_MPS", motorVelocity.getValue() * METERS_PER_ROTATION);
        Logger.recordOutput("Climb/Current_Amps", motorCurrent.getValue());
        Logger.recordOutput("Climb/Temperature_C", motorTemperature.getValue());
        Logger.recordOutput("Climb/AtUpperLimit", isAtUpperLimit());
        Logger.recordOutput("Climb/AtLowerLimit", isAtLowerLimit());
    }

}