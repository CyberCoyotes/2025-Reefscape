package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import java.util.logging.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    /*
     * ElevatorLeft CAN ID: 24
     * ElevatorRight CAN ID: 23
     */
    private final TalonFX elevatorLeadMotor; // Currently the Left
    private final TalonFX elevatorFollowMotor; // Currently the Right
    private final VoltageOut targetVoltage; // For manual control
    private final MotionMagicVoltage targetPosition; // For position control

    public ElevatorSubsystem() {
        // Initialize motors
        elevatorLeadMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_ID, "rio");
        elevatorFollowMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_ID, "rio");

        // Create control requests
        targetVoltage = new VoltageOut(0);
        targetPosition = new MotionMagicVoltage(0);

        // Configure motors
        resetEncoders();
        configureMotor();

        // Set both motors to brake mode for better position holding
        elevatorLeadMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorFollowMotor.setNeutralMode(NeutralModeValue.Brake);

        // Set motor inversions
        elevatorLeadMotor.setInverted(false);
        elevatorFollowMotor.setInverted(true);

    } // End of ElevatorSubsystem() constructor

    private void configureMotor() {
        var config = new TalonFXConfiguration();

        // Configure current limits to prevent brownouts
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Set motor to brake mode
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Soft limits
        var softLimits = config.SoftwareLimitSwitch;
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = ElevatorConstants.MAX_POSITION;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = ElevatorConstants.MIN_POSITION;

        // Configure voltage compensation
        config.Voltage.PeakForwardVoltage = 11.5;
        config.Voltage.PeakReverseVoltage = -11.5;

        // Configure closed-loop gains for position control
        var slot0 = config.Slot0;
        slot0.kP = ElevatorConstants.Gains.kP; // Adjust these gains for your elevator
        slot0.kI = ElevatorConstants.Gains.kI;
        slot0.kD = ElevatorConstants.Gains.kD;
        slot0.kS = ElevatorConstants.Gains.kS; // Adjust for static friction
        slot0.kV = ElevatorConstants.Gains.kV; // Adjust for velocity feedforward
        slot0.kA = ElevatorConstants.Gains.kA; // Adjust for acceleration feedforward
        slot0.kG = ElevatorConstants.Gains.kG; // Adjust for gravity compensation

        // Add these lines in configureMotor()
        var motionMagic = config.MotionMagic;

        // TODO Adjust cruise as needed
        motionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagic.CRUISE_VELOCITY;

        // TODO adjust acceleration as needed
        motionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagic.ACCELERATION;

        // TODO Optional adjustment
        motionMagic.MotionMagicJerk = ElevatorConstants.MotionMagic.JERK;

        // Apply the configuration to both motors
        elevatorLeadMotor.getConfigurator().apply(config);
        elevatorFollowMotor.getConfigurator().apply(config);

        // Set up follower motor with oppositeLeader=true since we want it to run
        // opposite
        elevatorFollowMotor.setControl(
                new com.ctre.phoenix6.controls.Follower(elevatorLeadMotor.getDeviceID(), true));

                System.out.println("Follower setup complete - Following device ID: " + elevatorLeadMotor.getDeviceID());

    }

    // Write a method to move the elevator by voltage speed
    public void manualControl(double speed) {
        double voltage = speed * 12.0; // Convert speed to voltage

        elevatorLeadMotor.setControl(targetVoltage.withOutput(voltage));
        elevatorFollowMotor.setControl(targetVoltage.withOutput(voltage));
        // elevatorLeadMotor.setControl(targetVoltage.withOutput(speed));
        // elevatorFollowMotor.setControl(targetVoltage.withOutput(speed));
    }

    // Position control methods
    public void setPosition(double position) {
        // elevatorFollowMotor.setControl(targetPosition.withPosition(position));

        System.out.println("Setting position to: " + position);
        elevatorLeadMotor.setControl(targetPosition.withPosition(position));
    }

    public boolean isAtPosition(double targetPosition) {
        double currentPosition = getPosition();
        return Math.abs(currentPosition - targetPosition) <= ElevatorConstants.POSITION_TOLERANCE;
    }

    public double getPosition() {
        return elevatorLeadMotor.getPosition().getValueAsDouble(); // Changed to getValueAsDouble() from getValue()
    }

    public void resetEncoders() {
        elevatorLeadMotor.setPosition(0);
        elevatorFollowMotor.setPosition(0);
    }

    public void stop() {
        elevatorLeadMotor.setControl(targetVoltage.withOutput(0));
        elevatorFollowMotor.setControl(targetVoltage.withOutput(0));
    }

    /*********************************
     * Command factory methods
     *******************************/

         // Command factory methods
    // Command factory methods
    public Command manualVoltage(double speed) {
        return run(() -> manualControl(speed))
               .withName("Manual Elevator Control")
               .finallyDo((interrupted) -> stop()); // Make sure it stops when command ends
    }

    // And update moveUp/moveDown to use voltage values
public Command moveUp() {
    return run(() -> manualControl(0.3))
           .withName("Move Up")
           .finallyDo((interrupted) -> stop());
}

public Command moveDown() {
    return run(() -> manualControl(-0.2))
           .withName("Move Down")
           .finallyDo((interrupted) -> stop());
}

     public Command moveToPosition(double position) {
        return runOnce(() -> setPosition(position));
    }

    public Command stopElevator() {
        return runOnce(this::stop);
    }

    public Command resetElevatorPosition() {
        return runOnce(this::resetEncoders);
    }

    // Example of a more complex command that waits for completion
    public Command moveToPositionAndWait(double position) {
        return run(() -> setPosition(position))
                .until(() -> isAtPosition(position));
    }

    // Add these command factory methods
    public Command moveToPreset(double presetPosition) {
        return new SequentialCommandGroup(
                // Log start of movement
                runOnce(() -> System.out.println("Moving elevator to position: " + presetPosition)),
                // Move to position and wait
                moveToPositionAndWait(presetPosition),
                // Log completion
                runOnce(() -> System.out.println("Elevator reached position: " + presetPosition)));
    }

    public Command moveToTestPosition() {
        return moveToPositionAndWait(5.0)
                .withName("Move To Test Position");
    }

    public Command moveToHome() {
        return moveToPositionAndWait(0)
                .withName("Move To Home");
    }

    @Override
    public void periodic() {
        // Log data
        SmartDashboard.putNumber("Elevator/Lead Voltage",
                elevatorLeadMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Follow Voltage",
                elevatorFollowMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator/Lead Direction",
                elevatorLeadMotor.getMotorVoltage().getValueAsDouble() > 0);
        SmartDashboard.putBoolean("Elevator/Follow Direction",
                elevatorFollowMotor.getMotorVoltage().getValueAsDouble() > 0);

        SmartDashboard.putNumber("Elevator/Position", getPosition());
        SmartDashboard.putNumber("Elevator/Target Position", targetPosition.Position);
        SmartDashboard.putNumber("Elevator/Lead Current", elevatorLeadMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Follow Current", elevatorFollowMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator/At Position", isAtPosition(targetPosition.Position));

        Logger.getLogger("Elevator").info("Elevator Position: " + getPosition());
        Logger.getLogger("Elevator").info("Elevator Target Position: " + targetPosition.Position);
        Logger.getLogger("Elevator")
                .info("Elevator Lead Current: " + elevatorLeadMotor.getStatorCurrent().getValueAsDouble());
        Logger.getLogger("Elevator")
                .info("Elevator Follow Current: " + elevatorFollowMotor.getStatorCurrent().getValueAsDouble());
        Logger.getLogger("Elevator").info("Elevator At Position: " + isAtPosition(targetPosition.Position));

        // Clear control request when disabled
        if (!DriverStation.isEnabled()) {
            stop();
        }
    }
}

// https://claude.ai/chat/db526c8f-c875-4b99-bf07-fac5c0e1019d

/*
 * 
 * public void testLeadMotor(double voltage) {
 * elevatorFollowMotor.setControl(new VoltageOut(0)); // Disable follower
 * elevatorLeadMotor.setControl(targetVoltage.withOutput(voltage));
 * }
 * 
 * public void testFollowMotor(double voltage) {
 * elevatorLeadMotor.setControl(new VoltageOut(0)); // Disable lead
 * elevatorFollowMotor.setControl(targetVoltage.withOutput(voltage));
 * }
 * 
 * public Command testLeadMotorCommand() {
 * return run(() -> {
 * testLeadMotor(2.0); // Start with 2V
 * })
 * .withName("Test Lead Motor")
 * .until(() -> getPosition() == 4.0) // Stop after 5 rotations
 * .finallyDo((interrupted) -> stop());
 * }
 * 
 * // Command factory methods
 * public Command testFollowMotorCommand() {
 * return run(() -> {
 * testFollowMotor(2.0); // Start with 2V
 * })
 * .withName("Test Follow Motor")
 * .until(() -> getPosition() == 10.0) // Stop after 5 rotations
 * .finallyDo((interrupted) -> stop());
 * }
 */