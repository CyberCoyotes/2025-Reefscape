package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
        // Initialize motor with device ID and CAN bus name
        elevatorLeadMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_ID, "rio"); // CORRECT
        elevatorFollowMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_ID, "rio"); // CORRECT

        // Create control requests
        targetVoltage = new VoltageOut(0);
        targetPosition = new MotionMagicVoltage(0); // Early version suggested new PositionVoltage()

        // Configure the elevator motor
        configureMotor();
    }

    private void configureMotor() {
        var config = new TalonFXConfiguration();

        // Configure current limits to prevent brownouts
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Set motor to brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
        motionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagic.CRUISE_VELOCITY; // Adjust based on your needs
        motionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagic.ACCELERATION; // Adjust based on your needs
        motionMagic.MotionMagicJerk = ElevatorConstants.MotionMagic.JERK; // Optional, for smoother motion


        // Apply the configuration
        elevatorLeadMotor.getConfigurator().apply(config);
        // Configure follower motor
        elevatorFollowMotor.getConfigurator().apply(config);

        // Set up follower motor to follow the lead motor
        // false parameter means it should not invert the leader's signal
        elevatorFollowMotor.setControl(new com.ctre.phoenix6.controls.Follower(
                elevatorLeadMotor.getDeviceID(), true));

        // var motionMagic = config.MotionMagic;
        // motionMagic.MotionMagicCruiseVelocity = 40; //
        // ElevatorConstants.MotionMagic.CRUISE_VELOCITY;
        // motionMagic.MotionMagicAcceleration = 40; //
        // ElevatorConstants.MotionMagic.ACCELERATION;
        // motionMagic.MotionMagicJerk = 0 // 400 // ElevatorConstants.MotionMagic.JERK;

    }

    public void manualControl(double speed) {
        // Apply a speed limit for safety
        speed = Math.max(-0.1, Math.min(0.1, speed));

        // Only need to control the lead motor since the follower will match
        elevatorLeadMotor.setControl(targetVoltage.withOutput(speed * 12.0));
        // Follower will automatically follow the lead motor
    }

    // Command factory methods
    public Command manualVoltage(double speed) {
        return run(() -> {
            manualControl(speed);
        });
    }

    // Position control methods
    public void setPosition(double position) {
        elevatorLeadMotor.setControl(targetPosition.withPosition(position));
    }

    // Command factory methods
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

    public Command moveToTestPose() {
        return moveToPreset(1); // TODO Test this position
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
        // Don't need to reset follower as it's following the leader
    }

    public void stop() {
        elevatorLeadMotor.setControl(targetVoltage.withOutput(0));
        elevatorFollowMotor.setControl(targetVoltage.withOutput(0));
    }

    @Override
    public void periodic() {
       // Log data
    SmartDashboard.putNumber("Elevator/Position", getPosition());
    SmartDashboard.putNumber("Elevator/Target Position", targetPosition.Position);
    SmartDashboard.putNumber("Elevator/Lead Current", elevatorLeadMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Follow Current", elevatorFollowMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator/At Position", isAtPosition(targetPosition.Position));
    
    // Clear control request when disabled
    if (!DriverStation.isEnabled()) {
        stop();
    }
    }
}