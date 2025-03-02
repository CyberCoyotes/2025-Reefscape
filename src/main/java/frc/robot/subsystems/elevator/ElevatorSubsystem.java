package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    // Subsystem Modes
    public enum ElevatorMode {
        PERFORMANCE, // High-speed, competition mode
        SAFETY // Reduced speed, testing/practice mode
    }

    public enum ElevatorPosition {
        HOME(0.00),
        L1(0.00),
        ScoreAlgae(0.50), // TODO Test
        L2(0.90),
        Algae2(1.1), // TODO Test
        L3(2.27),
        Algae3(2.45), // TODO Test
        L4( 4.66);

        private final double position;

        ElevatorPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    // Hardware
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    // Control Requests
    private MotionMagicVoltage motionMagicRequest;

    // Mode Tracking
    private ElevatorMode currentMode = ElevatorMode.SAFETY; // Default to safety mode
    private double targetPosition = 0.0;
    // public WristSubsystem wristSub;

    // Increment Settings
    public static final double INCREMENT = 0.02; // Small adjustments

    // Tolerance/Deadband Settings
    private static final double POSITION_TOLERANCE = 0.02;
    private static final double DEADBAND = 0.02;

    private final Slot1Configs safetyGains = new Slot1Configs()
            .withKP(1.0)
            .withKI(0.01)
            .withKD(0.10)
            .withKS(0.25)
            .withKV(0.12)
            .withKG(0.25)
            .withGravityType(GravityTypeValue.Elevator_Static);

    private final Slot2Configs incrementalGains = new Slot2Configs()
            .withKP(1.0)
            .withKI(0.01)
            .withKD(0.10)
            .withKS(0.25)
            .withKV(0.12)
            .withKG(0.12)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public ElevatorSubsystem() {
        // Initialize motors
        elevatorLeader = new TalonFX(Constants.ELEVATOR_LEAD_ID, Constants.kCANBus);
        elevatorFollower = new TalonFX(Constants.ELEVATOR_FOLLOW_ID, Constants.kCANBus);
        // this.wristSub = wristSub;

        // Initialize control requests
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        // Reset the elevator position
        resetElevator();

        // Configure motors
        configureMotors();
        // configureSafetyMotors();
    }

    private void configureMotors() {
        // Create configurations for both motors
        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        leadConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Configure leader motor
        leadConfig.Slot0.kP = ElevatorConstants.kP;
        leadConfig.Slot0.kI = ElevatorConstants.kI;
        leadConfig.Slot0.kD = ElevatorConstants.kD;
        leadConfig.Slot0.kV = ElevatorConstants.kV;
        leadConfig.Slot0.kS = ElevatorConstants.kS;
        leadConfig.Slot0.kG = ElevatorConstants.kG;

        // Configure motion magic
        leadConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leadConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.CRUISE_VELOCITY;
        leadConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK;

        // Configure soft limits
        leadConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        leadConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.FORWARD_LIMIT;
        leadConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leadConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.REVERSE_LIMIT;

        // Inside configureMotors() method, add to both leadConfig and followerConfig:
        leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leadConfig.CurrentLimits.StatorCurrentLimit = 40; // Adjust value based on

        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = 40;

        // Apply configurations
        elevatorLeader.getConfigurator().apply(leadConfig);

        // Configure follower motor to oppose the leader
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorFollower.getConfigurator().apply(followerConfig);

        // Set up follower to follow leader with opposite direction
        elevatorFollower.setControl(new Follower(Constants.ELEVATOR_LEAD_ID, true));
    }

    public void resetElevator() {
        elevatorLeader.setPosition(0);
    }

    public void setPosition(double positionRotations) {
        targetPosition = positionRotations;

        // Only move if change is larger than deadband
        if (Math.abs(targetPosition - getPosition()) > DEADBAND) {
            elevatorLeader.setControl(motionMagicRequest.withPosition(targetPosition * ElevatorConstants.GEAR_RATIO));
        }

        elevatorLeader.setControl(motionMagicRequest.withPosition(positionRotations * ElevatorConstants.GEAR_RATIO));
    }

    public void incrementPosition(boolean up) {
        double currentPos = getPosition();
        double increment = up ? INCREMENT : -INCREMENT;

        // Use slot 2 for incremental movement
        motionMagicRequest = motionMagicRequest.withSlot(2);

        // TODO Make sure limits are respected in motor configs
        double newTarget = (currentPos + increment);

        setPosition(newTarget);

        // Reset back to current mode's slot for next movement
        motionMagicRequest = motionMagicRequest.withSlot(0);
    }

    // This is required for the incremental command 
    public void incrementPosition(double increment) {
        double currentPos = getPosition();
        double newPos = currentPos + increment;
        setPosition(newPos);
    }

    public double getPosition() {
        return elevatorLeader.getPosition().getValueAsDouble() / ElevatorConstants.GEAR_RATIO;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    /*********************************
     * Command Factories
     ********************************/

    // TODO Test these original commands
    public Command setPositionCommand(double position) {
        return run(() -> setPosition(position))
                .withName("SetElevatorPosition");
    }

    public Command incrementUp() {
        return run(() -> incrementPosition(INCREMENT))
                .withName("IncrementElevatorUp");
    }

    public Command incrementDown() {
        return run(() -> incrementPosition(-INCREMENT))
                .withName("IncrementElevatorDown");
    }

    @Override
    public void periodic() {
        // Mode and State Information
        Logger.recordOutput("Elevator/Mode", currentMode.toString());
        Logger.recordOutput("Elevator/Position/Current", getPosition());
        Logger.recordOutput("Elevator/Position/Target", targetPosition);
        Logger.recordOutput("Elevator/AtTarget", isAtPosition(targetPosition));
        // Logger.recordOutput("Elevator/Wrist/SafePose", ()-> wrist.inSafePosition());
        // Motor Telemetry
        Logger.recordOutput("Elevator/Voltage", elevatorLeader.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/Current", elevatorLeader.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Elevator/Velocity",
                elevatorLeader.getVelocity().getValueAsDouble() / ElevatorConstants.GEAR_RATIO);

        // Log the applied motor voltage
        Logger.recordOutput("Elevator/AppliedVoltage", elevatorLeader.getMotorVoltage().getValueAsDouble());

        // Log the motor current draw
        Logger.recordOutput("Elevator/MotorCurrent", elevatorLeader.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Elevator/StatorCurrent", elevatorLeader.getStatorCurrent().getValueAsDouble());

        // Log the target position for debugging
        // Logger.recordOutput("Elevator/TargetPosition", isAtPosition();
    }
}
