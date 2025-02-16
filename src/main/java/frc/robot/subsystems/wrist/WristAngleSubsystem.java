package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.Angle;
import com.ctre.phoenix6.signals.Position;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

    // Hardware
    private final TalonFX motor;
    private final CANcoder encoder;
    private final MotionMagicVoltage motionMagicRequest;

    // This typed StatusSignal represents the angle read from the CANCoder.
    private final StatusSignal<Angle> canCoderAngle;

    // State tracking
    private double targetPosition = WristConstants.LOAD_CORAL;
    private boolean isTestMode = false;


    public enum AnglePosition {
        LOAD_CORAL(0.00),
        VERTICAL(0.16),
        HORIZONTAL(0.26),
        GRAB_ALGAE(0.45),
        SCORE_L1(0.05),
        SCORE_L2(0.20),
        SCORE_L3(0.30),
        SCORE_L4(0.40);

        public final Angle angle;

        Position(Angle angle) {
            this.angle = angle;
        }
    }

    public WristSubsystem() {
        motor = new TalonFX(WristConstants.WRIST_ID, WristConstants.kCANBUS);
        encoder = new CANcoder(WristConstants.WRIST_CANCODER_ID, WristConstants.kCANBUS);
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

        configureHardware();
        setDefaultCommand(createMaintainPositionCommand());
    }

    private void configureHardware() {
        // Configure CANcoder
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(WristConstants.MAGNET_OFFSET);
        encoder.getConfigurator().apply(encoderConfig);

        // Configure TalonFX
        var motorConfig = new TalonFXConfiguration();

        // Feedback settings
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = WristConstants.GEAR_RATIO;

        // Motor output settings
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motion Magic settings
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.CRUISE_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = WristConstants.ACCELERATION;
        motorConfig.MotionMagic.MotionMagicJerk = WristConstants.JERK;

        // PID Gains - Normal Mode (Slot 0)
        motorConfig.Slot0.kP = WristConstants.Gains0.kP;
        motorConfig.Slot0.kI = WristConstants.Gains0.kI;
        motorConfig.Slot0.kD = WristConstants.Gains0.kD;
        motorConfig.Slot0.kG = WristConstants.Gains0.kG;
        motorConfig.Slot0.kS = WristConstants.Gains0.kS;
        motorConfig.Slot0.kV = WristConstants.Gains0.kV;
        motorConfig.Slot0.kA = WristConstants.Gains0.kA;

        // Apply configuration
        motor.getConfigurator().apply(motorConfig);
    }

    // Command Factories

    /**
     * Creates a command that sets the wrist to a predefined position.
     */
    public Command createMoveToPositionCommand(Position position) {
        return this.runOnce(() -> setPosition(position.rotations))
                .withName("Move To " + position.name());
    }

    /**
     * Creates a command that incrementally adjusts the wrist position.
     */
    public Command createIncrementPositionCommand(double increment) {
        return this.runOnce(() -> {
            double newTarget = constrainPosition(targetPosition + increment);
            setPosition(newTarget);
        }).withName("Increment Position");
    }

    /**
     * Creates the default command that maintains the target position.
     */
    private Command createMaintainPositionCommand() {
        return this.run(() -> motor.setControl(motionMagicRequest.withPosition(targetPosition)))
                .withName("Maintain Position");
    }

    // Public Methods

    /**
     * Sets the wrist position in rotations.
     */
    public void setPosition(double rotations) {
        targetPosition = constrainPosition(rotations);
    }

    /**
     * Gets the current position in rotations.
     */
    public Angle getRawCANCoderValue() {
        return encoder.getPosition().getValue();
    }

    /**
     * Converts that typed Angle into a double (in degrees).
     */
    public double getPosition() {

        return getRawCANCoderValue().getValueAsDouble();
    }

    /**
     * Checks if the wrist is at its target position.
     */

    public boolean atTargetPosition() {
        double tolerance = isTestMode ? WristConstants.POSITION_TOLERANCE_TEST : WristConstants.POSITION_TOLERANCE;
        return Math.abs(getPosition(tolerance) - targetPosition) < tolerance;
    }

    // Helper Methods

    /**
     * Constrains a position to be within the valid range.
     */
    private double constrainPosition(double position) {
        return Math.min(Math.max(position,
                WristConstants.REVERSE_LIMIT),
                WristConstants.FORWARD_LIMIT);
    }



     @Override
    public void periodic() {
        // Refresh the velocity signal in the subsystem's periodic method to ensure it's up to date.
        angleSignal.refresh();

        // Example usage: read the typed AngularVelocity
        AngularVelocity typedVel = velocitySignal.getValue();
        // Or retrieve a double in canonical units (often rotations-per-second or degrees-per-second):
        double velAsDouble = velocitySignal.getValueAsDouble();

        // Do something with that data, e.g. logging, feed it into a custom control loop, etc.
        // ...
    }
}