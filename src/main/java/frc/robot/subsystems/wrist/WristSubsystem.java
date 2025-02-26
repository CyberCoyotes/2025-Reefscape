package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final MotionMagicVoltage motionMagic;

    // Configuration constants
    private static final double GEAR_RATIO = 80.0;
    private static final double REVERSE_LIMIT = 0.0; // In rotations
    private static final double FORWARD_LIMIT = 20.0; // In rotations
    private static final double INCREMENT = 0.10; // 0.5 rotations per button press
    private static final double POSE_TOLERANCE = 0.02; // TODO Implement 0.02 rotations tolerance

    // Motion Magic Constants (from config file)
    private static final double kP = 8.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.0;
    private static final double kS = 0.0;
    private static final double kG = 0.05;

    // Motion Profile Constraints (from config file)
    private static final double MOTION_MAGIC_VELOCITY = 80.0; // rotations per second // TODO Bump up?
    private static final double MOTION_MAGIC_ACCELERATION = 80.0; // rotations per second squared // TODO Bump up?
    private static final double MOTION_MAGIC_JERK = 300.0; // rotations per second cubed

    private double targetPosition = 0.0;
    private boolean hasBeenReset = false;

    public WristSubsystem() {
        wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
        motionMagic = new MotionMagicVoltage(0).withSlot(0);

        configureMotor();
        resetWrist();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure Motion Magic and PID
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kS = kS;
        config.Slot0.kG = kG;
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        // Configure soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_LIMIT;

        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Set brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configuration
        wristMotor.getConfigurator().apply(config);
    }

    // Preset positions enum
    public static enum WristPositions {
        STOWED(0.0), // Stowed position from config
        L2(2.15), // L2 scoring position from your value
        L3(2.15), // L3 scoring position from your value
        L4(14.0), // Estimated
        PICK_ALGAE(14.0), // Estimated 8 which was about horitzontal
        SCORE_ALGAE(18.0); // Estimated about 19 max with Phoenix Tuner

        private final double rotations;

        WristPositions(double rotations) {
            this.rotations = rotations;
        }

        public double getRotations() {
            return rotations;
        }
    }


    public void resetWrist() {
        wristMotor.setPosition(0.0);
        targetPosition = 0.0;
        hasBeenReset = true;
    }

    public double getPosition() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public void setPosition(double targetRotations) {
        targetPosition = targetRotations;
        wristMotor.setControl(motionMagic.withPosition(targetRotations));
    }

    public void incrementOut() {
        // Using CTRE Forward Limits already
        setPosition(getPosition() + INCREMENT);
    }

    public void incrementIn() {
        // Using CTRE Reverse Limits already
        setPosition(getPosition() - INCREMENT);
    }

    public boolean atPosition(double targetRotations, double toleranceRotations) {
        return Math.abs(getPosition() - targetRotations) <= toleranceRotations;
    }

    // Check if the wrist is in a safe position for the elevator to move
    public boolean inSafePosition() {
        return getPosition() >= WristPositions.L2.getRotations();
    }


    @Override
    public void periodic() {
        // update inSafePosition
        boolean safeForElevator = inSafePosition();

        // Update SmartDashboard
        SmartDashboard.putNumber("Wrist/Position (rot)", getPosition());
        SmartDashboard.putNumber("Wrist/Target (rot)", targetPosition);
        SmartDashboard.putBoolean("Wrist/Reset Status", hasBeenReset);
        SmartDashboard.putNumber("Wrist/Error (rot)", Math.abs(getPosition() - targetPosition));

        // For Tuning
        SmartDashboard.putNumber("Wrist/kP", kP);
        SmartDashboard.putNumber("Wrist/kI", kI);
        SmartDashboard.putNumber("Wrist/kD", kD);
        SmartDashboard.putNumber("Wrist/kV", kV);
        SmartDashboard.putNumber("Wrist/kS", kS);
        SmartDashboard.putNumber("Wrist/kG", kG);
        SmartDashboard.putNumber("Wrist/Motion Magic Velocity", MOTION_MAGIC_VELOCITY);
        SmartDashboard.putNumber("Wrist/Motion Magic Acceleration", MOTION_MAGIC_ACCELERATION);
        SmartDashboard.putNumber("Wrist/Motion Magic Jerk", MOTION_MAGIC_JERK);
        SmartDashboard.putNumber("Wrist/ForwardLimit", FORWARD_LIMIT);
        
        // For Telemetry logging
        Logger.recordOutput("Wrist/Position", getPosition());
        Logger.recordOutput("Wrist/Target", targetPosition);
        Logger.recordOutput("Wrist/Reset", hasBeenReset);
        Logger.recordOutput("Wrist Error (deg)", Math.abs(getPosition() - targetPosition));
        Logger.recordOutput("Wrist/Voltage", wristMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Wrist/Position", wristMotor.getPosition().getValue());
        Logger.recordOutput("Wrist/Stator Current", wristMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Wrist/Safe for Elevator",safeForElevator);
    }

}