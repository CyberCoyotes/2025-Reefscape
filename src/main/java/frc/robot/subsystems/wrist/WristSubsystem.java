package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")

public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final MotionMagicVoltage motionMagic;

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

        // Configure Motion Magic and PID using constants
        config.Slot0.kP = WristConstants.Slot0.kP;
        config.Slot0.kI = WristConstants.Slot0.kI;
        config.Slot0.kD = WristConstants.Slot0.kD;
        config.Slot0.kV = WristConstants.Slot0.kV;
        config.Slot0.kS = WristConstants.Slot0.kS;
        config.Slot0.kG = WristConstants.Slot0.kG;
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = WristConstants.VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = WristConstants.ACCELERATION;
        config.MotionMagic.MotionMagicJerk = WristConstants.JERK;

        // Configure soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.FORWARD_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.REVERSE_LIMIT;

        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = WristConstants.ENABLE_CURRENT_LIMIT;

        // Set brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configuration
        wristMotor.getConfigurator().apply(config);
    }

    // Preset positions enum
    public static enum WristPositions {
        /* 
        * Physically the end effector of the wrist should be resting against the elevator at the start of the match (START)
        * This position should not be used during a match after the start.
        * The STOWED position is currently not in use, but could be used to stow the wrist for transport as it was with previous configiaration
        * L1 is not a position currently in use, but could be used with testing differet scoring heights
        * L2 was set to 2.15 with the previous configuration
        * L3 should be the same as L2
        * L4 Approximately 6.0 and found to be near vertical in classroom
        * INTAKE CORAL is a new pose and estimate between L4 and PICK ALGAE 10.0
        * PICK ALGAE was previously 14.0 
        * SCORE ALGAE was previously 19.0
        */

        START(WristConstants.Positions.START), 
        STOWED(WristConstants.Positions.STOWED),
        L1(WristConstants.Positions.L1),
        L2(WristConstants.Positions.L2), 
        L3(WristConstants.Positions.L3),
        L4(WristConstants.Positions.L4),
        TRAVEL(WristConstants.Positions.TRAVEL),
        INTAKE_CORAL(WristConstants.Positions.INTAKE_CORAL),
        PICK_ALGAE(WristConstants.Positions.PICK_ALGAE),
        SCORE_ALGAE(WristConstants.Positions.SCORE_ALGAE);

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
        setPosition(getPosition() + WristConstants.INCREMENT);
    }

    public void incrementIn() {
        // Using CTRE Reverse Limits already
        setPosition(getPosition() - WristConstants.INCREMENT);
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

        // For Tuning - Now referencing the constants
        SmartDashboard.putNumber("Wrist/kP", WristConstants.Slot0.kP);
        SmartDashboard.putNumber("Wrist/kI", WristConstants.Slot0.kI);
        SmartDashboard.putNumber("Wrist/kD", WristConstants.Slot0.kD);
        SmartDashboard.putNumber("Wrist/kV", WristConstants.Slot0.kV);
        SmartDashboard.putNumber("Wrist/kS", WristConstants.Slot0.kS);
        SmartDashboard.putNumber("Wrist/kG", WristConstants.Slot0.kG);
        SmartDashboard.putNumber("Wrist/Motion Magic Velocity", WristConstants.VELOCITY);
        SmartDashboard.putNumber("Wrist/Motion Magic Acceleration", WristConstants.ACCELERATION);
        SmartDashboard.putNumber("Wrist/Motion Magic Jerk", WristConstants.JERK);
        SmartDashboard.putNumber("Wrist/ForwardLimit", WristConstants.FORWARD_LIMIT);
        SmartDashboard.putNumber("Wrist/Increment", WristConstants.INCREMENT);

    }
}