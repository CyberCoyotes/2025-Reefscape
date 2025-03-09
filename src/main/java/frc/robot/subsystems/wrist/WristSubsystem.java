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

    // Configuration constants
    private static final double GEAR_RATIO = 80.0;
    private static final double REVERSE_LIMIT = 0.0; // In rotations
    private static final double FORWARD_LIMIT = 20.0; // In rotations
    private static final double INCREMENT = 0.50;
    private static final double TOLERANCE = 0.02; // Position Tolerance in rotations
    private static final double VELOCITY = 120.0; // Motion Magic rotations per second
    private static final double ACCELERATION = 120.0; // Motion Magic rotations per second squared
    private static final double JERK = 300.0; // Motion Magic rotations per second cubed

    private static final double kP = 10.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.0;
    private static final double kS = 0.0;
    private static final double kG = 0.05;

    // END of Configuration constants

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

        config.MotionMagic.MotionMagicCruiseVelocity = VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
        config.MotionMagic.MotionMagicJerk = JERK;

        // Configure soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_LIMIT;

        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;

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
        * TODO The new L2 is approx 1.75; needs to be confirmed with a reef test
        * L3 should be the same as L2
        * L4 Approximately 6.0 and found to be near vertical in classroom
        * TODO Confirm L4 scoring pose with a Reef test
        * INTAKE CORAL is a new pose and estimate between L4 and PICK ALGAE 10.0
        * TODO Confirm new INTAKE CORAL pose with a Reef test
        * PICK ALGAE was previously 14.0 
        * TODO Confirm new PICK ALGAE pose with a Reef test
        * SCORE ALGAE was previously 19.0
        * TODO Confirm new SCORE ALGAE pose with a Reef test
        
        */

        START(0.0), 
        STOWED(0.0),
        L1(0.5),
        L2(1.75), 
        L3(1.75),
        L4(4.2), // Verified
        INTAKE_CORAL(10.0),
        PICK_ALGAE(14.0),
        SCORE_ALGAE(18.0);

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
        SmartDashboard.putNumber("Wrist/Motion Magic Velocity", VELOCITY);
        SmartDashboard.putNumber("Wrist/Motion Magic Acceleration", ACCELERATION);
        SmartDashboard.putNumber("Wrist/Motion Magic Jerk", JERK);
        SmartDashboard.putNumber("Wrist/ForwardLimit", FORWARD_LIMIT);
        SmartDashboard.putNumber("Wrist/Increment", INCREMENT);

        /* 
        // For Telemetry logging
        Logger.recordOutput("Wrist/Position", getPosition());
        Logger.recordOutput("Wrist/Target", targetPosition);
        Logger.recordOutput("Wrist/Reset", hasBeenReset);
        Logger.recordOutput("Wrist Error (deg)", Math.abs(getPosition() - targetPosition));
        Logger.recordOutput("Wrist/Voltage", wristMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Wrist/Position", wristMotor.getPosition().getValue());
        Logger.recordOutput("Wrist/Stator Current", wristMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Wrist/Safe for Elevator",safeForElevator);
         */
    }

}