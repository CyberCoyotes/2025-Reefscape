package frc.robot.subsystems.wrist_4;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.units.Angle;
// import edu.wpi.first.units.AngularVelocity;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.sim.PhysicsSim;
import frc.robot.Robot;

public class WristSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX motor;
    private final CANcoder encoder;
    
    // Motion Magic Control Request
    private final MotionMagicVoltage motionMagicRequest;
    
    // Status Signals
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;
    private final StatusSignal<Angle> motorRotorPosition;
   
    // private final StatusSignal<Measure<Angle> encoderPosition;
    // private final StatusSignal<Measure<AngularVelocity> encoderVelocity;
    // private final StatusSignal<Measure<Angle> motorRotorPosition;
   
    // Fault Signals
    private final StatusSignal<Boolean> fusedSensorOutOfSync;
    private final StatusSignal<Boolean> stickyFusedSensorOutOfSync;
    private final StatusSignal<Boolean> remoteSensorInvalid;
    private final StatusSignal<Boolean> stickyRemoteSensorInvalid;
    
    // Constants
    private static final double GEAR_RATIO = 75.0; // Example - adjust for your gearing
    private static final double MOTION_MAGIC_VELOCITY = 10.0; // rotations per second
    private static final double MOTION_MAGIC_ACCELERATION = 20.0; // rotations per second^2
    private static final double MOTION_MAGIC_JERK = 100.0; // rotations per second^3
    private static final double VOLTAGE_FEEDFORWARD = 0.0; // Volts to add to overcome gravity
    
    // Wrist angle limits in rotations
    private static final double MIN_ROTATION = -0.25; // -90 degrees
    private static final double MAX_ROTATION = 0.25;  // 90 degrees

    public WristSubsystem(int motorId, int encoderId, String canBus) {
        motor = new TalonFX(motorId, canBus);
        encoder = new CANcoder(encoderId, canBus);
        
        // Configure CANcoder
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0.0)); // Adjust offset as needed
        
        encoder.getConfigurator().apply(encoderConfig);
        
        // Configure TalonFX
        var motorConfig = new TalonFXConfiguration();
        
        // Feedback Configuration
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
        motorConfig.Feedback.SensorToMechanismRatio = 1.0;
        
        // Motion Magic Configuration
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        motorConfig.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;
        
        // Slot 0 Configuration for Motion Magic
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.kS = 0.25;
        motorConfig.Slot0.kV = 0.12;
        motorConfig.Slot0.kA = 0.01;
        motorConfig.Slot0.kP = 4.8;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.1;
        
        // Current Limits
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Apply configuration
        motor.getConfigurator().apply(motorConfig);
        
        // Set brake mode
        motor.setNeutralMode(NeutralModeValue.Brake);
        
        // Initialize Motion Magic request
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
        
        // Initialize all status signals
        motorPosition = motor.getPosition(false);
        motorVelocity = motor.getVelocity(false);
        encoderPosition = encoder.getPosition(true);
        encoderVelocity = encoder.getVelocity(true);
        motorRotorPosition = motor.getRotorPosition(false);
        
        // Initialize fault signals
        fusedSensorOutOfSync = motor.getFault_FusedSensorOutOfSync();
        stickyFusedSensorOutOfSync = motor.getStickyFault_FusedSensorOutOfSync();
        remoteSensorInvalid = motor.getFault_RemoteSensorDataInvalid();
        stickyRemoteSensorInvalid = motor.getStickyFault_RemoteSensorDataInvalid();

         
        // if (Robot.isSimulation()) {
        //     PhysicsSim.getInstance().addTalonFX(motor, encoder, GEAR_RATIO, 0.001);
        // }

    }
    
    public void setPosition(double targetRotations) {
        targetRotations = MathUtil.clamp(targetRotations, MIN_ROTATION, MAX_ROTATION);
        motor.setControl(motionMagicRequest.withPosition(targetRotations)
                                         .withFeedForward(VOLTAGE_FEEDFORWARD));
    }
    
    public double getPosition() {
        return motorPosition.refresh().getValue().in(Rotations);
    }
    
    public double getVelocity() {
        return motorVelocity.refresh().getValue().in(RotationsPerSecond);
    }
    
    public boolean atTargetPosition(double toleranceRotations) {
        return Math.abs(getPosition() - motionMagicRequest.Position) < toleranceRotations;
    }
    
    @Override
    public void periodic() {
        // Refresh all signals together for efficiency
        BaseStatusSignal.refreshAll(
            fusedSensorOutOfSync,
            stickyFusedSensorOutOfSync,
            remoteSensorInvalid,
            stickyRemoteSensorInvalid,
            motorPosition,
            motorVelocity,
            encoderPosition,
            encoderVelocity
        );
        
        // Check for faults
        boolean anyFault = stickyFusedSensorOutOfSync.getValue() || 
                          stickyRemoteSensorInvalid.getValue();
        
        if (anyFault) {
            if (fusedSensorOutOfSync.getValue()) {
                System.out.println("Wrist: Fused sensor out of sync");
            }
            if (remoteSensorInvalid.getValue()) {
                System.out.println("Wrist: Remote sensor invalid");
            }
        }
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Wrist/Position", getPosition());
        SmartDashboard.putNumber("Wrist/Velocity", getVelocity());
        SmartDashboard.putNumber("Wrist/Target", motionMagicRequest.Position);
        SmartDashboard.putBoolean("Wrist/AtTarget", atTargetPosition(0.02));
        SmartDashboard.putBoolean("Wrist/HasFault", anyFault);
    }
    
    /**
     * Command factory class for WristSubsystem
     */
    public static class CommandFactory {
        private final WristSubsystem wrist;
        
        public CommandFactory(WristSubsystem wrist) {
            this.wrist = wrist;
        }
        
        /**
         * Creates a command to move the wrist to a specific position
         */
        public Command setPosition(double targetRotations) {
            return Commands.run(() -> wrist.setPosition(targetRotations), wrist)
                         .until(() -> wrist.atTargetPosition(0.02))
                         .withName("Wrist To " + targetRotations);
        }

        /**
         * Creates a command to clear sticky faults
         */
        public Command clearFaults() {
            return Commands.runOnce(() -> wrist.motor.clearStickyFaults())
                         .withName("Clear Wrist Faults");
        }
        
        /**
         * Commonly used wrist positions
         */
        public final class Positions {
            public Command stow() {
                return setPosition(0.0).withName("Wrist Stow");
            }
            
            public Command ground() {
                return setPosition(-0.25).withName("Wrist Ground");
            }
            
            public Command shelf() {
                return setPosition(0.125).withName("Wrist Shelf");
            }
        }
        
        public final Positions positions = new Positions();
    }
}