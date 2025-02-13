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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystemMotor extends SubsystemBase {
    
        private final TalonFX wristMotor;
        private final MotionMagicVoltage motionMagicRequest;
        
        private String currentPositionName = "Unknown";
        private double targetPosition = 0.0;
    
        public WristSubsystemMotor() {
            wristMotor = new TalonFX(WristConstants_Motor.MOTOR_CAN_ID, "rio");
            motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
            
            configureMotor();
        }
    
        private void configureMotor() {
            var motorConfig = new TalonFXConfiguration();
            
            // Current limits for protection
            var currentLimits = motorConfig.CurrentLimits;
            currentLimits.StatorCurrentLimit = WristConstants.STATOR_CURRENT_LIMIT;
            currentLimits.StatorCurrentLimitEnable = true;
            currentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_CURRENT_LIMIT;
            currentLimits.SupplyCurrentLimitEnable = true;
    
            // Configure motor output behavior
            var motorOutput = motorConfig.MotorOutput;
            motorOutput.NeutralMode = NeutralModeValue.Brake;
            motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
            // Configure feedback from internal sensor
            var feedback = motorConfig.Feedback;
            feedback.SensorToMechanismRatio = WristConstants_Motor.GEAR_RATIO;
    
            // Software limits to prevent mechanism damage
            var softLimits = motorConfig.SoftwareLimitSwitch;
            softLimits.ForwardSoftLimitEnable = true;
            softLimits.ForwardSoftLimitThreshold = WristConstants.MAX_POSITION;
            softLimits.ReverseSoftLimitEnable = true;
            softLimits.ReverseSoftLimitThreshold = WristConstants.MIN_POSITION;
    
            // Motion Magic settings
            var motionMagic = motorConfig.MotionMagic;
            motionMagic.MotionMagicCruiseVelocity = WristConstants_Motor.MotionMagic.CRUISE_VELOCITY; // rps
            motionMagic.MotionMagicAcceleration = WristConstants_Motor.MotionMagic.ACCELERATION; // rps/s
            motionMagic.MotionMagicJerk = WristConstants_Motor.MotionMagic.JERK; // rps/s/s
    
            // PID and Feedforward gains
            var slot0 = motorConfig.Slot0;
            slot0.GravityType = GravityTypeValue.Arm_Cosine;
            slot0.kP = WristConstants.Gains.kP;
            slot0.kI = WristConstants.Gains.kI;
            slot0.kD = WristConstants.Gains.kD;
            slot0.kG = WristConstants.Gains.kG;
            slot0.kV = WristConstants.Gains.kV;
            slot0.kS = WristConstants.Gains.kS;
            slot0.kA = WristConstants.Gains.kA;
    
            // Apply the configuration
            wristMotor.getConfigurator().apply(motorConfig);
        }
    
        /**
         * Sets the target position using Motion Magic
         * @param targetPositionRotations The target position in rotations
         */
        public void setPosition(double targetPositionRotations) {
            targetPosition = targetPositionRotations;
            wristMotor.setControl(motionMagicRequest.withPosition(targetPositionRotations));
        }
    
        /**
         * Gets the current position of the wrist
         * @return Current position in rotations
         */
        public double getPosition() {
            return wristMotor.getPosition().getValueAsDouble();
        }
    
        /**
         * Checks if the wrist is at the target position within tolerance
         * @param tolerance The position tolerance in rotations
         * @return true if at target position
         */
        public boolean atTargetPosition(double tolerance) {
            return Math.abs(getPosition() - targetPosition) <= tolerance;
        }
    
        /**
         * Stops the wrist motor
         */
        public void stop() {
            wristMotor.stopMotor();
        }
    
        /**
         * Sets the current position as the new zero reference
         */
        public void setWristZero() {
            wristMotor.setPosition(0);
            System.out.println("Wrist encoder zeroed");
            SmartDashboard.putBoolean("Wrist/WasZeroed", true);
        }
    
        @Override
        public void periodic() {
            // Update dashboard with current status
            SmartDashboard.putString("Wrist/CurrentPosition", currentPositionName);
            SmartDashboard.putNumber("Wrist/CurrentRotations", getPosition());
            SmartDashboard.putNumber("Wrist/TargetRotations", targetPosition);
            SmartDashboard.putBoolean("Wrist/AtPosition", atTargetPosition(WristConstants_Motor.POSITION_TOLERANCE));
            
            // Motion Magic specific telemetry
            // SmartDashboard.putNumber("Wrist/Velocity", wristMotor.getVelocity().getValue());
            SmartDashboard.putNumber("Wrist/ClosedLoopError", wristMotor.getClosedLoopError().getValue());
            // SmartDashboard.putNumber("Wrist/MotorVoltage", wristMotor.getMotorVoltage().getValue());
        }
    }