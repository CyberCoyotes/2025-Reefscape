package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class ClimberSubsystem extends SubsystemBase {

    // Motor & control request object
    private final TalonFX climbMotor;
    private final VoltageOut voltageRequest;
    private final PositionVoltage positionRequest;

    
    // REV Smart Servo
    private final Servo climbServo;
    private boolean servoExtended = true;

    public ClimberSubsystem() {
        // Initialize motor
        climbMotor = new TalonFX(Constants.CLIMBER_ID, Constants.kCANBus);

        // Create a Voltage control request initially set to 0 V
        voltageRequest = new VoltageOut(0);
        
        positionRequest = new PositionVoltage(0).withSlot(0);

        // Initialize servo on PWM port
        climbServo = new Servo(ClimbConstants.SERVO_PWM_CHANNEL);
        
        // Set initial servo position to retracted
        climbServo.setAngle(ClimbConstants.SERVO_MIN_ANGLE);
        climbServo.setAngle(ClimbConstants.SERVO_MAX_ANGLE);

    
        // configureClimberMotor();

        }

        public void configureClimberMotor() {
            
            SoftwareLimitSwitchConfigs softLimitConfig = new SoftwareLimitSwitchConfigs();
            softLimitConfig.ForwardSoftLimitEnable = false; // TODO Test before using
            // softLimitConfig.ForwardSoftLimitThreshold = ClimbConstants.FORWARD_SOFT_LIMIT;
            softLimitConfig.ReverseSoftLimitEnable = false; // TODO Test before using
            // softLimitConfig.ReverseSoftLimitThreshold = ClimbConstants.REVERSE_SOFT_LIMIT;
            climbMotor.getConfigurator().apply(softLimitConfig);
        }

    // Output to lift the robot up
    public void climbUp() {
        climbMotor.setControl(voltageRequest.withOutput(-ClimbConstants.CLIMB_VOLTAGE));
    }

    // Output to lower the robot down
    public void climbDown() {
        climbMotor.setControl(voltageRequest.withOutput(ClimbConstants.CLIMB_VOLTAGE));
    }

    public void stopClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.0));
    }

    public void holdClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.5));
    }
    
    public void setServoAngle(double angle) {
        climbServo.setAngle(angle);
        servoExtended = (angle >= 45.0);
    }
    
    public double getServoAngle() {
        return climbServo.getAngle();
    }
    
    public void rotateServoRange() {
        if (servoExtended) {
            climbServo.setAngle(ClimbConstants.SERVO_MIN_ANGLE);
        } else {
            climbServo.setAngle(ClimbConstants.SERVO_MAX_ANGLE);
        }
        servoExtended = !servoExtended;
    }

    @Override
    public void periodic() {

        /*
        // Add these debug outputs
        SmartDashboard.putNumber("Servo/Angle", climbServo.getAngle());
        SmartDashboard.putBoolean("Servo/Extended", servoExtended);
        SmartDashboard.putNumber("Servo/Position", climbServo.getPosition());  // 0-1 range
        
        // Try to diagnose if servo commands are being sent
        SmartDashboard.putNumber("Servo/RequestedAngle", climbServo.getAngle());
        
        // Send position to Logger
        Logger.recordOutput("Climber/MotorPosition", climbMotor.getPosition().getValue());
        // Get the power output of the motor
        Logger.recordOutput("Climber/MotorVoltage", climbMotor.getSupplyVoltage().getValue());
        // Log servo position
        Logger.recordOutput("Climber/ServoAngle", climbServo.getAngle());
        Logger.recordOutput("Climber/ServoExtended", servoExtended);
         */
    }
}