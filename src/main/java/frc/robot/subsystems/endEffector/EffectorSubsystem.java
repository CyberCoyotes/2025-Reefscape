package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import frc.robot.Constants;

public class EffectorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.EFFECTOR_MOTOR_ID, Constants.kCANBus);
    private final LaserCan coralLaser;

    // Control request (pre-allocated to avoid memory churn)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // State tracking
    public enum EffectorState {
        IDLE,
        INTAKING,
        HOLDING_GAME_PIECE,
        SCORING
    }
    
    private EffectorState currentState = EffectorState.IDLE;
    private boolean isStoppedDueToLaser = false;
    private double stopTimestamp = 0;
    private static final double RESUME_DELAY_SECONDS = 1.0;
    private double lastCoralDistance = 0.0;
    private double filteredDistance = 0.0;
    private static final double FILTER_FACTOR = 0.8; // Adjust based on testing

    // Distance thresholds with hysteresis
    private static final double CORAL_DETECT_THRESHOLD = 30.0; // 30mm
    private static final double CORAL_RELEASE_THRESHOLD = 35.0; // 35mm

    // Status signals for monitoring
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Voltage> motorVoltage;

    public EffectorSubsystem() {
        // Configure status signals
        supplyVoltage = motor.getSupplyVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        motorVoltage = motor.getMotorVoltage();

        // Set update frequencies
        supplyVoltage.setUpdateFrequency(10); // 10 Hz
        supplyCurrent.setUpdateFrequency(50); // 50 Hz
        torqueCurrent.setUpdateFrequency(50); // 50 Hz
        motorVoltage.setUpdateFrequency(50); // 50 Hz

        configureMotor();

        // Initialize LaserCan
        coralLaser = new LaserCan(Constants.CORAL_LASER_ID);
        configureLaser();

        StatusSignal.refreshAll(supplyVoltage, supplyCurrent, torqueCurrent, motorVoltage);
    }

    private void configureMotor() {
        // Apply configuration
        motor.getConfigurator().apply(EffectorConstants.EFFECTOR_CONFIG);
    }

    private void configureLaser() {
        try {
            coralLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            coralLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            coralLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Coral Laser configuration failed! " + e);
        }
    }

    /**
     * Sets the effector output using duty cycle control
     * 
     * @param output The desired output (-1 to 1 for duty cycle)
     */
    public void setEffectorOutput(double output) {
        motor.setControl(dutyCycleRequest.withOutput(output));
    }

    /**
     * Stops the motor
     */
    public void stopMotor() {
        motor.setControl(dutyCycleRequest.withOutput(0));
        currentState = EffectorState.IDLE;
    }

    // Sensor methods
    public boolean isCoralRangeValid() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    public double getCoralDistanceMillimeters() {
        return lastCoralDistance;
    }

    public boolean isCoralDetected() {
        if (lastCoralDistance < CORAL_DETECT_THRESHOLD) {
            return true;
        } else if (isStoppedDueToLaser && lastCoralDistance < CORAL_RELEASE_THRESHOLD) {
            // Keep returning true until we're clearly past the threshold (hysteresis)
            return true;
        }
        return false;
    }

    private void stopIfDetected() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (lastCoralDistance < CORAL_DETECT_THRESHOLD) {
            if (!isStoppedDueToLaser) {
                stopMotor();
                isStoppedDueToLaser = true;
                currentState = EffectorState.HOLDING_GAME_PIECE;
            }
            // Reset timer while object is still detected
            stopTimestamp = currentTime;
        } else if (isStoppedDueToLaser && (currentTime - stopTimestamp > RESUME_DELAY_SECONDS)) {
            // Only clear the flag, don't auto-restart
            isStoppedDueToLaser = false;
        }
    }

    /******************************
     * Command Factories
     ******************************/

    // Coral Commands (with laser detection)
    public Command intakeCoralNoSensor() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.INTAKE_CORAL);
            currentState = EffectorState.INTAKING;
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    public Command scoreCoral() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.SCORE_CORAL);
            currentState = EffectorState.SCORING;
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }
    
    public Command slowCoral() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.lSCORE_CORAL);
            currentState = EffectorState.SCORING;
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    /**
     * Creates a command that runs the effector motor until a coral is detected by
     * the laser sensor.
     * If a coral is detected, the motor will stop.
     *
     * @return A command that runs the effector motor with sensor feedback.
     */
    public Command intakeCoral() {
        return new RunCommand(() -> {
            // Direct distance check for faster response
            if (!isStoppedDueToLaser && lastCoralDistance < CORAL_DETECT_THRESHOLD) {
                stopMotor();
                isStoppedDueToLaser = true;
                stopTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                currentState = EffectorState.HOLDING_GAME_PIECE;
            } else if (!isStoppedDueToLaser) {
                // Only run if we haven't detected anything yet
                setEffectorOutput(EffectorConstants.INTAKE_CORAL);
                currentState = EffectorState.INTAKING;
            }
            // Once stopped, stay stopped until command ends
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
                isStoppedDueToLaser = false;
            }
        };
    }
    
    // Algae Commands (NO laser detection)
    public Command intakeAlgae() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.INTAKE_ALGAE);
            currentState = EffectorState.INTAKING;
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    public Command scoreAlgae() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.SCORE_ALGAE);
            currentState = EffectorState.SCORING;
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }
    
    public Command holdAlgae() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.HOLD_ALGAE);
            currentState = EffectorState.HOLDING_GAME_PIECE;
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    @Override
    public void periodic() {
           // Update measurement with minimal processing
    LaserCan.Measurement measurement = coralLaser.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        // Less filtering, faster response
        lastCoralDistance = (FILTER_FACTOR * lastCoralDistance) + (0.7 * measurement.distance_mm);
        
        // Critical check right after getting data
        if (currentState == EffectorState.INTAKING && lastCoralDistance < CORAL_DETECT_THRESHOLD) {
            stopMotor();
            isStoppedDueToLaser = true;
        }
    }

        // Update dashboard
        SmartDashboard.putString("Effector/State", currentState.toString());
        SmartDashboard.putNumber("Effector/Current", supplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Torque", torqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Voltage", motorVoltage.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Supply Voltage", supplyVoltage.getValueAsDouble());

        SmartDashboard.putNumber("Effector/Sensor/Coral Distance (mm)", getCoralDistanceMillimeters());
        SmartDashboard.putBoolean("Effector/Sensor/Laser Valid", isCoralRangeValid());
        SmartDashboard.putBoolean("Effector/Sensor/Coral Detected", isCoralDetected());
        SmartDashboard.putBoolean("Effector/Stopped Due To Laser", isStoppedDueToLaser);
    }
}