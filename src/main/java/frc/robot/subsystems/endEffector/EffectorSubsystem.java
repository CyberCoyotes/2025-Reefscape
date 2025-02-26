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
import java.util.function.BooleanSupplier;

public class EffectorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.EFFECTOR_MOTOR_ID, Constants.kCANBus);
    private final LaserCan coralLaser;

    // Control requests (pre-allocated to avoid memory churn)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // Status tracking
    private boolean isStoppedDueToLaser = false;
    private double stopTimestamp = 0;
    private static final double RESUME_DELAY_SECONDS = 1.0; 
    private double lastCoralDistance = 0.0;
    private double previousDistance = 1000.0; // For trend detection
    
    // Detection settings
    private static final double CORAL_DETECT_THRESHOLD = 150.0; // Increased from 100mm to 150mm
    private static final double CORAL_APPROACH_THRESHOLD = 250.0; // Earlier warning threshold

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

        // Set all update frequencies to same value for consistency
        supplyVoltage.setUpdateFrequency(50); // Increased to 50 Hz
        supplyCurrent.setUpdateFrequency(50);
        torqueCurrent.setUpdateFrequency(50);
        motorVoltage.setUpdateFrequency(50);

        configureMotor();

        // Initialize LaserCan
        coralLaser = new LaserCan(Constants.CORAL_LASER_ID);
        configureLaser();

        StatusSignal.refreshAll(supplyVoltage, supplyCurrent, torqueCurrent, motorVoltage);
    }

    private void configureMotor() {
        var config = EffectorConstants.EFFECTOR_CONFIG;

        // Simplify current limiting - only use one type
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;

        // Apply configuration
        motor.getConfigurator().apply(config);
    }

    private void configureLaser() {
        try {
            coralLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            // Smaller region of interest for faster processing
            coralLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(6, 6, 12, 12));
            // Faster timing budget for quicker measurements
            coralLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Coral Laser configuration failed! " + e);
        }
    }

    /**
     * Sets the effector output using duty cycle control
     * @param output The desired output (-1 to 1)
     */
    public void setEffectorOutput(double output) {
        motor.setControl(dutyCycleRequest.withOutput(output));
    }

    /**
     * Stops the motor
     */
    public void stopMotor() {
        motor.setControl(dutyCycleRequest.withOutput(0));
    }

    // Sensor methods
    public boolean isCoralRangeValid() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    // Measured in mm
    public double getCoralDistance() {
        return lastCoralDistance;
    }

    /**
     * Detects coral using both absolute distance and trend detection
     * @return true if coral is detected or approaching rapidly
     */
    public boolean isCoralDetected() {
        double currentDistance = getCoralDistance();
        
        // Check if coral is close enough to detect
        boolean isClose = currentDistance < CORAL_DETECT_THRESHOLD;
        
        // Check if coral is approaching rapidly (distance decreasing and within warning threshold)
        boolean isApproaching = currentDistance < previousDistance &&
                                currentDistance < CORAL_APPROACH_THRESHOLD &&
                                (previousDistance - currentDistance) > 20; // Rapid approach threshold
        
        // Update previous distance for next check
        previousDistance = currentDistance;
        
        return isClose || isApproaching;
    }

    public void stopIfDetected() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (isCoralDetected()) {
            if (!isStoppedDueToLaser) {
                stopMotor();
                isStoppedDueToLaser = true;
                stopTimestamp = currentTime;
                
                // Log detection for debugging
                System.out.println("Coral detected at distance: " + lastCoralDistance + "mm, motor stopped");
            }
        } else if (isStoppedDueToLaser && (currentTime - stopTimestamp > RESUME_DELAY_SECONDS)) {
            // Motor resumes after delay has passed and coral is no longer detected
            isStoppedDueToLaser = false;
        }
    }

    /******************************
     * Command Factories
     ******************************/

    // Duty Cycle Control
    public Command intakeCoralNoSensor() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.INTAKE_CORAL);
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    /**
     * Creates a command that runs the effector until coral is detected
     * or the button is released. If coral is detected, motor stops for delay.
     * 
     * @param buttonReleased A boolean supplier that returns true when the button is released
     * @return A command that controls the effector with automated coral detection
     */
    public Command intakeCoralWithSensor(BooleanSupplier buttonReleased) {
        return new RunCommand(() -> {
            // If coral is detected, motor is already stopped due to stopIfDetected() in periodic()
            if (!isCoralDetected() && !isStoppedDueToLaser) {
                setEffectorOutput(EffectorConstants.INTAKE_CORAL);
            }
        }, this) {
            @Override
            public boolean isFinished() {
                // End the command if the button is released
                return buttonReleased.getAsBoolean();
            }

            @Override
            public void end(boolean interrupted) {
                // If ending due to button release and not due to coral detection,
                // make sure to stop the motor
                if (!isStoppedDueToLaser) {
                    stopMotor();
                }
            }
        };
    }

    // Duty Cycle Control
    public Command scoreCoral() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.SCORE_CORAL);
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
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    public Command intakeAlgae() {
        return new RunCommand(() -> {
            setEffectorOutput(EffectorConstants.INTAKE_ALGAE);
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
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    /**
     * Creates a command that runs the effector motor until coral is detected by
     * the sensor. If coral is detected, the motor will stop.
     *
     * @return A command that runs the effector motor with sensor feedback.
     */
    public Command intakeCoral() {
        return new RunCommand(() -> {
            if (isCoralDetected()) {
                stopMotor();
            } else {
                setEffectorOutput(EffectorConstants.INTAKE_CORAL);
            }
        }, this).until(this::isCoralDetected);
    }

    @Override
    public void periodic() {
        // Update the coral laser measurement
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        
        // Update last distance if measurement is valid
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            lastCoralDistance = measurement.distance_mm;
        }

        // Handle automatic stopping based on sensor reading
        stopIfDetected();

        // Update dashboard
        SmartDashboard.putNumber("Effector/Current", supplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Torque", torqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Voltage", motorVoltage.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Supply Voltage", supplyVoltage.getValueAsDouble());

        SmartDashboard.putNumber("Effector/Sensor/Coral Distance (mm)", getCoralDistance());
        SmartDashboard.putBoolean("Effector/Sensor/Laser Valid", isCoralRangeValid());
        SmartDashboard.putBoolean("Effector/Sensor/Coral Detected", isCoralDetected());
        SmartDashboard.putBoolean("Effector/Stopped Due To Detection", isStoppedDueToLaser);
        SmartDashboard.putNumber("Effector/Sensor/Previous Distance", previousDistance);
    }
}