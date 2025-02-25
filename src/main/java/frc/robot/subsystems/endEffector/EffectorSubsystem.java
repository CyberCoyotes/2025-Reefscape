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
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class EffectorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.EFFECTOR_MOTOR_ID, Constants.kCANBus);
    private final LaserCan coralLaser;

    // Control mode selection
    public enum ControlMode {
        DUTY_CYCLE, // Simple percent output control
        TORQUE_FOC // Advanced torque control
    }

    // TODO Test Both Control Modes
    private ControlMode currentControlMode = ControlMode.DUTY_CYCLE; // Default to simple control
    // private ControlMode currentControlMode = ControlMode.TORQUE_FOC; // Default
    // to simple control

    // Control requests (pre-allocated to avoid memory churn)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);

    // Status tracking
    private boolean isStoppedDueToLaser = false;
    private double stopTimestamp = 0;
    private static final double RESUME_DELAY_SECONDS = 3.0; // Changed from 1.0 to 3.0 seconds
    private double lastCoralDistance = 0.0;

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
        var config = EffectorConstants.EFFECTOR_CONFIG;

        // Add current limits for torque control
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        // Apply configuration
        motor.getConfigurator().apply(config);
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
     * Sets the control mode for the effector
     * 
     * @param mode The desired control mode (DUTY_CYCLE or TORQUE_FOC)
     */
    public void setControlMode(ControlMode mode) {
        currentControlMode = mode;
        // Reset the motor to 0 when switching modes
        stopMotor();
    }

    /**
     * Sets the effector output based on the current control mode
     * 
     * @param output The desired output (-1 to 1 for duty cycle, amps for torque)
     */
    public void setEffectorOutput(double output) {
        switch (currentControlMode) {
            case DUTY_CYCLE:
                motor.setControl(dutyCycleRequest.withOutput(output));
                break;
            case TORQUE_FOC:
                motor.setControl(torqueRequest.withOutput(output));
                break;
        }
    }

    /**
     * Stops the motor regardless of control mode
     */
    public void stopMotor() {
        switch (currentControlMode) {
            case DUTY_CYCLE:
                motor.setControl(dutyCycleRequest.withOutput(0));
                break;
            case TORQUE_FOC:
                motor.setControl(torqueRequest.withOutput(0));
                break;
        }
    }

    // Sensor methods
    public boolean isCoralRangeValid() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    public double getCoralDistanceMillimeters() {
        return lastCoralDistance;
    }

    // Changed from 30mm to 100mm (10cm)
    private static final double CORAL_DETECT_THRESHOLD = 100.0; // 100mm = 10cm

    public boolean isCoralDetected() {
        return getCoralDistanceMillimeters() < CORAL_DETECT_THRESHOLD;
    }

    public void stopIfDetected() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (isCoralDetected()) {
            if (!isStoppedDueToLaser) {
                stopMotor();
                isStoppedDueToLaser = true;
                stopTimestamp = currentTime;
            }
        } else if (isStoppedDueToLaser && (currentTime - stopTimestamp > RESUME_DELAY_SECONDS)) {
            // Resume with appropriate control mode
            switch (currentControlMode) {
                case DUTY_CYCLE:
                    setEffectorOutput(EffectorConstants.SCORE_CORAL);
                    break;
                case TORQUE_FOC:
                    setEffectorOutput(EffectorConstants.SCORE_CURRENT_AMPS);
                    break;
            }
            isStoppedDueToLaser = false;
        }
    }

    /******************************
     * Command Factories
     ******************************/

    // Duty Cycle Control Commands
    public Command runEffectorDutyCycleORIGINAL() {
        return run(() -> {
            setControlMode(ControlMode.DUTY_CYCLE);
            setEffectorOutput(EffectorConstants.INTAKE_CORAL);
        });
    }

    // Duty Cycle Control
    public Command intakeCoralNoSensor() {
        return new RunCommand(() -> {
            setControlMode(ControlMode.DUTY_CYCLE);
            setEffectorOutput(EffectorConstants.INTAKE_CORAL);
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    /**
     * Creates a command that runs the effector until a coral is detected
     * or the button is released. If coral is detected, motor stops for 3 seconds.
     * 
     * @param buttonReleased A boolean supplier that returns true when the button is released
     * @return A command that controls the effector with automated coral detection
     */
    public Command intakeCoralWithSensor(BooleanSupplier buttonReleased) {
        return new RunCommand(() -> {
            // If coral is detected, motor is already stopped due to stopIfDetected() in periodic()
            if (!isCoralDetected() && !isStoppedDueToLaser) {
                setControlMode(ControlMode.DUTY_CYCLE);
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
            setControlMode(ControlMode.DUTY_CYCLE);
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
            setControlMode(ControlMode.DUTY_CYCLE);
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
            setControlMode(ControlMode.DUTY_CYCLE);
            setEffectorOutput(EffectorConstants.INTAKE_ALGAE);
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    // Duty Cycle Control
    public Command scoreAlgae() {
        return new RunCommand(() -> {
            setControlMode(ControlMode.DUTY_CYCLE);
            setEffectorOutput(EffectorConstants.SCORE_ALGAE);
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    // Torque Control Commands
    public Command runEffectorTorque() {
        return run(() -> {
            setControlMode(ControlMode.TORQUE_FOC);
            setEffectorOutput(EffectorConstants.INTAKE_CURRENT_AMPS);
        });
    }

    public Command scoreEffectorTorque() {
        return run(() -> {
            setControlMode(ControlMode.TORQUE_FOC);
            setEffectorOutput(EffectorConstants.SCORE_CURRENT_AMPS);
        });
    }

    // Common Commands
    /**
     * Creates a command that runs the effector motor until a coral is detected by
     * the sensor.
     * If a coral is detected, the motor will stop.
     *
     * @return A command that runs the effector motor with sensor feedback.
     */
    public Command intakeCoral() {
        return run(() -> {
            if (isCoralDetected()) {
                stopMotor();
            } else {
                setControlMode(ControlMode.DUTY_CYCLE);
                setEffectorOutput(EffectorConstants.INTAKE_CORAL);
            }
        }).until(this::isCoralDetected);
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
        SmartDashboard.putString("Effector/Control Mode", currentControlMode.toString());
        SmartDashboard.putNumber("Effector/Current", supplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Torque", torqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Voltage", motorVoltage.getValueAsDouble());
        SmartDashboard.putNumber("Effector/Supply Voltage", supplyVoltage.getValueAsDouble());

        SmartDashboard.putNumber("Effector/Sensor/Coral Distance (mm)", getCoralDistanceMillimeters());
        SmartDashboard.putBoolean("Effector/Sensor/Laser Valid", isCoralRangeValid());
        SmartDashboard.putBoolean("Effector/Sensor/Coral Detected", isCoralDetected());
        SmartDashboard.putBoolean("Effector/Stopped Due To Detection", isStoppedDueToLaser);
    }
}