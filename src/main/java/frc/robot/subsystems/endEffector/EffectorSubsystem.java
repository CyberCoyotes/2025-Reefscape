package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")

public class EffectorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.EFFECTOR_MOTOR_ID, Constants.kCANBus);
    
    private TimeOfFlight coralSensor = new TimeOfFlight(Constants.CORAL_SENSOR_ID);

    private int coralDetectionDistance = 80; // mm 

    // NetworkTable entries for more reliable data publishing
    private final NetworkTable sensorTable;
    private final NetworkTableEntry distanceEntry;
    private final NetworkTableEntry isLoadedEntry;
    private final NetworkTableEntry thresholdEntry;

    // Control request (pre-allocated to avoid memory churn)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // Status signals for monitoring
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Voltage> motorVoltage;

    public EffectorSubsystem() {

        /* ToF SENSOR SETUP */
        // Initialize sensor with short ranging mode and fast update rate (20ms)
        coralSensor.setRangingMode(RangingMode.Short, 1);

         // Set up NetworkTables for more reliable data publishing
         sensorTable = NetworkTableInstance.getDefault().getTable("CoralSensor");
         distanceEntry = sensorTable.getEntry("Distance");
         isLoadedEntry = sensorTable.getEntry("NoteLoaded");
         thresholdEntry = sensorTable.getEntry("DistanceThreshold");
         
         // Initialize threshold in NetworkTables
         thresholdEntry.setDouble(coralDetectionDistance);
         
         // Also set up SmartDashboard for visualization
         SmartDashboard.putString("CoralSensor/Status", "Initialized");
         SmartDashboard.putNumber("CoralSensor/Distance Threshold", coralDetectionDistance);
         
         // Make distance threshold adjustable from dashboard
         SmartDashboard.setPersistent("CoralSensor/Distance Threshold");

         /* MOTOR */
        // Configure status signals
        supplyVoltage = motor.getSupplyVoltage();
        motorVoltage = motor.getMotorVoltage();

        // Set update frequencies
        supplyVoltage.setUpdateFrequency(50); // 50 Hz = 20 ms
        motorVoltage.setUpdateFrequency(50); // 50 Hz

        configureMotor();

        StatusSignal.refreshAll(supplyVoltage, motorVoltage);

    }


    private void configureMotor() {
        var config = EffectorConstants.EFFECTOR_CONFIG;
        
        // Apply configuration
        motor.getConfigurator().apply(config);
    }


    /**
     * Sets the effector output using duty cycle control
     * 
     * @param output The desired output (-1 to 1 for duty cycle)
     */
    public void setEffectorOutput(double output) {
        motor.setControl(dutyCycleRequest.withOutput(output));
    }

   // Stops the motor
    public void stopMotor() {
        motor.setControl(dutyCycleRequest.withOutput(0));
    }

    /*******************************
     * Sensor Methods
     *******************************/

    public double getCoralDistance() {
        // Get the range in millimeters
        // System.out.println("Coral Distance: " + coralSensor.getRange());
        return coralSensor.getRange();
    }

    public boolean isCoralLoaded() {
        /* Returns true if the note is loaded, false if not */
        // System.out.println("Coral Loaded: " + (coralSensor.getRange() < coralDetectionDistance));
        return coralSensor.getRange() < coralDetectionDistance; // return NoteDistance.
    }


    /******************************
     * Command Factories
     ******************************/

     /* Testing removal */
     /*
    public Command intakeCoralBasic() {
        return run(() -> {
            setEffectorOutput(EffectorConstants.INTAKE_CORAL);
        });
    }

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
     */
    /**
     * Creates a command that runs the effector motor until a coral is detected by
     * the sensor.
     * If a coral is detected, the motor will stop.
     *
     * @return A command that runs the effector motor with sensor feedback.
     */
    /*
    public Command intakeCoralWithSensor() {

        return new RunCommand(() -> {
            // Check coral sensor
            if (isCoralLoaded()) {
                stopMotor();
            } else {
                // No coral detected, run the intake as normal
                setEffectorOutput(EffectorConstants.INTAKE_CORAL);
            }
        }, this) {
            @Override
            public void end(boolean interrupted) {
                stopMotor();
            }
        };
    }

    public Command reverseCoralNoSensor() {
        return new RunCommand(() -> {
            setEffectorOutput(-EffectorConstants.INTAKE_CORAL*0.55);
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
    */
    @Override
    public void periodic() {
        // stopIfDetected();


        SmartDashboard.putBoolean("Effector/Sensor/isCoralLoaded", isCoralLoaded());        
        SmartDashboard.putNumber("Effector/Sensor/getCoralDistance (mm)", getCoralDistance());
        SmartDashboard.putNumber("Effector/Sensor/Coral Distance (mm)", coralDetectionDistance);

        //
        // Update dashboard - uncomment if needed
        // SmartDashboard.putNumber("Effector/Voltage", motorVoltage.getValueAsDouble());
        // SmartDashboard.putNumber("Effector/Supply Voltage", supplyVoltage.getValueAsDouble());

        // SmartDashboard.putNumber("Effector/Sensor/Coral Distance (mm)", getCoralDistanceMillimeters());
        // SmartDashboard.putBoolean("Effector/Sensor/Range Valid", isCoralRangeValid());
        // SmartDashboard.putBoolean("Effector/Sensor/Coral Detected", isCoralDetected());
        
    }
}