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
    private final TalonFX sideMotor = new TalonFX(Constants.EFFECTOR_MOTOR_ID, Constants.kCANBus);
    private final TalonFX topMotor = new TalonFX(Constants.TOP_EFFECTOR_MOTOR_ID, Constants.kCANBus);
    
    private TimeOfFlight coralSensor = new TimeOfFlight(Constants.CORAL_SENSOR_ID);

    private int coralDetectionDistance = 50; // mm 

    // NetworkTable entries for more reliable data publishing
    private final NetworkTable sensorTable;
    private final NetworkTableEntry distanceEntry;
    private final NetworkTableEntry isLoadedEntry;
    private final NetworkTableEntry thresholdEntry;

    // Control request (pre-allocated to avoid memory churn)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // Status signals for monitoring
    private final StatusSignal<Voltage> sideSupplyVoltage;
    private final StatusSignal<Voltage> sideMotorVoltage;
    private final StatusSignal<Voltage> topSupplyVoltage;
    private final StatusSignal<Voltage> topMotorVoltage;

    public EffectorSubsystem() {

        /* TOF SENSOR SETUP */
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
         SmartDashboard.setPersistent("CoralSensor/Adjust Distance Threshold");
        // Configure status signals
        sideSupplyVoltage = sideMotor.getSupplyVoltage();
        sideMotorVoltage = sideMotor.getMotorVoltage();
        topSupplyVoltage = topMotor.getSupplyVoltage();
        topMotorVoltage = topMotor.getMotorVoltage();

        // Set update frequencies
        sideSupplyVoltage.setUpdateFrequency(50); // 50 Hz = 20 ms
        sideMotorVoltage.setUpdateFrequency(50); // 50 Hz
        topSupplyVoltage.setUpdateFrequency(50); // 50 Hz = 20 ms
        topMotorVoltage.setUpdateFrequency(50); // 50 Hz

        configureMotor();

        StatusSignal.refreshAll(sideSupplyVoltage, sideMotorVoltage, topSupplyVoltage, topMotorVoltage);

    }

    private void configureMotor() {
        var effectorConfig = EffectorConstants.EFFECTOR_CONFIG;
        var topEffectorConfig = EffectorConstants.TOP_EFFECTOR_CONFIG;
       
        // Apply configuration
        sideMotor.getConfigurator().apply(effectorConfig);
        topMotor.getConfigurator().apply(topEffectorConfig);
    }


    /**
     * Sets the effector output using duty cycle control
     * 
     * @param output The desired output (-1 to 1 for duty cycle)
     */
    public void setSideEffector(double output) {
        sideMotor.setControl(dutyCycleRequest.withOutput(output));
    }

    /**
     * Sets the top roller output using duty cycle control
     * 
     * @param output The desired output (-1 to 1 for duty cycle)
     */
    public void setTopEffector(double output) {
        topMotor.setControl(dutyCycleRequest.withOutput(output));
    }

   // Stops the motor
    public void stopMotor() {
        sideMotor.setControl(dutyCycleRequest.withOutput(0));
        topMotor.setControl(dutyCycleRequest.withOutput(0));
    }

    /**
     * Stops the top roller motor
     */
    public void stopTopRoller() {
        topMotor.setControl(dutyCycleRequest.withOutput(0));
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
