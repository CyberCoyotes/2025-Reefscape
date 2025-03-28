package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused") // Suppress warnings for unused imports and methods

/**
 * Subsystem class to primarily use a Time of Flight sensor from 'Playing with Fusion'.
 */
public class ReefTOFSubsystem extends SubsystemBase {

    // Time of Flight sensor
    private final TimeOfFlight reefTOF = new TimeOfFlight(Constants.REEF_SENSOR_ID);
    
    private int YOU_SHALL_NOT_PASS = 300; // TODO Determine empirically

    // NetworkTable entries for more reliable data publishing
    private final NetworkTable reef_sensorTable;
    private final NetworkTableEntry reef_distanceEntry;
    private final NetworkTableEntry reef_atReefTargetEntry;
    private final NetworkTableEntry reef_thresholdEntry; // static
    
    // Constructor
    public ReefTOFSubsystem() {
        // Initialize sensor with short ranging mode and fast update rate (20ms)
        reefTOF.setRangingMode(RangingMode.Medium, 1.5);
        
        // Set up NetworkTables for more reliable data publishing
        reef_sensorTable = NetworkTableInstance.getDefault().getTable("reefTOF");
        reef_distanceEntry = reef_sensorTable.getEntry("Distance");
        reef_thresholdEntry = reef_sensorTable.getEntry("DistanceThreshold");
        reef_atReefTargetEntry = reef_sensorTable.getEntry("AtReefTarget");
        
        // Initialize threshold in NetworkTables
        reef_thresholdEntry.setDouble(YOU_SHALL_NOT_PASS);
        
        // Also set up SmartDashboard for visualization
        SmartDashboard.putString("reefTOF/Status", "Initialized");
        SmartDashboard.putNumber("reefTOF/You Shall Not Pass", YOU_SHALL_NOT_PASS);
        
        // Make distance threshold adjustable from dashboard
        SmartDashboard.setPersistent("reefTOF/Distance Threshold");
    }

    /**
     * Gets the raw distance reading from the sensor to the nearest object
     */
    public double getReefDistance() {
        return reefTOF.getRange();
    }
    
    /**
     * Returns true if the reef branch is reached, false if not
     */
    public boolean atReefTarget() {
        return (getReefDistance() > 0) && (getReefDistance() < YOU_SHALL_NOT_PASS);
    }
    
    /**
     * Set the distance threshold for reef detection
     */
    public void setDistanceThreshold(int threshold) {
        YOU_SHALL_NOT_PASS = threshold;
        reef_thresholdEntry.setDouble(YOU_SHALL_NOT_PASS);
        SmartDashboard.putNumber("reefTOF/Distance Threshold", YOU_SHALL_NOT_PASS);
    }
    
    @Override
    public void periodic() {
        // Get current distance
        double currentDistance = getReefDistance();
        boolean reefDistanceReached = atReefTarget();
        
        // Update NetworkTable entries - this ensures data is published
        reef_distanceEntry.setDouble(currentDistance);
        reef_atReefTargetEntry.setBoolean(reefDistanceReached);
       
        // Update SmartDashboard values
        SmartDashboard.putNumber("reefTOF/Raw Distance (mm)", currentDistance);
        
    }

} // end of class