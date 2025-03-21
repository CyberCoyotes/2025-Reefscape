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
public class FrontTOFSubsystem extends SubsystemBase {

    // Time of Flight sensor
    private final TimeOfFlight frontTOF = new TimeOfFlight(Constants.TOF_SENSOR_ID);
    
    private int frontTargetDistance = 725;
    private int FRONT_LOWER_LIMIT = 720;
    private int FRONT_UPPER_LIMIT = 730;
    
    // NetworkTable entries for more reliable data publishing
    private final NetworkTable front_sensorTable;
    private final NetworkTableEntry front_distanceEntry;
    // private final NetworkTableEntry front_isLoadedEntry;
    private final NetworkTableEntry front_thresholdEntry;
    
    // Constructor
    public FrontTOFSubsystem() {
        // Initialize sensor with short ranging mode and fast update rate (20ms)
        frontTOF.setRangingMode(RangingMode.Medium, 1.5);
        
        // Set up NetworkTables for more reliable data publishing
        front_sensorTable = NetworkTableInstance.getDefault().getTable("FrontTOF");
        front_distanceEntry = front_sensorTable.getEntry("Distance");
        front_thresholdEntry = front_sensorTable.getEntry("DistanceThreshold");
        
        // Initialize threshold in NetworkTables
        front_thresholdEntry.setDouble(frontTargetDistance);
        
        // Also set up SmartDashboard for visualization
        SmartDashboard.putString("FrontTOF/Status", "Initialized");
        SmartDashboard.putNumber("FrontTOF/Distance Threshold", frontTargetDistance);
        
        // Make distance threshold adjustable from dashboard
        SmartDashboard.setPersistent("FrontTOF/Distance Threshold");
    }

    /**
     * Gets the raw distance reading from the sensor to the nearest edge (in mm)
     */
    public double getFrontDistance() {
        return frontTOF.getRange();
    }
    
    /**
     * Returns true if the note is loaded, false if not
     */
    public boolean isFrontAtTarget() {
        return getFrontDistance() < frontTargetDistance;
    }
    
    /**
     * Set the distance threshold for note detection
     */
    public void setDistanceThreshold(int threshold) {
        frontTargetDistance = threshold;
        front_thresholdEntry.setDouble(frontTargetDistance);
        SmartDashboard.putNumber("FrontTOF/Distance Threshold", frontTargetDistance);
    }
    
    @Override
    public void periodic() {
        // Get current distance
        double currentDistance = getFrontDistance();
        boolean frontDistanceReached = isFrontAtTarget();
        
        // Update NetworkTable entries - this ensures data is published
        front_distanceEntry.setDouble(currentDistance);
        // front_isLoadedEntry.setBoolean(frontDistanceReached);
        
        // Check if the dashboard has updated our threshold value
        double dashThreshold = SmartDashboard.getNumber("FrontTOF/Distance Threshold", frontTargetDistance);
        if (dashThreshold != frontTargetDistance) {
            frontTargetDistance = (int) dashThreshold;
            front_thresholdEntry.setDouble(frontTargetDistance);
        }
        
        // Update SmartDashboard values
        SmartDashboard.putNumber("FrontTOF/Raw Distance (mm)", currentDistance);
        
        // Force a NetworkTables flush to ensure data is sent immediately
        // NetworkTableInstance.getDefault().flush();
    }
}