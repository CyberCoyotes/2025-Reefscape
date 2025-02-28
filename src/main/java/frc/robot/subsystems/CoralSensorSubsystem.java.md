package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem class to primarily use a Time of Flight sensor from 'Playing with Fusion'.
 * It reads the distance from the sensor to the 'note' and determines if the note is in a load position.
 */
public class CoralSensorSubsystem extends SubsystemBase {

    // Time of Flight sensor
    private final TimeOfFlight noteDistance = new TimeOfFlight(42);
    
    // Distance threshold for considering a note loaded (in mm)
    private int noteDistanceCheck = 30;
    
    // NetworkTable entries for more reliable data publishing
    private final NetworkTable sensorTable;
    private final NetworkTableEntry distanceEntry;
    private final NetworkTableEntry isLoadedEntry;
    private final NetworkTableEntry thresholdEntry;
    
    // Constructor
    public CoralSensorSubsystem() {
        // Initialize sensor with short ranging mode and fast update rate (20ms)
        noteDistance.setRangingMode(RangingMode.Short, 1);
        
        // Set up NetworkTables for more reliable data publishing
        sensorTable = NetworkTableInstance.getDefault().getTable("CoralSensor");
        distanceEntry = sensorTable.getEntry("Distance");
        isLoadedEntry = sensorTable.getEntry("NoteLoaded");
        thresholdEntry = sensorTable.getEntry("DistanceThreshold");
        
        // Initialize threshold in NetworkTables
        thresholdEntry.setDouble(noteDistanceCheck);
        
        // Also set up SmartDashboard for visualization
        SmartDashboard.putString("CoralSensor/Status", "Initialized");
        SmartDashboard.putNumber("CoralSensor/Distance Threshold", noteDistanceCheck);
        
        // Make distance threshold adjustable from dashboard
        SmartDashboard.setPersistent("CoralSensor/Distance Threshold");
    }

    /**
     * Gets the raw distance reading from the sensor to the nearest edge (in mm)
     */
    public double getNoteDistance() {
        return noteDistance.getRange();
    }
    
    /**
     * Returns true if the note is loaded, false if not
     */
    public boolean isNoteLoaded() {
        return getNoteDistance() < noteDistanceCheck;
    }
    
    /**
     * Set the distance threshold for note detection
     */
    public void setDistanceThreshold(int threshold) {
        noteDistanceCheck = threshold;
        thresholdEntry.setDouble(noteDistanceCheck);
        SmartDashboard.putNumber("CoralSensor/Distance Threshold", noteDistanceCheck);
    }
    
    @Override
    public void periodic() {
        // Get current distance
        double currentDistance = getNoteDistance();
        boolean noteLoaded = isNoteLoaded();
        
        // Update NetworkTable entries - this ensures data is published
        distanceEntry.setDouble(currentDistance);
        isLoadedEntry.setBoolean(noteLoaded);
        
        // Check if the dashboard has updated our threshold value
        double dashThreshold = SmartDashboard.getNumber("CoralSensor/Distance Threshold", noteDistanceCheck);
        if (dashThreshold != noteDistanceCheck) {
            noteDistanceCheck = (int) dashThreshold;
            thresholdEntry.setDouble(noteDistanceCheck);
        }
        
        // Update SmartDashboard values
        SmartDashboard.putNumber("CoralSensor/Raw Distance (mm)", currentDistance);
        SmartDashboard.putBoolean("CoralSensor/Note Loaded", noteLoaded);
        
        // Force a NetworkTables flush to ensure data is sent immediately
        // NetworkTableInstance.getDefault().flush();
    }
}