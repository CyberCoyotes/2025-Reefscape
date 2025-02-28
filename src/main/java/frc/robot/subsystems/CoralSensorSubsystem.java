package frc.robot.subsystems;

import java.util.Optional;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem class to primarily use a Time of Flight sensor from 'Playing with Fusion'.
 * It reads the distance from the sensor to the 'note' and determines if the note is in a load position.
 */
public class CoralSensorSubsystem extends SubsystemBase {

    // Time of Flight sensor
    private final TimeOfFlight noteDistance = new TimeOfFlight(42);
    
    // Distance threshold for considering a note loaded (in mm)
    public int noteDistanceCheck = 30;
    

    
    // Constructor
    public CoralSensorSubsystem() {
        // Initialize sensor with short ranging mode and fast update rate (20ms)
        noteDistance.setRangingMode(RangingMode.Short, 1);
        noteDistance.getAmbientLightLevel();

        
        // Create a structured dashboard layout
        SmartDashboard.putString("CoralSensor/Status", "Initialized");
        SmartDashboard.putNumber("CoralSensor/Distance Threshold", noteDistanceCheck);
        
        // Make distance threshold adjustable from dashboard
        // Now operators can tune this value without redeploying code
        SmartDashboard.setPersistent("CoralSensor/Distance Threshold");
    }

    /**
     * Gets the raw distance reading from the sensor to the nearest edge (in mm)
     */
    public double getNoteDistance() {
        System.out.println("Coral Distance: " + noteDistance.getRange());

        return noteDistance.getRange();
        
    }
    


    /**
     * Returns true if the note is loaded, false if not
     */
    public boolean isNoteLoaded() {
        // Use the filtered distance for more stable detection
        return noteDistance.getRange() < noteDistanceCheck;
    }
    
    
    @Override
    public void periodic() {
        // Get current distance
        double currentDistance = getNoteDistance();
        // System.err.println("CoralSensorSubsystem: " + noteDistance.getAmbientLightLevel());

       
        
        // Check if the dashboard has updated our threshold value
        double dashThreshold = SmartDashboard.getNumber("CoralSensor/Distance Threshold", noteDistanceCheck);
        if (dashThreshold != noteDistanceCheck) {
            noteDistanceCheck = (int) dashThreshold;
        }
        
        // Update all dashboard values
        SmartDashboard.putNumber("CoralSensor/Raw Distance (mm)", currentDistance);
        SmartDashboard.putBoolean("CoralSensor/Note Loaded", isNoteLoaded());

        
        
        // Optional: Force a NetworkTables update to ensure data transmission
    }
    
}