package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MaserCannon extends SubsystemBase {
    
    // Constants for LaserCAN IDs
    private final LaserCan reefMaser;

    // Last valid measurements
    private double lastReefDistance = 0.0;

    public MaserCannon() {
        // Initialize both LaserCAN sensors
        reefMaser = new LaserCan(Constants.MASER_ID);

        // Configure sensors
        configureMaser(reefMaser, "ReefMaser");
    }

    private void configureMaser(LaserCan maser, String name) {
        try {
            maser.setRangingMode(LaserCan.RangingMode.SHORT);
            maser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            maser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println(name + " Laser configuration failed! " + e);
        }
    }

    @Override
    public void periodic() {
        // Update measurements for both sensors
        updateReefMeasurement();

        // Post to SmartDashboard for debugging
        SmartDashboard.putNumber("Reef Distance (mm)", lastReefDistance);
    }

    private void updateReefMeasurement() {
        LaserCan.Measurement measurement = reefMaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            lastReefDistance = measurement.distance_mm;
        }
    }

    // Getter methods for distances (mm)
    public double getReefDistance() {
        return lastReefDistance;
    }

    public boolean isReefRangeValid() {
        LaserCan.Measurement measurement = reefMaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }
}

// Example usage in a command or another subsystem
/*
// Get distances
double coralDistance = m_tof.getCoralDistanceMeters();
double elevatorDistance = m_tof.getElevatorDistanceMillimeters();

// Check if readings are valid
if (m_tof.isCoralRangeValid()) {
    // Use coral distance
}

if (m_tof.isElevatorRangeValid()) {
    // Use elevator distance
}
 */


/* 
// Example command that triggers when target is within range
public Command isTargetInRange() {
    return new FunctionalCommand(
        () -> {}, // No initialization
        () -> {}, // No periodic execution
        (interrupted) -> {}, // No end behavior
        () -> m_tof.getCoralDistanceMeters() < 0.5, // Returns true when target is closer than 0.5 meters
        m_tof // Requires the TOF subsystem
    );
}
    https://claude.ai/chat/e76b632d-add8-4303-82f2-40758a6cd975
    */