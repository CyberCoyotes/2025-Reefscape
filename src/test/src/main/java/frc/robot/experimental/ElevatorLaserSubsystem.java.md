package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorLaserSubsystem extends SubsystemBase {
    // Constants for LaserCAN IDs

    // LaserCAN devices
    // private final LaserCan coralLaser;
    private final LaserCan elevatorLaser;

    // Last valid measurements
    // private double lastCoralDistance = 0.0;
    private double lastElevatorDistance = 0.0;

    public ElevatorLaserSubsystem() {
        // Initialize both LaserCAN sensors
        // coralLaser = new LaserCan(Constants.CORAL_LASER_ID);
        elevatorLaser = new LaserCan(Constants.ELEVATOR_LASER_ID);

        // Configure both sensors
        // configureLaser(coralLaser, "Coral");
        configureLaser(elevatorLaser, "Elevator");
    }


    private void configureLaser(LaserCan laser, String name) {
        try {
            laser.setRangingMode(LaserCan.RangingMode.SHORT);
            laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println(name + " Laser configuration failed! " + e);
        }
    }

    @Override
    public void periodic() {
        // Update measurements for both sensors
        // updateCoralMeasurement();
        updateElevatorMeasurement();

        // Post to SmartDashboard for debugging
        // SmartDashboard.putNumber("Coral Distance (mm)", lastCoralDistance);
        SmartDashboard.putNumber("Elevator Distance (mm)", lastElevatorDistance);
    }


    /*
    private void updateCoralMeasurement() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            lastCoralDistance = measurement.distance_mm;
        }
    }
     */

    private void updateElevatorMeasurement() {
        LaserCan.Measurement measurement = elevatorLaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            lastElevatorDistance = measurement.distance_mm;
        }
    }

    // Getter methods for distances (mm)
    /* 
    public double getCoralDistance() {
        return lastCoralDistance;
    }
*/
    // Getter methods for distances (mm)
    public double getElevatorDistance() {
        return lastElevatorDistance;
    }

    
  /*
    // Methods to check if measurements are valid
    public boolean isCoralRangeValid() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }
 */
    public boolean isElevatorRangeValid() {
        LaserCan.Measurement measurement = elevatorLaser.getMeasurement();
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