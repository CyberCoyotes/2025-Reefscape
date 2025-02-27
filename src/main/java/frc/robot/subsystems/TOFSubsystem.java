package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TOFSubsystem extends SubsystemBase {
    // LaserCAN devices
    private final LaserCan coralLaser;
    private final LaserCan elevatorLaser;

    // Last valid measurements
    private double lastCoralDistance = 0.0;
    private double lastElevatorDistance = 0.0;
    
    // Filtered measurements for smoother readings
    private double filteredCoralDistance = 0.0;
    private double filteredElevatorDistance = 0.0;
    
    // Adjust based on testing
    // Suggested lowering to improve responsiveness. Started with 0.8
    private static final double FILTER_FACTOR = 0.2; 
    
    public TOFSubsystem() {
        // Initialize both LaserCAN sensors
        coralLaser = new LaserCan(Constants.CORAL_LASER_ID);
        elevatorLaser = new LaserCan(Constants.ELEVATOR_LASER_ID);

        // Configure both sensors
        configureLaser(coralLaser, "Coral");
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
        updateCoralMeasurement();
        updateElevatorMeasurement();

        // Post to SmartDashboard for debugging
        SmartDashboard.putNumber("Coral Distance (mm)", lastCoralDistance);
        SmartDashboard.putNumber("Elevator Distance (mm)", lastElevatorDistance);
        SmartDashboard.putBoolean("Coral Range Valid", isCoralRangeValid());
        SmartDashboard.putBoolean("Elevator Range Valid", isElevatorRangeValid());
    }

    private void updateCoralMeasurement() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            // Apply simple low-pass filter for smoother readings
            filteredCoralDistance = (FILTER_FACTOR * filteredCoralDistance) + 
                                   ((1 - FILTER_FACTOR) * measurement.distance_mm);
            lastCoralDistance = filteredCoralDistance;
        }
    }

    private void updateElevatorMeasurement() {
        LaserCan.Measurement measurement = elevatorLaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            // Apply simple low-pass filter for smoother readings
            filteredElevatorDistance = (FILTER_FACTOR * filteredElevatorDistance) + 
                                      ((1 - FILTER_FACTOR) * measurement.distance_mm);
            lastElevatorDistance = filteredElevatorDistance;
        }
    }

    // Getter methods for distances (mm)
    public double getCoralDistance() {
        return lastCoralDistance;
    }

    public double getElevatorDistance() {
        return lastElevatorDistance;
    }
    
    // Methods to check if measurements are valid
    public boolean isCoralRangeValid() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    public boolean isElevatorRangeValid() {
        LaserCan.Measurement measurement = elevatorLaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }
}