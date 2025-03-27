package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MaserCannon extends SubsystemBase {
    
    private final LaserCan reefMaser;

    // Last valid measurements
    private double lastReefDistance = 1000.0;

    public MaserCannon() {
        reefMaser = new LaserCan(Constants.MASER_ID);

        // Configure sensors
        configureMaser(reefMaser, "ReefMaser");
    }

    private void configureMaser(LaserCan maser, String name) {
        try {
            maser.setRangingMode(LaserCan.RangingMode.SHORT);
            maser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 6, 6));
            maser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println(name + " Laser configuration failed! " + e);
        }
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


    @Override
    public void periodic() {
        // Update measurements for both sensors
        updateReefMeasurement();

        // Post to SmartDashboard for debugging
        SmartDashboard.putNumber("Reef Distance (mm)", lastReefDistance);
    }

} // end of class