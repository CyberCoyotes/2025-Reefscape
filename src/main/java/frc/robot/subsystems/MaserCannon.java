package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MaserCannon extends SubsystemBase {
    
    private final LaserCan reefMaser;
    
    // Instead of keeping a lastReefDistance, we'll directly fetch from sensor
    // when requested and only return values we know are current
    
    public MaserCannon() {
        reefMaser = new LaserCan(Constants.MASER_ID);
        
        // Configure sensor
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
    
    // This method will either return a current reading or -1 if no valid reading
    public double getReefDistance() {
        LaserCan.Measurement measurement = reefMaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        }
        // Return -1 to indicate no valid measurement
        return -1;
    }
    
    public boolean isReefRangeValid() {
        LaserCan.Measurement measurement = reefMaser.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }
    
    @Override
    public void periodic() {
        // Get a fresh measurement for SmartDashboard
        double currentDistance = getReefDistance();
        
        // Only put valid measurements on the dashboard (-1 means invalid)
        if (currentDistance >= 0) {
            SmartDashboard.putNumber("Reef Distance (mm)", currentDistance);
        } else {
            // Remove the value from SmartDashboard when invalid
            // or set it to NaN to show it's not a valid reading
            SmartDashboard.putNumber("Reef Distance (mm)", Double.NaN);
        }
        
        // Also post a boolean indicator for valid reading
        SmartDashboard.putBoolean("Reef Sensor Valid", currentDistance >= 0);
    }
}