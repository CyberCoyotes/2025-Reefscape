package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

/**
 * https://github.com/GrappleRobotics/LaserCAN/blob/master/docs/example-java.md
 */
public class LaserCANSubsystem extends SubsystemBase {
    // Hypothetical LaserCAN class from Grapple Robotics
    public static LaserCan coralLaser;
    public static LaserCan elevatorLaser;
    public double elevator_distance;

    public int CORAL_LASER_ID = 60;
    public int ELEVATOR_LASER_ID = 61;

    /**
     * Constructs a new LaserCANSubsystem with a particular CAN device ID.
     *
     * @param canId The CAN ID for this LaserCAN sensor
     * @throws ConfigurationFailedException
     */
    public LaserCANSubsystem(int CORAL_LASER_ID, int ELEVATOR_LASER_ID) throws ConfigurationFailedException {
        
        coralLaser = new LaserCan(CORAL_LASER_ID);
        elevatorLaser = new LaserCan(CORAL_LASER_ID);

        // Configure the sensor for short-range operation
        coralLaser.setRangingMode(LaserCan.RangingMode.SHORT);
        elevatorLaser.setRangingMode(LaserCan.RangingMode.SHORT);

        // coralLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        // coralLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);

    }


    /**
     * @return The most recent distance reading from the LaserCAN in millimeters.
     *         If sensor or reading is invalid, this might return -1, 0, etc.
     */
    public double getCoralDistance(double distance) {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        // return (measurement.isValid() && measurement.distance_mm() < 20);
        // if (measurement.distance_mm() < 20);
        return measurement.distance_mm();
    }

     /**
     * @return The most recent distance reading from the LaserCAN in millimeters.
     *         If sensor or reading is invalid, this might return -1, 0, etc.
     */

    public double getElevatorDistance() {
        LaserCan.Measurement measurement = elevatorLaser.getMeasurement();
    }


    @Override
    public void periodic() {
        // Gather current data
        // double currentDistMM = getDistanceMM();
        // double currentDistM = currentDistMM / 1000.0;
        // boolean validReading = isRangeValid();

        // Publish to SmartDashboard
        SmartDashboard.putNumber("LaserCAN/Coral", coralDistance.getDistance());
        SmartDashboard.putNumber("LaserCAN/Elevator", elevatorDistance.getDistance());
        // SmartDashboard.putBoolean("LaserCAN/IsValid", validReading);

    }
}
