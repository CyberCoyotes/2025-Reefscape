package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import frc.robot.Constants;

public class EffectorSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final LaserCan coralLaser;
    private boolean isStoppedDueToLaser = false;
    private double stopTimestamp = 0;
    private static final double RESUME_DELAY_SECONDS = 1.0; // 1-second delay before resuming

    public EffectorSubsystem() {
        motor = new TalonFX(Constants.EFFECTOR_MOTOR_ID, Constants.kCANBus);
        configureMotor();

        // Initialize LaserCan directly in this subsystem
        coralLaser = new LaserCan(Constants.CORAL_LASER_ID);
        configureLaser();
    }

    private void configureMotor() {
        motor.getConfigurator().apply(EffectorConstants.EFFECTOR_CONFIG);
    }

    private void configureLaser() {
        try {
            coralLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            coralLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            coralLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Coral Laser configuration failed! " + e);
        }
    }

    public void setEffectorOutput(double output) {
        motor.setControl(new DutyCycleOut(output));
    }


    public double getCoralDistanceMillimeters() {
        LaserCan.Measurement measurement = coralLaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        }
        return Double.MAX_VALUE; // Return a large value if the sensor is not working
    }

    public void stopIfObjectDetected() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (getCoralDistanceMillimeters() < 30) {
            if (!isStoppedDueToLaser) {
                motor.setControl(new DutyCycleOut(0.0)); // Stop motor
                isStoppedDueToLaser = true;
                stopTimestamp = currentTime;
            }
        } else if (isStoppedDueToLaser && (currentTime - stopTimestamp > RESUME_DELAY_SECONDS)) {
            motor.setControl(new DutyCycleOut(EffectorConstants.SCORE_CORAL));
            ; // Resume operation
            isStoppedDueToLaser = false;
        }
    }

    @Override
    public void periodic() {
        stopIfObjectDetected();
    }
}