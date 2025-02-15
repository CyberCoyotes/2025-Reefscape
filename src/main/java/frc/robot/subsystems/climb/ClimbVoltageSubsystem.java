package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;

public class ClimbVoltageSubsystem extends SubsystemBase {
    private static final int CLIMB_ID = 50;
    private static final CANBus kCANBus = new CANBus("rio");

    // Motor & control request object
    private final TalonFX climbMotor;
    private final VoltageOut voltageRequest;

    // voltage variable
    public final double CLIMB_VOLTAGE = 3; // This should be about 1/4 of the max voltage


    public ClimbVoltageSubsystem() {
        // Initialize motor
        climbMotor = new TalonFX(CLIMB_ID, kCANBus);

        // Create a Voltage control request initially set to 0 V
        voltageRequest = new VoltageOut(0);

       
        // If you want to configure the motor hardware (inversions, ramp, etc.), do it here:
        // e.g. climbMotor.getConfigurator().apply(...someMotorConfigs...);
    }

    /**
     * Run the motor "up" at +6 V, for example.
     */
    public void climbUp() {
        climbMotor.setControl(voltageRequest.withOutput(CLIMB_VOLTAGE));  // 6 V out of max ~12 V
    }

    /**
     * Run the motor "down" at -6 V, for example.
     */
    public void climbDown() {
        climbMotor.setControl(voltageRequest.withOutput(-CLIMB_VOLTAGE));
    }

    /**
     * Stop the motor by sending 0 V.
     */
    public void stopClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.0));
    }

    public void holdClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.5));
    }

    /***********************
     * 
     ****************************/
 /**
     * Returns a Command that drives the motor at +6 V until canceled or interrupted.
     */
    public Command climbUpCommand() {
        return run(
            () -> climbUp())
        // When command ends, stop the motor by setting 0 V:
        .finallyDo((boolean interrupted) -> stopClimb())
        .withName("ClimbUp");
    }

    /**
     * Returns a Command that drives the motor at -6 V until canceled or interrupted.
     */
    public Command climbDownCommand() {
        return run(
            () -> climbDown())
        // When command ends, stop the motor by setting 0 V:
        .finallyDo((boolean interrupted) -> stopClimb())
        .withName("ClimbDown");
    }

    /**
     * Returns a Command that immediately stops the climb motor (0 V).
     * This can be used in “instant” scenarios.
     */
    public Command stopClimbCommand() {
        return runOnce(
            () -> stopClimb()
        ).withName("StopClimb");
    }


    public Command climbUpCommand_v2() {
        return runOnce(this::climbUp);
    }

    
    @Override
    public void periodic() {
        // If you need to read signals or log data, do that here.
    }
}
