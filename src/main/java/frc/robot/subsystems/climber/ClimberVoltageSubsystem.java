package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;

public class ClimberVoltageSubsystem extends SubsystemBase {
    private static final int CLIMB_ID = 25;
    private static final CANBus kCANBus = new CANBus("rio");

    // Motor & control request object
    private final TalonFX climbMotor;
    private final VoltageOut voltageRequest;

    // voltage variable
    public final double CLIMB_VOLTAGE = 9; // With a max of 12, This should be about 3/4 of the max voltage


    public ClimberVoltageSubsystem() {
        // Initialize motor
        climbMotor = new TalonFX(CLIMB_ID, kCANBus);

        // Create a Voltage control request initially set to 0 V
        voltageRequest = new VoltageOut(0);
    }


    // Output to lift the robot up
    public void climbUp() {
        climbMotor.setControl(voltageRequest.withOutput(-CLIMB_VOLTAGE));  // 6 V out of max ~12 V
    }

    // Output to lower the robot down
    public void climbDown() {
        climbMotor.setControl(voltageRequest.withOutput(CLIMB_VOLTAGE));
    }

    public void stopClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.0));
    }

    public void holdClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.5));
    }

    /***********************
     * 
     ****************************/
 
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
        // Send position to Logger
        Logger.recordOutput("climbMotorPosition", climbMotor.getPosition().getValue());

    }
}