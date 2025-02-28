package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class SlowMoDriveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final double slowMoFactor;
    private final SwerveRequest.FieldCentric slowMoDrive;

    public SlowMoDriveCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, double slowMoFactor) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.slowMoFactor = slowMoFactor;
        this.slowMoDrive = new SwerveRequest.FieldCentric(); // Create a new SwerveRequest
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(slowMoDrive
            .withVelocityX(-driverController.getLeftY() * slowMoFactor)
            .withVelocityY(-driverController.getLeftX() * slowMoFactor)
            .withRotationalRate(-driverController.getRightX() * slowMoFactor)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // The default command will automatically take over when this command ends
        // No need to explicitly set a new default command here
    }
}