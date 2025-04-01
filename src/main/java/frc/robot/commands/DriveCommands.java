package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrontTOFSubsystem;
import frc.robot.Constants;

@SuppressWarnings("unused")
/**
 * This class contains commands for driving the robot.
 * It includes commands for driving forward, backward, and until a certain distance is reached.
 */

public class DriveCommands {
    private final CommandSwerveDrivetrain drivetrain;

    public DriveCommands(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Command driveForward15cm() {
        return new DriveForwardCommand(drivetrain, 0.15);
    }

    public Command driveForward15cm(double speedMetersPerSecond) {
        return new DriveForwardCommand(drivetrain, 0.15, speedMetersPerSecond);
    }

    public Command driveDistance(double distanceMeters) {
        return new DriveForwardCommand(drivetrain, distanceMeters);
    }

    public Command driveDistance(double distanceMeters, double speedMetersPerSecond) {
        return new DriveForwardCommand(drivetrain, distanceMeters, speedMetersPerSecond);
    }

    public Command driveBackward15cm() {
        return new DriveForwardCommand(drivetrain, -0.15);
    }

    public Command driveUntilDistance(FrontTOFSubsystem tofSensor) {
        return new DriveUntilDistance(drivetrain, tofSensor);
    }

    public Command slowMoDrive(CommandXboxController driverController, double slowMoFactor) {
        return new SlowMoDriveCommand(drivetrain, driverController, slowMoFactor);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return drivetrain.run(() -> drivetrain.setControl(requestSupplier.get()));
    }

    private static class DriveForwardCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final double distanceMeters;
        private final double speedMetersPerSecond;
        private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
        private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private double timeToTravel;

        public DriveForwardCommand(CommandSwerveDrivetrain drivetrain, double distanceMeters, double speedMetersPerSecond) {
            this.drivetrain = drivetrain;
            this.distanceMeters = distanceMeters;
            this.speedMetersPerSecond = speedMetersPerSecond;

            this.timeToTravel = Math.abs(distanceMeters / speedMetersPerSecond);

            addRequirements(drivetrain);
        }

        public DriveForwardCommand(CommandSwerveDrivetrain drivetrain, double distanceMeters) {
            this(drivetrain, distanceMeters, 1);
        }

        public static Command forward15cm(CommandSwerveDrivetrain drivetrain, double speedMetersPerSecond) {
            return new DriveForwardCommand(drivetrain, 0.15, speedMetersPerSecond);
        }

        @Override
        public void initialize() {
            timer.reset();
            timer.start();
        }

        @Override
        public void execute() {
            double speed = distanceMeters >= 0 ? speedMetersPerSecond : -speedMetersPerSecond;
            drivetrain.setControl(robotCentricRequest
                    .withVelocityX(speed)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setControl(robotCentricRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
            timer.stop();
        }

        @Override
        public boolean isFinished() {
            return timer.hasElapsed(timeToTravel);
        }
    }

    private static class DriveUntilDistance extends SequentialCommandGroup {
        private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

        public DriveUntilDistance(CommandSwerveDrivetrain drivetrain, FrontTOFSubsystem tofSensor) {
            addCommands(
                    Commands.runOnce(() -> System.out.println("Starting approach to reef")),
                    drivetrain.run(() -> {
                        drivetrain.setControl(driveRequest
                                .withVelocityX(Constants.AUTO_DRIVE_SPEED)
                                .withVelocityY(0)
                                .withRotationalRate(0));
                    }).until(() -> tofSensor.getFrontDistance() <= Constants.AUTO_TARGET_DISTANCE),
                    drivetrain.runOnce(() -> drivetrain.setControl(driveRequest
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0))),
                    Commands.runOnce(() -> {
                        System.out.println("Target distance reached: " + tofSensor.getFrontDistance() + " mm");
                    })
            );

            addRequirements(drivetrain);
        }
    }

    private static class SlowMoDriveCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final CommandXboxController driverController;
        private final double slowMoFactor;
        private final SwerveRequest.FieldCentric slowMoDrive;

        public SlowMoDriveCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, double slowMoFactor) {
            this.drivetrain = drivetrain;
            this.driverController = driverController;
            this.slowMoFactor = slowMoFactor;
            this.slowMoDrive = new SwerveRequest.FieldCentric();
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
        }
    }
}
