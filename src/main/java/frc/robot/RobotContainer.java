// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.auto.AutoRoutines;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TOFSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem.WristPositions;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.ElevatorCommands;

public class RobotContainer {

    private final EffectorSubsystem endEffector = new EffectorSubsystem();

    private final WristSubsystem wrist = new WristSubsystem();
    private final WristSubsystem.CommandFactory wristCommands = new WristSubsystem.CommandFactory(wrist);

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ElevatorCommands elevatorCommands;

    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final ClimberCommands climberCommands = new ClimberCommands(climber, wrist);

    private final TOFSubsystem m_tof = new TOFSubsystem();


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second                                                            // max angular velocity

    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, endEffector);

        elevatorCommands = new ElevatorCommands(elevator, wrist);
        // wristCommands = new WristCommands();

        configureBindings();
        configureAutoRoutines();
        // SmartDashboard.putBoolean("Wrist/EncoderConnected", false);
        // wrist.setWristZero(); // Verify encoder reading resets

    }

    private void configureAutoRoutines() {
        
       // autoChooser.addRoutine("Drive Forward", autoRoutines::driveForward);
       // autoChooser.addRoutine("Center Score", autoRoutines::driveForward);
       //autoChooser.addRoutine("TwoMeters", autoRoutines::TwoMeters); 
       autoChooser.addRoutine("ScoreTwoMetersBack", autoRoutines::ScoreTwoMetersBack); 
       autoChooser.addRoutine("SetupA", autoRoutines::SetupA);
       autoChooser.addRoutine("BetterSTA", autoRoutines::STA3);
       autoChooser.addRoutine("STA-L1?", autoRoutines::STAL1);
       autoChooser.addRoutine("SmashA", autoRoutines::ReefSMASH);
       autoChooser.addRoutine("SmashB", autoRoutines::ReefSMASH2);

       SmartDashboard.putData("Autonomous", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                           // forward
                                                                                                           // with
                                                                                                           // negative Y
                                                                                                           // (forward)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                            // with negative X (left)
                ));

        /*
         * FIXME Comment out for Testing of other commands
         * driverController.b().whileTrue(drivetrain.applyRequest(() ->
         * point.withModuleDirection(new Rotation2d(-driverController.getLeftY(),
         * -driverController.getLeftX()))
         * ));
         */

        /***** Driver Controls *****/
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftBumper().whileTrue(endEffector.intakeCoral());
        driverController.rightBumper().whileTrue(endEffector.scoreCoral());

        // driverController.start().onTrue(wrist.runOnce(() -> wrist.setWristZero()));

        // driverController.start().and(driverController.x().onTrue(elevatorCommands.moveToL2Raw()));
        
        driverController.x().onTrue(elevatorCommands.moveToHomeRaw());
        driverController.y().onTrue(elevatorCommands.moveToL2Raw());
        // driverController.start().and(driverController.a().onTrue(elevatorCommands.moveToHomeRaw()));

        // driverController.y().onTrue(elevatorCommands.moveToL3Raw());
        driverController.b().onTrue(wristCommands.moveTo(WristPositions.SCORE_L2));
        driverController.a().onTrue(wristCommands.moveTo(WristPositions.STOWED));

        driverController.povUp().whileTrue(elevator.incrementUpCommand());
        driverController.povDown().whileTrue(elevator.decrementDownCommand());
        driverController.povLeft().whileTrue(wristCommands.incrementIn());
        driverController.povRight().whileTrue(wristCommands.incrementOut());

        /***** Operator Controls *****/

        operatorController.leftBumper().whileTrue(endEffector.intakeAlgae());
        operatorController.rightBumper().whileTrue(endEffector.scoreAlgae());

        operatorController.a().whileTrue(climberCommands.climbUpCommand());
        operatorController.b().whileTrue(climberCommands.climbDownCommand());
        // operatorController.x().whileTrue(_());
        // operatorController.y().whileTrue(_());

        operatorController.povUp().whileTrue(elevatorCommands.incrementUpRaw()); // Orange but no movement
        operatorController.povDown().whileTrue(elevatorCommands.incrementDown());
        // operatorController.povLeft().onTrue(WristCommands.incrementDown(wrist));
        // operatorController.povRight().onTrue(WristCommands.setSafePose(wrist));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }
}