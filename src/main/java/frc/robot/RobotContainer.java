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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EffectorState;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristMotorSubsystem;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.SetEndEffectorCommand;
import frc.robot.commands.WristCommands;

public class RobotContainer {

    private final EffectorSubsystem endEffector = new EffectorSubsystem();

    private final WristSubsystem wrist = new WristSubsystem();
    private final WristMotorSubsystem wristMotor = new WristMotorSubsystem();

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ElevatorCommands elevatorCommands;

// TODO Slomo
    private double MaxSpeed = 4; // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
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
        autoRoutines = new AutoRoutines(autoFactory, drivetrain);

        elevatorCommands = new ElevatorCommands(elevator, wrist);
        
        // TODO Toggle - True is safety mode, false is performance mode
        elevatorCommands.setTestMode(true);  // Set to FALSE for competition

        configureBindings();
        configureAutoRoutines();
        // SmartDashboard.putBoolean("Wrist/EncoderConnected", false);
            // wrist.setWristZero(); // Verify encoder reading resets
    }
    private void configureAutoRoutines() {
        
       // autoChooser.addRoutine("Drive Forward", autoRoutines::driveForward);
       // autoChooser.addRoutine("Center Score", autoRoutines::driveForward);
       autoChooser.addRoutine("TwoMeters", autoRoutines::TwoMeters); 
       autoChooser.addRoutine("TwoMetersBack", autoRoutines::TwoMetersBack); 
       //autoChooser.addRoutine("STA", autoRoutines::STA);


        
        //autoChooser.addRoutine("Testing Events", autoRoutines::testEvents);
         // autoBETAChooser.addRoutine("Drive and Align", autoRoutines::driveAndAlign);

        SmartDashboard.putData("Autonomous", autoChooser);
       
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /* FIXME Comment out for Testing of other commands 
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));
         */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // CORAL
        driverController.leftBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.INTAKE_CORAL));
        driverController.rightBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.SCORE_CORAL));
        
        // ALGAE a.k.a reverse CORAL
        operatorController.leftBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.INTAKE_ALGAE));
        operatorController.rightBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.SCORE_ALGAE));


            /*
        driverController.x()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.HOLD));
         */

        // Example button bindings in RobotContainer
        // driverController.start().onTrue(wrist.runOnce(() -> wrist.setWristZero()));
    
        // Hold buttons for manual movement
        // driverController.b().whileTrue(elevator.moveToPosition(10));
        // driverController.a().whileTrue(elevator.moveToPositionAndWait(10));

        // driverController.a().whileTrue(elevator.moveDown());
    
        // One-time position commands
        // driverController.x().onTrue(elevatorCommands.moveToL1()); // Works

        // Wrist safety check is now default behavior with elevator
        driverController.a().onTrue(elevatorCommands.moveToBase()); // Moves the wrist, then moves elevator
        driverController.b().onTrue(elevatorCommands.moveToL1());
        driverController.y().onTrue(elevatorCommands.moveToL2());
        driverController.x().onTrue(elevatorCommands.moveToL3());

        // driverController.povUp().onTrue(WristCommands.setLoadCoral(wrist));
        operatorController.povLeft().onTrue(WristCommands.setElevatorSafe(wrist));
        // driverController.povLeft().onTrue(WristCommands.setL2(wrist));
        // driverController.povRight().onTrue(WristCommands.setL4(wrist));

        // driverController.povUp().whileTrue(ElevatorCommands.manualUp(elevator));
        // driverController.povDown().whileTrue(ElevatorCommands.manualDown(elevator));
        // Bind manual elevator control to the left stick Y-axis
        elevator.setDefaultCommand(
            elevatorCommands.createManualCommand(
                () -> operatorController.getLeftY()
            )
        );

        // driverController.povRight().whileTrue(WristCommands.manualUp(wristMotor));
        // driverController.povLeft().whileTrue(WristCommands.manualDown(wristMotor));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        return autoChooser.selectedCommand();
    }
}
