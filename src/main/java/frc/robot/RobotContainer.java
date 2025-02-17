// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMode;
import frc.robot.subsystems.endEffector.EffectorState;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.climber.ClimberVoltageSubsystem;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.SetEndEffectorCommand;
import frc.robot.commands.WristCommands;

public class RobotContainer {

    private final EffectorSubsystem endEffector = new EffectorSubsystem();

    private final WristSubsystem wrist = new WristSubsystem();
    private final WristCommands wristCommands;

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ElevatorCommands elevatorCommands;

    private final ClimberVoltageSubsystem climber = new ClimberVoltageSubsystem();
    private final ClimberCommands climberCommands = new ClimberCommands(climber, wrist);

    private final TOFSubsystem m_tof = new TOFSubsystem();


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
        wristCommands = new WristCommands();

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
       autoChooser.addRoutine("STA", autoRoutines::STA);
       autoChooser.addRoutine("BetterSTA", autoRoutines::STA3);
       autoChooser.addRoutine("STI", autoRoutines::STI);
       autoChooser.addRoutine("STJ", autoRoutines::STJ);

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
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));
         */

        /***** Driver Controls *****/
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        driverController.leftBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.INTAKE_CORAL));
        driverController.rightBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.SCORE_CORAL));
        
        // driverController.start().onTrue(wrist.runOnce(() -> wrist.setWristZero()));

        driverController.x().onTrue(elevatorCommands.moveToL2());
        driverController.y().onTrue(elevatorCommands.moveToL3()); 
        driverController.b().onTrue(elevatorCommands.moveToL4());
        driverController.a().onTrue(elevatorCommands.moveToHome());

        driverController.povUp()
            .whileTrue(elevator.incrementUpCommand());
        driverController.povDown()
            .whileTrue(elevator.decrementDownCommand());
        driverController.povLeft().onTrue(wristCommands.setSafePose(wrist));
        driverController.povRight().onTrue(wristCommands.setSafePose(wrist));

        /***** Operator Controls *****/
        operatorController.start().
            onTrue(elevatorCommands.setMode(ElevatorMode.PERFORMANCE))
            .onFalse(elevatorCommands.setMode(ElevatorMode.SAFETY));

        operatorController.leftBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.INTAKE_ALGAE));
        operatorController.rightBumper()
            .whileTrue(new SetEndEffectorCommand(endEffector, EffectorState.SCORE_ALGAE));
        
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