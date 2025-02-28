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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.SlowMoDriveCommand;
import frc.robot.commands.WristCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TOFSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.vision.CameraSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotContainer {

    private final EffectorSubsystem endEffector = new EffectorSubsystem();

    private final WristSubsystem wrist = new WristSubsystem();
    private final WristCommands wristCommands = new WristCommands(wrist);

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ElevatorCommands elevatorCommands = new ElevatorCommands(elevator, wrist);

    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final ClimberCommands climberCommands = new ClimberCommands(climber, wrist);

    private final CommandGroups commandGroups = new CommandGroups();
    
    private final TOFSubsystem m_tof = new TOFSubsystem();

   private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();


    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    
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
        autoFactory = drivetrain.createAutoFactory(); // Leave outside container?
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, endEffector, elevator); // Leave outside container?
    
        configureBindings();
        configureAutoRoutines();
    }


    private void configureAutoRoutines() {

        // TODO Create user friendly names for the Drive Team
        
       autoChooser.addRoutine("StartLeft->ScoreJ&A-L1", autoRoutines::STJtoAL1);
       //autoChooser.addRoutine("StartLeft->ScoreJ-L1&A-L2", autoRoutines::STJtoAL12);
       autoChooser.addRoutine("StartRight->ScoreE&B-L1", autoRoutines::SBEtoBL1);
       //autoChooser.addRoutine("StartRight->ScoreE-L1&B-L2", autoRoutines::SBEtoBL12);
       autoChooser.addRoutine("Smith Smasher", autoRoutines::MHL1);

       //autoChooser.addRoutine("BetterSTA", autoRoutines::STA3);
      // autoChooser.addRoutine("STA-L1", autoRoutines::STAL1);
      // autoChooser.addRoutine("STJ-L1", autoRoutines::STJL1);
       SmartDashboard.putData("Autonomous", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically // Drive forward with
                // negative Y (forward)
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                        // Drive left with negative X (left)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                        // Drive counterclockwise with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate)

                ));

        /***********************************************
         ** Driver Controls **
         ***********************************************/

        // Resets the gyro
         driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 
        
        // driverController.back().onTrue((/* */));

        driverController.leftBumper().whileTrue(endEffector.intakeCoral()); // (+)
        driverController.rightBumper().whileTrue(endEffector.scoreCoral()); //(+)

        // TODO Add a slow reverse Coral for fine tuning
        driverController.leftTrigger().whileTrue(endEffector.intakeAlgae()); // (-)
        // Have been trying 25%, bump up to 35% for testing
        driverController.rightTrigger().whileTrue(new SlowMoDriveCommand(drivetrain, driverController, 0.35));

        /* 
                                                **Y**  
                                          **X**      **B**
                                                **A**
        */

        driverController.x().onTrue(commandGroups.moveToL2Group(wristCommands, elevatorCommands));
        driverController.y().onTrue(commandGroups.moveToL3Group(wristCommands, elevatorCommands));

        driverController.a().onTrue(commandGroups.moveToHomeGroup(wristCommands, elevatorCommands));
        // Add a slow motion command for the driver to use when button held
        driverController.b().onTrue(new SlowMoDriveCommand(drivetrain, driverController, 0.25));

        // driverController.povUp().whileTrue(elevatorCommands.incrementUpCommand());
        // driverController.povDown().whileTrue(elevatorCommands.decrementDownCommand());
        driverController.povLeft().whileTrue(wristCommands.setStowed());
        // driverController.povRight().whileTrue(wristCommands.incrementOut());

        /***********************************************
         ** Operator Controls **
         ***********************************************/

         // Rotates the servo to a specific angle when the start button is pressed
        operatorController.start().onTrue(climberCommands.toggleServo());

        operatorController.leftBumper().whileTrue(climberCommands.incrementUp());
        operatorController.rightBumper().whileTrue(climberCommands.incrementDown());

        operatorController.leftTrigger().whileTrue(endEffector.intakeAlgae());
        operatorController.rightTrigger().whileTrue(endEffector.scoreAlgae());
        
        operatorController.x().onTrue(commandGroups.moveToPickAlgae2Group(wristCommands, elevatorCommands));
        operatorController.y().onTrue(commandGroups.moveToPickAlgae3Group(wristCommands, elevatorCommands));

        operatorController.a().onTrue(commandGroups.moveToScoreAlgaeGroup(wristCommands, elevatorCommands));
        // operatorController.b().onTrue(commandGroups.moveToL4Group(wristCommands, elevatorCommands));

        operatorController.povUp().whileTrue(elevator.incrementUp()); // Directly from the subsystem
        operatorController.povDown().whileTrue(elevator.incrementDown()); // Directly from the subsystem
        // operatorController.povUp().whileTrue(elevatorCommands.incrementUpVersion2()); // TODO Test before using
        // operatorController.povDown().whileTrue(elevatorCommands.incrementDownVersion2()); // TODO Test before using
        
        operatorController.povLeft().whileTrue(wristCommands.incrementIn());
        operatorController.povRight().whileTrue(wristCommands.incrementOut());

        drivetrain.registerTelemetry(logger::telemeterize);

    } // End of configureBindings

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();

    } // End of getAutonomousCommand


} // End of RobotContainer