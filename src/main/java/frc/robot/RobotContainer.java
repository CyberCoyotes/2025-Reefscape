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
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.DriveDistanceCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.SlowMoDriveCommand;
import frc.robot.commands.WristCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrontTOFSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.subsystems.vision.CameraSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

@SuppressWarnings("unused")

public class RobotContainer {

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final EffectorSubsystem endEffector = new EffectorSubsystem();
    private final EndEffectorCommands endEffectorCommands = new EndEffectorCommands(endEffector);
    private final WristSubsystem wrist = new WristSubsystem();
    private final WristCommands wristCommands = new WristCommands(wrist);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ElevatorCommands elevatorCommands = new ElevatorCommands(elevator);
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final ClimberCommands climberCommands = new ClimberCommands(climber, wrist);
    private final FrontTOFSubsystem frontToF = new FrontTOFSubsystem();
    private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
    private final CommandGroups commandGroups = new CommandGroups(wristCommands, elevatorCommands, endEffector, endEffectorCommands, frontToF, drivetrain);
    private final DriveDistanceCommands driveCommands = new DriveDistanceCommands(drivetrain);
    // private final CoralSensorSubsystem coralSensor = new CoralSensorSubsystem();
    
    // 3 meters per second max speed
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

    public RobotContainer() {

        autoFactory = drivetrain.createAutoFactory();
        // Check the AutoRoutines class to ensure the constructor matches this parameter list
        // If it doesn't, update either this call or the constructor in AutoRoutines class
        autoRoutines = new AutoRoutines(
                autoFactory,
                drivetrain,
                endEffector,
                elevator,
                elevatorCommands,
                commandGroups,                
                endEffectorCommands,
                wrist,
                wristCommands
                );

        // Alternative approach: Check the AutoRoutines.java file and pass parameters in the correct order

        configureBindings();
        configureAutoRoutines();
    }

    private void configureAutoRoutines() {

        autoChooser.addRoutine("StartLeft->ScoreJ&A-L1", autoRoutines::STJtoAL1);
        // autoChooser.addRoutine("StartLeft->ScoreJ-L1&A-L2", autoRoutines::STJtoAL12);
        autoChooser.addRoutine("StartLeft->ScoreAL4", autoRoutines::STAL4);
        autoChooser.addRoutine("StartLeft->ScoreJL4", autoRoutines::STJL4);
        autoChooser.addRoutine("StartRight->ScoreEL4", autoRoutines::SBEL4);
        autoChooser.addRoutine("StartLeft->ScoreJL4->ScoreAL4", autoRoutines::STJ4toAL4);
        autoChooser.addRoutine("StartRight->ScoreEL4->ScoreBL4", autoRoutines::SBE4toBL4);
        // autoChooser.addRoutine("StartLeft->ScoreJ-L1&A+AL2",
        // autoRoutines::STJtoAL1AL2);
        autoChooser.addRoutine("StartRight->ScoreE&B-L1", autoRoutines::SBEtoBL1);
        // autoChooser.addRoutine("StartRight->ScoreE-L1&B-L2",
        // autoRoutines::SBEtoBL12);
        // autoChooser.addRoutine("StartRight->ScoreE-L1&B+BL2",
        // autoRoutines::SBEtoBL1BL2);
        autoChooser.addRoutine("Smith Smasher", autoRoutines::MHL1);
        autoChooser.addRoutine("Left Road Runner", autoRoutines::LeftSideRoadRunner);
        // autoChooser.addRoutine("Left Beep-Beep", autoRoutines::LeftSideBeepBeep );

        // autoChooser.addRoutine("BetterSTA", autoRoutines::STA3);
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
      
        driverController.back().onTrue(commandGroups.autoRoadRunnerL4()); // Testing button only
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Handle End Effector Commands for Coral
        driverController.leftBumper().whileTrue(endEffectorCommands.intakeCoral()); // Includes a sensor to auto stop
        driverController.rightBumper().whileTrue(endEffectorCommands.scoreCoral()); // No Sensor

        driverController.leftTrigger().whileTrue(endEffectorCommands.reverseCoralNoSensor());
        driverController.rightTrigger().whileTrue(new SlowMoDriveCommand(drivetrain, driverController, 0.50));

        // Groups commands for wrist and elevator to move to specific positions
        driverController.x().onTrue(commandGroups.moveToL2(wristCommands, elevatorCommands));
        driverController.y().onTrue(commandGroups.moveToL3(wristCommands, elevatorCommands));
        driverController.a().onTrue(commandGroups.moveToHome(wristCommands, elevatorCommands));
        driverController.b().onTrue(commandGroups.moveToL4(wristCommands, elevatorCommands));

        // Manual Elevator Commands
        driverController.povUp().whileTrue(elevatorCommands.incrementUp());
        driverController.povDown().whileTrue(elevatorCommands.incrementDown());

        // Manual Wrist Commands
        driverController.povLeft().whileTrue(wristCommands.incrementIn());
        driverController.povRight().whileTrue(wristCommands.incrementOut());

        /***********************************************
         ** Operator Controls **
         ***********************************************/

        // Rotates the servo to a specific angle when the start button is pressed
        operatorController.back().onTrue(commandGroups.releaseKickSetWrist(wristCommands, climberCommands)); // Redundant 
        operatorController.start().onTrue(commandGroups.releaseKickSetWrist(wristCommands, climberCommands));
        
        operatorController.leftBumper().whileTrue(climberCommands.incrementUp());
        operatorController.rightBumper().whileTrue(climberCommands.incrementDown());

        operatorController.leftTrigger().whileTrue(endEffectorCommands.intakeAlgae());
        operatorController.rightTrigger().whileTrue(endEffectorCommands.scoreAlgae());

        // Algae Commands
        operatorController.x().onTrue(commandGroups.moveToPickAlgae2(wristCommands, elevatorCommands));
        operatorController.y().onTrue(commandGroups.moveToPickAlgae3(wristCommands, elevatorCommands));
        operatorController.a().onTrue(commandGroups.moveToScoreAlgae(wristCommands, elevatorCommands));
        operatorController.b().onTrue(commandGroups.intakeCoralMinimum(wristCommands, elevatorCommands)); 

        // Manual Elevator Commands
        operatorController.povUp().whileTrue(elevatorCommands.incrementUp());
        operatorController.povDown().whileTrue(elevatorCommands.incrementDown());

        // Manual Wrist Commands
        operatorController.povLeft().whileTrue(wristCommands.incrementIn());
        operatorController.povRight().whileTrue(wristCommands.incrementOut());


        drivetrain.registerTelemetry(logger::telemeterize);

    } // End of configureBindings

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();

    } // End of getAutonomousCommand

} // End of RobotContainer