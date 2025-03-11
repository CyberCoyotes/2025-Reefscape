package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AlignWithTagCommand;
import frc.robot.commands.SeekAprilTagCommand;
import frc.robot.commands.SmartTagAlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.CameraSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  
  /* Vision Alignment Buttons */
  private final JoystickButton seekTagButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton alignLeftButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton alignRightButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton smartAlignButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  /* Subsystems */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final VisionSubsystem vision = new VisionSubsystem("limelight", drivetrain);
  private final CameraSubsystem camera = new CameraSubsystem();

  /* Driver control requests */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Max speeds */
  private final double MaxSpeed = 6; // 6 meters per second
  private final double MaxAngularRate = 1.5 * Math.PI; // 1.5 rotations per second

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDriverBindings();
    configureDashboard();
  }

  private void configureDriverBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          double xSpeed = -driver.getRawAxis(translationAxis) * MaxSpeed;
          double ySpeed = -driver.getRawAxis(strafeAxis) * MaxSpeed;
          double rotSpeed = -driver.getRawAxis(rotationAxis) * MaxAngularRate;

          /* Deadbands */
          xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
          ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed;
          rotSpeed = Math.abs(rotSpeed) < 0.1 ? 0 : rotSpeed;

          /* Robot vs Field Centric */
          if (robotCentric.getAsBoolean()) {
            return robotCentricDrive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotSpeed);
          } else {
            return drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotSpeed);
          }
        }));

    // Zero the gyro when Y is pressed
    
    // Vision alignment commands
    seekTagButton.whileTrue(new SeekAprilTagCommand(vision, drivetrain));
    alignLeftButton.whileTrue(new AlignWithTagCommand(vision, drivetrain, false)); // align to left
    alignRightButton.whileTrue(new AlignWithTagCommand(vision, drivetrain, true)); // align to right
    smartAlignButton.whileTrue(new SmartTagAlignCommand(vision, drivetrain));
  }
  
  private void configureDashboard() {
    // Add vision and alignment info to the dashboard
    SmartDashboard.putData("Vision Subsystem", vision);
    SmartDashboard.putData("Seek AprilTag", new SeekAprilTagCommand(vision, drivetrain));
    SmartDashboard.putData("Align Left of Tag", new AlignWithTagCommand(vision, drivetrain, false));
    SmartDashboard.putData("Align Right of Tag", new AlignWithTagCommand(vision, drivetrain, true));
    SmartDashboard.putData("Smart Tag Alignment", new SmartTagAlignCommand(vision, drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivetrain.stop();
  }
}