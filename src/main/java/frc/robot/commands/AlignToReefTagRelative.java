package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivebase;
    
    // Create SwerveRequest for robot-centric control
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();

    // TODO: Tune these values
    /** 
     * represent proportional control constants for the X, Y, and rotational axes, respectively. 
     * These constants are used in proportional control algorithms to adjust the system's position 
     * and orientation based on the error between the current state 
     * and the desired setpoint. */
    private double X_REEF_ALIGNMENT_P = 2; 
    private double Y_REEF_ALIGNMENT_P = 2;
    private double ROT_REEF_ALIGNMENT_P = 0.1;

    /**
     * Define the desired setpoints for the X, Y, and rotational axes. 
     * These setpoints represent the target positions and 
     * orientation that the system aims to achieve during the alignment process.
     */
    // TODO: Test these values
    private double X_SETPOINT_REEF_ALIGNMENT = -0.5; // One meter away from the target?
    private double Y_SETPOINT_REEF_ALIGNMENT = 0.15;
    private double ROT_SETPOINT_REEF_ALIGNMENT = 0.0;
    private double X_TOLERANCE_REEF_ALIGNMENT = 0.1;
    private double Y_TOLERANCE_REEF_ALIGNMENT = 0.1;
    private double ROT_TOLERANCE_REEF_ALIGNMENT = 0.1;
    private double DONT_SEE_TAG_WAIT_TIME = 0.3;
    private double POSE_VALIDATION_TIME = 0.3;

    public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
        xController = new PIDController(X_REEF_ALIGNMENT_P, 0, 0); // Vertical movement
        yController = new PIDController(Y_REEF_ALIGNMENT_P, 0, 0); // Horitontal movement
        rotController = new PIDController(ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
        this.isRightScore = isRightScore;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

        yController
                .setSetpoint(isRightScore ? Y_SETPOINT_REEF_ALIGNMENT : -Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("")) {
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            SmartDashboard.putNumber("x", positions[2]);

            double xSpeed = xController.calculate(positions[2]);
            SmartDashboard.putNumber("xspeed", xSpeed);
            double ySpeed = -yController.calculate(positions[0]);
            double rotValue = -rotController.calculate(positions[4]);

            // Use robot-centric control for simplicity
            if (yController.getError() < Y_TOLERANCE_REEF_ALIGNMENT) {
                // Using CTRE's setControl with RobotCentric request for velocity control
                drivebase.setControl(
                    robotCentricRequest
                        .withVelocityX(xSpeed)  // Forward/backward
                        .withVelocityY(ySpeed)  // Left/right 
                        .withRotationalRate(rotValue)  // Rotation rate
                );
            } else {
                drivebase.setControl(
                    robotCentricRequest
                        .withVelocityX(0)
                        .withVelocityY(ySpeed)
                        .withRotationalRate(rotValue)
                );
            }

            if (!rotController.atSetpoint() ||
                    !yController.atSetpoint() ||
                    !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            // Stop the robot when no tag is visible
            drivebase.setControl(
                robotCentricRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure the robot stops when command ends
        drivebase.setControl(
            robotCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long
        // as it gets a tag in the camera
        return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) ||
                stopTimer.hasElapsed(POSE_VALIDATION_TIME);
    }
}