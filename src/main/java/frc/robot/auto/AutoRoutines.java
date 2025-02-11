package frc.robot.auto;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        // m_elevator = new ElevatorSubsystem(); // Competition robot only
        // m_intake = new IntakeSubsystem(); // Competition robot only
        // m_score = new ScoreSubsystem(); // Competition robot only
        }
        public AutoRoutine Test() {
            final AutoRoutine routine = m_factory.newRoutine("ST-A");
            final AutoTrajectory STA = routine.trajectory("ST-A");
    
            routine.active().onTrue(
                    Commands.sequence(
                            STA.resetOdometry(), // Always reset odometry first
                            STA.cmd() // Follow the path
                            // driveForward.done() // TODO add a done command
                    ));
    
            return routine;
        }
}
