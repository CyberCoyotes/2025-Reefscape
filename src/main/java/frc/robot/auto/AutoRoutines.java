package frc.robot.auto;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        // m_elevator = new ElevatorSubsystem(); 
        // m_intake = new IntakeSubsystem(); 
        // m_score = new ScoreSubsystem(); 
        }
        public AutoRoutine TwoMeters() {
                final AutoRoutine routine = m_factory.newRoutine("TwoMeters");
                final AutoTrajectory TwoMeters = routine.trajectory("TwoMeters");
        
                routine.active().onTrue(
                        Commands.sequence(
                                TwoMeters.resetOdometry(), // Always reset odometry first
                                TwoMeters.cmd() // Follow the path
                                // TwoMeters.done() // TODO add a Done command
                        ));
        
                return routine;
        }
        public AutoRoutine TwoMetersBack() {
                final AutoRoutine routine = m_factory.newRoutine("TwoMetersBack");
                final AutoTrajectory TwoMetersBack = routine.trajectory("TwoMetersBack");
        
                routine.active().onTrue(
                        Commands.sequence(
                                TwoMetersBack.resetOdometry(), // Always reset odometry first
                                TwoMetersBack.cmd() // Follow the path
                                //TwoMetersBack.done() // TODO add a Done command
                        ));
        
                return routine;
        }
        public AutoRoutine STA() {
            final AutoRoutine routine = m_factory.newRoutine("ST-A");
            final AutoTrajectory STA = routine.trajectory("ST-A");
    
            routine.active().onTrue(
                    Commands.sequence(
                            STA.resetOdometry(), // Always reset odometry first
                            STA.cmd() // Follow the path
                            // STA.done() // TODO add a Done command
                    ));
    
            return routine;
        }
}
