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
        // m_elevator = new ElevatorSubsystem(); 
        // m_intake = new IntakeSubsystem(); 
        // m_score = new ScoreSubsystem(); 
       // private final double scoreDelay = 1.0;
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
                final AutoTrajectory TwoMetersT = routine.trajectory("TwoMetersBack",0);
                final AutoTrajectory Back = routine.trajectory("TwoMetersBack",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                TwoMetersT.resetOdometry(), // Always reset odometry first
                                TwoMetersT.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0),
                                Back.cmd()
                                //If this doesnt work m_drivetrain.stop().withTimeout(scoreDelay)
                                //TwoMetersBack.done() // TODO add a Done command
                        ));
        
                return routine;
        }
        public AutoRoutine STA() {
            final AutoRoutine routine = m_factory.newRoutine("ST-A");
            final AutoTrajectory STA = routine.trajectory("ST-A",0);
            final AutoTrajectory STA2 = routine.trajectory("ST-A",1);
    
            routine.active().onTrue(
                    Commands.sequence(
                            STA.resetOdometry(), // Always reset odometry first
                            STA.cmd(), // Follow the path
                            m_drivetrain.stop().withTimeout(2.0), 
                            STA2.cmd()
                            // STA.done() // TODO add a Done command
                    ));
    
            return routine;
        }
        public AutoRoutine STA3() {
                final AutoRoutine routine = m_factory.newRoutine("ST-A");
                final AutoTrajectory STA = routine.trajectory("ST-A",0);
                final AutoTrajectory STA2 = routine.trajectory("ST-A",1);
                final AutoTrajectory STL = routine.trajectory("CS1-L",0);
                final AutoTrajectory STL2 = routine.trajectory("CS1-L",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                STA.resetOdometry(), // Always reset odometry first
                                STA.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                STA2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0), 
                                STL.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                STL2.cmd()
                                // STA.done() // TODO add a Done command
                        ));
        
                return routine;
        }
        public AutoRoutine STI() {
                final AutoRoutine routine = m_factory.newRoutine("ST-I");
                final AutoTrajectory STI = routine.trajectory("ST-I",0);
                final AutoTrajectory STI2 = routine.trajectory("ST-I,1",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                STI.resetOdometry(), // Always reset odometry first
                                STI.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                STI2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0)
                              
                        ));
        
                return routine;
        }
            public AutoRoutine STJ() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J");
                final AutoTrajectory STJ = routine.trajectory("ST-J",0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                STJ.resetOdometry(), // Always reset odometry first
                                STJ.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                STJ2.cmd()//,
                                //m_drivetrain.stop().withTimeout(2.0)
                              
                        ));
        
                return routine;
        }
        public AutoRoutine STL() {
                final AutoRoutine routine = m_factory.newRoutine("ST-L");
                final AutoTrajectory STL = routine.trajectory("ST-L",0);
                final AutoTrajectory STL2 = routine.trajectory("ST-L,",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                STL.resetOdometry(), // Always reset odometry first
                                STL.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                STL2.cmd()//,
                               // m_drivetrain.stop().withTimeout(2.0)
                              
                        ));
        
                return routine;
        }
}
