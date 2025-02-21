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
       // private final double scoreDelay = 1.0;
        }
        public AutoRoutine TwoMeters() {
                final AutoRoutine routine = m_factory.newRoutine("TwoMeters");
                final AutoTrajectory TwoMeters = routine.trajectory("TwoMeters");
        
                routine.active().onTrue(
                        Commands.sequence(
                                TwoMeters.resetOdometry(), // Always reset odometry first
                                TwoMeters.cmd() // Follow the path
                                
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
                                
                        ));
        
                return routine;
        }
        public AutoRoutine ReefSMASH() {
                final AutoRoutine routine = m_factory.newRoutine("IDReefPoses");
                final AutoTrajectory A = routine.trajectory("IDReefPoses",0);
                final AutoTrajectory C = routine.trajectory("IDReefPoses",1);
                final AutoTrajectory E = routine.trajectory("IDReefPoses",2);
                final AutoTrajectory G = routine.trajectory("IDReefPoses",3);
                final AutoTrajectory I = routine.trajectory("IDReefPoses",4);
                final AutoTrajectory K = routine.trajectory("IDReefPoses",5);
               
        
                routine.active().onTrue(
                        Commands.sequence(
                                A.resetOdometry(), // Always reset odometry first
                                A.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                C.cmd(),
                                m_drivetrain.stop().withTimeout(2.0), 
                                E.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                G.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                I.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                K.cmd()
                                
                        ));
        
                return routine;
        }
        public AutoRoutine ReefSMASH2() {
                final AutoRoutine routine = m_factory.newRoutine("IDReefPoses (2)");
                final AutoTrajectory B = routine.trajectory("IDReefPoses (2)",0);
                final AutoTrajectory D = routine.trajectory("IDReefPoses (2)",1);
                final AutoTrajectory F = routine.trajectory("IDReefPoses (2)",2);
                final AutoTrajectory H = routine.trajectory("IDReefPoses (2)",3);
                final AutoTrajectory J = routine.trajectory("IDReefPoses (2)",4);
                final AutoTrajectory L = routine.trajectory("IDReefPoses (2)",5);
               
        
                routine.active().onTrue(
                        Commands.sequence(
                                B.resetOdometry(), // Always reset odometry first
                                B.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                D.cmd(),
                                m_drivetrain.stop().withTimeout(2.0), 
                                F.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                H.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                J.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                L.cmd()
                                
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
                            
                    ));
    
            return routine;
        }
        public AutoRoutine STA3() {
                final AutoRoutine routine = m_factory.newRoutine("ST-A");
                final AutoTrajectory STA = routine.trajectory("ST-A",0);
                final AutoTrajectory STA2 = routine.trajectory("ST-A",1);
                final AutoTrajectory CSL = routine.trajectory("CS1-L",0);
                final AutoTrajectory CSL2 = routine.trajectory("CS1-L",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                STA.resetOdometry(), // Always reset odometry first
                                STA.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                STA2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0), 
                                CSL.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                CSL2.cmd()
                                
                        ));
        
                return routine;
        }
        public AutoRoutine STI() {
                final AutoRoutine routine = m_factory.newRoutine("ST-I");
                final AutoTrajectory STI = routine.trajectory("ST-I",0);
                final AutoTrajectory STI2 = routine.trajectory("ST-I",1);
        
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
        public AutoRoutine SBB() {
                final AutoRoutine routine = m_factory.newRoutine("SB-B");
                final AutoTrajectory SBB = routine.trajectory("SB-B",0);
                final AutoTrajectory SBB2 = routine.trajectory("SB-B",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                SBB.resetOdometry(), // Always reset odometry first
                                SBB.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                SBB2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0)
                              
                        ));
        
                return routine;
        }
        public AutoRoutine SBBetter() {
                final AutoRoutine routine = m_factory.newRoutine("SB-B");
                final AutoTrajectory SBB = routine.trajectory("SB-B",0);
                final AutoTrajectory SBB2 = routine.trajectory("SB-B",1);
                final AutoTrajectory CSC = routine.trajectory("CS2-C",0);
                final AutoTrajectory CSC2 = routine.trajectory("CS2-C",1);
        
                routine.active().onTrue(
                        Commands.sequence(
                                SBB.resetOdometry(), // Always reset odometry first
                                SBB.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(2.0), 
                                SBB2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                CSC.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                CSC2.cmd()
                        ));
        
                return routine;
        }
}
