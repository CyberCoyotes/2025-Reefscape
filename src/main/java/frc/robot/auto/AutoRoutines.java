package frc.robot.auto;
// https://claude.ai/chat/f8059a9b-ef64-431c-aa67-7e5b7fb7e7dc Troubleshoot
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

@SuppressWarnings("unused")

public class AutoRoutines {
    /** 
     * This class contains all the auto routines for the robot. 
     * Each routine is defined as a method that returns an AutoRoutine object.
     * The methods use the AutoFactory to create trajectories and commands for the robot's subsystems.
     */
    
    // Subsystems and command groups used in the auto routines

    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final EffectorSubsystem m_effector;
    private final ElevatorSubsystem m_elevator;
    private final ElevatorCommands m_elevatorCommands;
    private final CommandGroups m_commandGroups;
    private final EndEffectorCommands m_effectorCommands;
    private final WristSubsystem m_wrist;
    private final WristCommands m_wristCommands;

    // How long to wait after driving before doing something else
    private final double DRIVE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less 

    // Time to wait for the elevator to move and score.
    private final double ELEVATOR_WAIT = 4.2; // 4.2 -> 1.0
    
    // Time to wait for the effector to load.
    private final double LOAD_WAIT = 2; // 2.0 -> 1.0
    private final double SCORE_WAIT = 1.0; // 1.0 -> 0.5

    public AutoRoutines(
        AutoFactory autoFactory,
        CommandSwerveDrivetrain drivetrain,
        EffectorSubsystem effector,
        ElevatorSubsystem elevator,
        ElevatorCommands elevatorCommands,
        CommandGroups commandGroups,
        EndEffectorCommands effectorCommands,
        WristSubsystem wrist,
        WristCommands wristCommands) {
            m_factory = autoFactory;
            m_drivetrain = drivetrain;
            m_effector = effector;
            m_elevator = elevator;
            m_elevatorCommands = elevatorCommands;
            m_commandGroups = commandGroups;
            m_effectorCommands = effectorCommands;
            m_wrist = wrist;
            m_wristCommands = wristCommands;
    }

        public AutoRoutine STAL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-A-L1");
                final AutoTrajectory STA = routine.trajectory("ST-A-L1", 0);
                final AutoTrajectory STA2 = routine.trajectory("ST-A-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STA.resetOdometry(), // Always reset odometry first
                                STA.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                STA2.cmd()

                        ));
                STA.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(SCORE_WAIT));

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                STA.atTime("Load").onTrue(m_effectorCommands.intakeCoral().withTimeout(LOAD_WAIT));
                return routine;
        }

        /** Added on Sunday 
         * TEST first
        */
        public AutoRoutine STAL2() {
                final AutoRoutine routine = m_factory.newRoutine("ST-A-L1");
                final AutoTrajectory STA = routine.trajectory("ST-A-L1", 0);
                final AutoTrajectory STA2 = routine.trajectory("ST-A-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STA.resetOdometry(),
                                STA.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                STA2.cmd()

                        ));
                STA.atTime("scoreL1").onTrue(m_commandGroups.autoScoreL2());

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                STA.atTime("Load").onTrue(m_effectorCommands.intakeCoral().withTimeout(LOAD_WAIT));
                return routine;
        }
        
        

        public AutoRoutine STAtoL() {
                final AutoRoutine routine = m_factory.newRoutine("ST-A");
                final AutoTrajectory STA = routine.trajectory("ST-A", 0);
                final AutoTrajectory STA2 = routine.trajectory("ST-A", 1);
                final AutoTrajectory CSL = routine.trajectory("CS1-L", 0);
                final AutoTrajectory CSL2 = routine.trajectory("CS1-L", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STA.resetOdometry(), // Always reset odometry first
                                STA.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                STA2.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                CSL.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                CSL2.cmd()

                        ));

                return routine;
        }

        public AutoRoutine STH() {
                final AutoRoutine routine = m_factory.newRoutine("ST-H");
                final AutoTrajectory STH = routine.trajectory("ST-H", 0);
                final AutoTrajectory STH2 = routine.trajectory("ST-H", 1);

                routine.active().onTrue(
                                Commands.sequence(
                                                STH.resetOdometry(), // Always reset odometry first
                                                STH.cmd(), // Follow the path
                                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                                STH2.cmd()

                                ));

                return routine;
        }

        public AutoRoutine STI() {
                final AutoRoutine routine = m_factory.newRoutine("ST-I");
                final AutoTrajectory STI = routine.trajectory("ST-I", 0);
                final AutoTrajectory STI2 = routine.trajectory("ST-I", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STI.resetOdometry(), // Always reset odometry first
                                STI.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                STI2.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT)

                        ));

                return routine;
        }

        public AutoRoutine STJL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J-L1");
                final AutoTrajectory STJ = routine.trajectory("ST-J-L1", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STJ.resetOdometry(), // Always reset odometry first
                                STJ.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                STJ2.cmd()

                        ));
                STJ.atTime("scoreL1").onTrue(m_effectorCommands.scoreCoral().withTimeout(SCORE_WAIT));
                return routine;
        }

        public AutoRoutine STKL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-K-L1");
                final AutoTrajectory STK = routine.trajectory("ST-K-L1", 0);
                final AutoTrajectory STK2 = routine.trajectory("ST-K-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STK.resetOdometry(), // Always reset odometry first
                                STK.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                STK2.cmd()

                        ));
                STK.atTime("scoreL1").onTrue(m_effectorCommands.scoreCoral().withTimeout(SCORE_WAIT));
                return routine;
        }

        public AutoRoutine STLL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-L-L1");
                final AutoTrajectory STL = routine.trajectory("ST-L-L1", 0);
                final AutoTrajectory STL2 = routine.trajectory("ST-L-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STL.resetOdometry(), // Always reset odometry first
                                STL.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                STL2.cmd()

                        ));
                STL.atTime("scoreL1").onTrue(m_effectorCommands.scoreCoral().withTimeout(SCORE_WAIT));
                return routine;
        }

        public AutoRoutine SBB() {
                final AutoRoutine routine = m_factory.newRoutine("SB-B");
                final AutoTrajectory SBB = routine.trajectory("SB-B", 0);
                final AutoTrajectory SBB2 = routine.trajectory("SB-B", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBB.resetOdometry(), // Always reset odometry first
                                SBB.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                SBB2.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT)

                        ));

                return routine;
        }

        public AutoRoutine SBBtoC() {
                final AutoRoutine routine = m_factory.newRoutine("SB-B");
                final AutoTrajectory SBB = routine.trajectory("SB-B", 0);
                final AutoTrajectory SBB2 = routine.trajectory("SB-B", 1);
                final AutoTrajectory CSC = routine.trajectory("CS2-C", 0);
                final AutoTrajectory CSC2 = routine.trajectory("CS2-C", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBB.resetOdometry(), // Always reset odometry first
                                SBB.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                SBB2.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                CSC.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                CSC2.cmd()));

                return routine;
        }

        public AutoRoutine SBCL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-C-L1");
                final AutoTrajectory SBC = routine.trajectory("ST-C-L1", 0);
                final AutoTrajectory SBC2 = routine.trajectory("ST-C-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBC.resetOdometry(), // Always reset odometry first
                                SBC.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                SBC2.cmd()

                        ));
                SBC.atTime("scoreL1").onTrue(m_effectorCommands.scoreCoral().withTimeout(SCORE_WAIT));
                return routine;
        }

        public AutoRoutine SBDL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-D-L1");
                final AutoTrajectory SBD = routine.trajectory("ST-D-L1", 0);
                final AutoTrajectory SBD2 = routine.trajectory("ST-D-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBD.resetOdometry(), // Always reset odometry first
                                SBD.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                SBD2.cmd()

                        ));
                SBD.atTime("scoreL1").onTrue(m_effectorCommands.scoreCoral().withTimeout(SCORE_WAIT));
                return routine;
        }

        public AutoRoutine SBE() {
                final AutoRoutine routine = m_factory.newRoutine("SB-E");
                final AutoTrajectory SBE = routine.trajectory("SB-E", 0);
                final AutoTrajectory SBE2 = routine.trajectory("SB-E", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBE.resetOdometry(), // Always reset odometry first
                                SBE.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                SBE2.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT)

                        ));

                return routine;
        }

        public AutoRoutine SBF() {
                final AutoRoutine routine = m_factory.newRoutine("SB-F");
                final AutoTrajectory SBF = routine.trajectory("SB-F", 0);
                final AutoTrajectory SBF2 = routine.trajectory("SB-F", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBF.resetOdometry(), // Always reset odometry first
                                SBF.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                SBF2.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT)

                        ));

                return routine;
        }

        public AutoRoutine SBG() {
                final AutoRoutine routine = m_factory.newRoutine("SB-G");
                final AutoTrajectory SBG = routine.trajectory("SB-G", 0);
                final AutoTrajectory SBG2 = routine.trajectory("SB-G", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBG.resetOdometry(), // Always reset odometry first
                                SBG.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                SBG2.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT)

                        ));

                return routine;
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

        public AutoRoutine ScoreTwoMetersBack() {
                final AutoRoutine routine = m_factory.newRoutine("TwoMetersBack");
                final AutoTrajectory TwoMetersT = routine.trajectory("TwoMetersBack", 0);
                final AutoTrajectory Back = routine.trajectory("TwoMetersBack", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                TwoMetersT.resetOdometry(), // Always reset odometry first
                                TwoMetersT.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                Back.cmd()

                        ));

                TwoMetersT.atTime("scoreL1").onTrue(m_effectorCommands.scoreCoral().withTimeout(SCORE_WAIT));
                return routine;
        }

        public AutoRoutine ReefSMASH() {
                final AutoRoutine routine = m_factory.newRoutine("IDReefPoses");
                final AutoTrajectory A = routine.trajectory("IDReefPoses", 0);
                final AutoTrajectory C = routine.trajectory("IDReefPoses", 1);
                final AutoTrajectory E = routine.trajectory("IDReefPoses", 2);
                final AutoTrajectory G = routine.trajectory("IDReefPoses", 3);
                final AutoTrajectory I = routine.trajectory("IDReefPoses", 4);
                final AutoTrajectory K = routine.trajectory("IDReefPoses", 5);

                routine.active().onTrue(
                        Commands.sequence(
                                A.resetOdometry(), // Always reset odometry first
                                A.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                C.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                E.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                G.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                I.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                K.cmd()

                        ));

                return routine;
        }

        public AutoRoutine ReefSMASH2() {
                final AutoRoutine routine = m_factory.newRoutine("IDReefPoses (2)");
                final AutoTrajectory B = routine.trajectory("IDReefPoses (2)", 0);
                final AutoTrajectory D = routine.trajectory("IDReefPoses (2)", 1);
                final AutoTrajectory F = routine.trajectory("IDReefPoses (2)", 2);
                final AutoTrajectory H = routine.trajectory("IDReefPoses (2)", 3);
                final AutoTrajectory J = routine.trajectory("IDReefPoses (2)", 4);
                final AutoTrajectory L = routine.trajectory("IDReefPoses (2)", 5);

                routine.active().onTrue(
                        Commands.sequence(
                                B.resetOdometry(), // Always reset odometry first
                                B.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                D.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                F.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                H.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                J.cmd(),
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                L.cmd()

                        ));

                return routine;
        }

        public AutoRoutine STJtoAL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J-L1");
                // 
                final AutoTrajectory STJ = routine.trajectory("ST-J-L1", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J-L1", 1);

                final AutoTrajectory CSA = routine.trajectory("CS1-A-L1", 0);
                final AutoTrajectory CSA2 = routine.trajectory("CS1-A-L1", 1);

                routine.active().onTrue(
                                Commands.sequence(
                                        STJ.resetOdometry(), // Always reset odometry first
                                        STJ.cmd(), // , // Follow the path
                                        m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                        STJ2.cmd(),
                                        m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                        CSA.cmd(),
                                        m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                        CSA2.cmd()));
                STJ.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(SCORE_WAIT));

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                STJ2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(LOAD_WAIT));
                CSA.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(SCORE_WAIT));

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                CSA2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(LOAD_WAIT));
                return routine;
        }

        // J L1 to A L2
        public AutoRoutine STJtoAL12() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J-L1");
                final AutoTrajectory STJ = routine.trajectory("ST-J-L1", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J-L1", 1);
                final AutoTrajectory CSA = routine.trajectory("CS1-A", 0);
                final AutoTrajectory CSA2 = routine.trajectory("CS1-A", 1);

                routine.active().onTrue(
                                Commands.sequence(
                                        STJ.resetOdometry(), // Always reset odometry first
                                        STJ.cmd(), // Follow the path
                                        m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                        STJ2.cmd(),
                                        m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                        CSA.cmd(),
                                        m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                        CSA2.cmd()

                                ));
                STJ.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(SCORE_WAIT));

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                STJ2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(2.0));
                CSA.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                CSA2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(1.0));
                return routine;
        }
        public AutoRoutine STJtoAL1AL2() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J-L1");
                final AutoTrajectory STJ = routine.trajectory("ST-J-L1", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J-L1", 1);
                final AutoTrajectory CSA = routine.trajectory("CS1-A-L1", 0);
                final AutoTrajectory CSA2 = routine.trajectory("CS1-A-L1", 1);
                final AutoTrajectory CSA3 = routine.trajectory("CS1-A-L1", 0);
                final AutoTrajectory CSA4 = routine.trajectory("CS1-A-L1", 1);

                routine.active().onTrue(
                                Commands.sequence(
                                        STJ.resetOdometry(), // Always reset odometry first
                                        STJ.cmd(), // Follow the path
                                        m_drivetrain.stop().withTimeout(1.0),
                                        STJ2.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA2.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA3.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA4.cmd()

                                ));
                STJ.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                STJ2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(2.0));
                CSA.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                CSA2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(1.0));
                CSA3.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                CSA4.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(1.0));
                return routine;
        }

        public AutoRoutine SBEtoBL1() {
                final AutoRoutine routine = m_factory.newRoutine("SB-E-L1");
                final AutoTrajectory SBE = routine.trajectory("SB-E-L1", 0);
                final AutoTrajectory SBE2 = routine.trajectory("SB-E-L1", 1);
                final AutoTrajectory CSB = routine.trajectory("CS2-B-L1", 0);
                final AutoTrajectory CSB2 = routine.trajectory("CS2-B-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBE.resetOdometry(), // Always reset odometry first
                                SBE.cmd(), // , // Follow the path
                                m_drivetrain.stop().withTimeout(1.5),
                                SBE2.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB2.cmd()));
                SBE.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)                
                SBE2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(2.0));
                CSB.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                CSB2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(1.0));
                return routine;
        }

        public AutoRoutine SBEtoBL12() {
                final AutoRoutine routine = m_factory.newRoutine("SB-E-L1");
                final AutoTrajectory SBE = routine.trajectory("SB-E-L1", 0);
                final AutoTrajectory SBE2 = routine.trajectory("SB-E-L1", 1);
                final AutoTrajectory CSB = routine.trajectory("CS2-B", 0);
                final AutoTrajectory CSB2 = routine.trajectory("CS2-B", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBE.resetOdometry(), // Always reset odometry first
                                SBE.cmd(), // , // Follow the path
                                m_drivetrain.stop().withTimeout(1.0),
                                SBE2.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB2.cmd()));
                SBE.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                SBE2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(2.0));
                CSB.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                CSB2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(1.0));
                return routine;
        }
        public AutoRoutine SBEtoBL1BL2() {
                final AutoRoutine routine = m_factory.newRoutine("SB-E-L1");
                final AutoTrajectory SBE = routine.trajectory("SB-E-L1", 0);
                final AutoTrajectory SBE2 = routine.trajectory("SB-E-L1", 1);
                final AutoTrajectory CSB = routine.trajectory("CS2-B-L1", 0);
                final AutoTrajectory CSB2 = routine.trajectory("CS2-B-L1", 1);
                final AutoTrajectory CSB3 = routine.trajectory("CS2-B", 0);
                final AutoTrajectory CSB4 = routine.trajectory("CS2-B", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBE.resetOdometry(),
                                SBE.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                SBE2.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB2.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB3.cmd(),
                                m_drivetrain.stop().withTimeout(1.0),
                                CSB4.cmd()
                                ));
                SBE.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                SBE2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(2.0));
                CSB.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));
                CSB2.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(1.0));
                CSB3.atTime("scoreL1").onTrue(m_effectorCommands.intakeCoral().withTimeout(1.0));                
                CSB4.atTime("Load").onTrue(m_effectorCommands.scoreCoralSlow().withTimeout(1.0));
                return routine;
        }

        public AutoRoutine MH() {
                final AutoRoutine routine = m_factory.newRoutine("Mid-H");
                final AutoTrajectory MH = routine.trajectory("Mid-H");
        

                routine.active().onTrue(
                        Commands.sequence(
                                MH.resetOdometry(), // Always reset odometry first
                                MH.cmd(), // Follow the path
                                m_commandGroups.stopUntilCoralReleased(6.0),
                                m_drivetrain.stop().withTimeout(6.0)

                        ));
                MH.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());
                
                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)                
                //MH2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                return routine;
        }
        public AutoRoutine STJL4() {
                final AutoRoutine routine = m_factory.newRoutine("ST-JL4");
                final AutoTrajectory STJ = routine.trajectory("ST-J", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STJ.resetOdometry(),
                                STJ.cmd(),
                                m_commandGroups.stopUntilCoralReleased(6.0),
                                STJ2.cmd()

                        ));
                   STJ.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());
                   return routine;
        }
        public AutoRoutine STAL4() {
                final AutoRoutine routine = m_factory.newRoutine("ST-AL4");
                final AutoTrajectory STA = routine.trajectory("ST-A", 0);
                //final AutoTrajectory STA2 = routine.trajectory("ST-A", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STA.resetOdometry(),
                                STA.cmd(),
                                m_commandGroups.stopUntilCoralReleased(6.0)//,
                                //STA2.cmd()

                        ));
                STA.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());

                STA.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                return routine;
        }
        public AutoRoutine SBEL4() {
                final AutoRoutine routine = m_factory.newRoutine("SB-EL4");
                final AutoTrajectory SBE = routine.trajectory("SB-E", 0);
                //final AutoTrajectory SBE2 = routine.trajectory("SB-E", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBE.resetOdometry(),
                                SBE.cmd(),
                                m_commandGroups.stopUntilCoralReleased(6.0)//,
                                //SBE2.cmd()

                        ));
                SBE.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                //SBE2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral().withTimeout(6.0));
                return routine;
        }
        public AutoRoutine SBBL4() {
                final AutoRoutine routine = m_factory.newRoutine("SB-BL4");
                final AutoTrajectory SBB = routine.trajectory("SB-B", 0);
                //final AutoTrajectory SBB2 = routine.trajectory("SB-B", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                SBB.resetOdometry(),
                                SBB.cmd(),
                                m_commandGroups.stopUntilCoralReleased(6.0)//,
                                //SBB2.cmd()

                        ));
                SBB.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());

                // Consider using m_commandGroups.autoIntakeCoral(m_wristCommands, m_elevatorCommands, m_wrist)
                //SBB2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral().withTimeout(6.0));
                return routine;
        }
        public AutoRoutine STJ4toAL4() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J->CS1-A");
                final AutoTrajectory STJ = routine.trajectory("ST-J", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J", 1);
                final AutoTrajectory CSA = routine.trajectory("CS1-A-Speedy", 0);
                final AutoTrajectory CSA2 = routine.trajectory("CS1-A-Speedy", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STJ.resetOdometry(),
                                //  Drives from Start to Branch J, stops & waits, and scores L4
                                STJ.cmd(),
                                m_commandGroups.stopUntilCoralReleased(4.5),
                                // Drives from Branch J to Coral Station, stops & waits to load
                                STJ2.cmd(),             
                                m_commandGroups.stopUntilCoralLoaded(4.5),
                                // Drives from Coral Station to Branch A, stops & waits to score L4
                                CSA.cmd(),
                                m_commandGroups.stopUntilCoralReleased(4.5),
                                // Drives from Branch A to Coral Station, stops & waits to load
                                CSA2.cmd()
                        ));
                STJ.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());
                STJ2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                CSA.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());
                CSA2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                return routine;
        }


         /************************************************
         * `LeftSideRoadRunner` auto routine
         *************************************************/
        public AutoRoutine LeftSideRoadRunner() {
                final AutoRoutine routine = m_factory.newRoutine("LeftRoadRunner");
                final AutoTrajectory STJS1 = routine.trajectory("ST-J-Speedy", 0);
                final AutoTrajectory STJS2 = routine.trajectory("ST-J-Speedy", 1);
                final AutoTrajectory CSSA1 = routine.trajectory("CS1-A-Speedy", 0);
                final AutoTrajectory CSSA2 = routine.trajectory("CS1-A-Speedy", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STJS1.resetOdometry(),

                                //  Drives from Start to Branch J
                                STJS1.cmd(),
                                    
                                m_commandGroups.stopUntilCoralReleased(6.0),
                                
                                // Drives from Branch J to Coral Station, stops & waits to load
                                STJS2.cmd(),             
                                
                                m_commandGroups.stopUntilCoralLoaded(6.0),
                                
                                // Drives from Coral Station to Branch A, stops & waits to score L4
                                CSSA1.cmd(),
                                
                                m_commandGroups.stopUntilCoralReleased(6.0),
                                
                                // Drives from Branch A to Coral Station, stops & waits to load
                                CSSA2.cmd()
                        ));

                // STJ.atTime("preScore").onTrue(m_commandGroups.autoPreScore());
                STJS1.atTime("score").onTrue(m_commandGroups.autoRoadRunnerL4());

                // STJ2.atTime("postScore").onTrue(m_commandGroups.moveToTravel());
                STJS2.atTime("load").onTrue(m_commandGroups.autoIntakeCoral());
                CSSA1.atTime("score").onTrue(m_commandGroups.autoRoadRunnerL4());
                CSSA2.atTime("load").onTrue(m_commandGroups.autoIntakeCoral());
                return routine;
        }

         /************************************************
         * `LeftSideBeepBeep` auto routine
         *************************************************/
        public AutoRoutine LeftSideBeepBeep() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J->CS1-A");
                final AutoTrajectory STJ = routine.trajectory("ST-J", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J", 1);
                final AutoTrajectory CSA = routine.trajectory("CS1-A", 0);
                final AutoTrajectory CSA2 = routine.trajectory("CS1-A", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STJ.resetOdometry(),                                
                                
                                //  Drives from Start to Branch J
                                STJ.cmd(),                                

                                m_commandGroups.stopUntilCoralReleased(6),

                                // Drives from Branch J to Coral Station, stops & waits to load
                                STJ2.cmd(),             

                                m_commandGroups.stopUntilCoralLoaded(6),

                                // Drives from Coral Station to Branch A, stops & waits to score L4
                                CSA.cmd(),

                                m_commandGroups.stopUntilCoralReleased(6),

                                // Drives from Branch A to Coral Station, stops & waits to load
                                CSA2.cmd()
                        ));
                STJ.atTime("scoreL1").onTrue(m_commandGroups.autoBeepBeepL4());
                STJ2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                CSA.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());
                CSA2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                return routine;
        }

        public AutoRoutine SBE4toBL4() {
                final AutoRoutine routine = m_factory.newRoutine("SB-EL4");
                final AutoTrajectory SBE = routine.trajectory("SB-E", 0);
                final AutoTrajectory SBE2 = routine.trajectory("SB-E", 1);
                final AutoTrajectory CSB = routine.trajectory("CS2-B", 0);
                final AutoTrajectory CSB2 = routine.trajectory("CS2-B", 1);
                routine.active().onTrue(
                        Commands.sequence(
                                SBE.resetOdometry(),
                                SBE.cmd(),
                                m_commandGroups.stopUntilCoralReleased(6.0),
                                SBE2.cmd(),
                                m_commandGroups.stopUntilCoralLoaded(6.0),
                                CSB.cmd(),
                                m_commandGroups.stopUntilCoralReleased(6.0),
                                CSB2.cmd()

                        ));
                SBE.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());
                SBE2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                CSB.atTime("scoreL1").onTrue(m_commandGroups.autoRoadRunnerL4());
                CSB2.atTime("Load").onTrue(m_commandGroups.autoIntakeCoral());
                return routine;
        }
}
