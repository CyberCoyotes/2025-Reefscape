package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.EndEffectorCommands;

public class AutoRoutines<IntakeSubsystem> {
private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final EffectorSubsystem m_effector;
    private final ElevatorSubsystem m_elevator;
    private final CommandGroups m_groupCommand;
    private final EndEffectorCommands m_effectorCommands;

    public AutoRoutines(
            AutoFactory factory, 
            CommandSwerveDrivetrain drivetrain, 
            EffectorSubsystem effector, 
            ElevatorSubsystem elevator, 
            CommandGroups groupCommand,
            EndEffectorCommands effectorCommands) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_effector = effector;
        m_elevator = elevator;
        m_groupCommand = groupCommand;
        m_effectorCommands = effectorCommands;

        }

        public AutoRoutine STAL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-A-L1");
                final AutoTrajectory STA = routine.trajectory("ST-A-L1", 0);
                final AutoTrajectory STA2 = routine.trajectory("ST-A-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                STA.resetOdometry(), // Always reset odometry first
                                STA.cmd(), // , // Follow the path
                                m_drivetrain.stop().withTimeout(2.0),
                                STA2.cmd()

                        ));
                STA.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                STA.atTime("Load").onTrue(m_effector.intakeCoralNoSensor().withTimeout(2.0));
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
                                m_drivetrain.stop().withTimeout(2.0),
                                STA2.cmd()

                        ));
                STA.atTime("scoreL1").onTrue(m_groupCommand.autoScoreL2());
                STA.atTime("scoreL2").onTrue(m_groupCommand.autoScoreL2());

                STA.atTime("Load").onTrue(m_effector.intakeCoralNoSensor().withTimeout(2.0));
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
                                m_drivetrain.stop().withTimeout(2.0),
                                STA2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                CSL.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
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
                                                m_drivetrain.stop().withTimeout(2.0),
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
                                m_drivetrain.stop().withTimeout(2.0),
                                STI2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0)

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
                                m_drivetrain.stop().withTimeout(2.0),
                                STJ2.cmd()

                        ));
                STJ.atTime("scoreL1").onTrue(m_effector.scoreCoral().withTimeout(1.0));
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
                                m_drivetrain.stop().withTimeout(2.0),
                                STK2.cmd()

                        ));
                STK.atTime("scoreL1").onTrue(m_effector.scoreCoral().withTimeout(1.0));
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
                                m_drivetrain.stop().withTimeout(2.0),
                                STL2.cmd()

                        ));
                STL.atTime("scoreL1").onTrue(m_effector.scoreCoral().withTimeout(1.0));
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
                                m_drivetrain.stop().withTimeout(2.0),
                                SBB2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0)

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
                                m_drivetrain.stop().withTimeout(2.0),
                                SBB2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
                                CSC.cmd(),
                                m_drivetrain.stop().withTimeout(2.0),
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
                                m_drivetrain.stop().withTimeout(2.0),
                                SBC2.cmd()

                        ));
                SBC.atTime("scoreL1").onTrue(m_effector.scoreCoral().withTimeout(1.0));
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
                                m_drivetrain.stop().withTimeout(2.0),
                                SBD2.cmd()

                        ));
                SBD.atTime("scoreL1").onTrue(m_effector.scoreCoral().withTimeout(1.0));
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
                                m_drivetrain.stop().withTimeout(2.0),
                                SBE2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0)

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
                                m_drivetrain.stop().withTimeout(2.0),
                                SBF2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0)

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
                                m_drivetrain.stop().withTimeout(2.0),
                                SBG2.cmd(),
                                m_drivetrain.stop().withTimeout(2.0)

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
                                m_drivetrain.stop().withTimeout(2.0),
                                Back.cmd()

                        ));

                TwoMetersT.atTime("scoreL1").onTrue(m_effector.scoreCoral().withTimeout(1.0));
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

        public AutoRoutine STJtoAL1() {
                final AutoRoutine routine = m_factory.newRoutine("ST-J-L1");
                final AutoTrajectory STJ = routine.trajectory("ST-J-L1", 0);
                final AutoTrajectory STJ2 = routine.trajectory("ST-J-L1", 1);
                final AutoTrajectory CSA = routine.trajectory("CS1-A-L1", 0);
                final AutoTrajectory CSA2 = routine.trajectory("CS1-A-L1", 1);

                routine.active().onTrue(
                                Commands.sequence(
                                        STJ.resetOdometry(), // Always reset odometry first
                                        STJ.cmd(), // , // Follow the path
                                        m_drivetrain.stop().withTimeout(1.5),
                                        STJ2.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA2.cmd()));
                STJ.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                STJ2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(2.0));
                CSA.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSA2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
                return routine;
        }

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
                                        m_drivetrain.stop().withTimeout(1.0),
                                        STJ2.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA.cmd(),
                                        m_drivetrain.stop().withTimeout(1.0),
                                        CSA2.cmd()

                                ));
                STJ.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                STJ2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(2.0));
                CSA.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSA2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
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
                STJ.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                STJ2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(2.0));
                CSA.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSA2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
                CSA3.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSA4.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
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
                SBE.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                SBE2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(2.0));
                CSB.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSB2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
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
                SBE.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                SBE2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(2.0));
                CSB.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSB2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
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
                                SBE.resetOdometry(), // Always reset odometry first
                                SBE.cmd(), // , // Follow the path
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
                SBE.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                SBE2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(2.0));
                CSB.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSB2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
                CSB3.atTime("scoreL1").onTrue(m_effector.intakeCoralNoSensor().withTimeout(1.0));
                CSB4.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(1.0));
                return routine;
        }

        public AutoRoutine MHL1() {
                final AutoRoutine routine = m_factory.newRoutine("Mid-H-L1");
                final AutoTrajectory MH = routine.trajectory("Mid-H-L1", 0);
                final AutoTrajectory MH2 = routine.trajectory("Mid-H-L1", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                MH.resetOdometry(), // Always reset odometry first
                                MH.cmd(), // Follow the path
                                m_drivetrain.stop().withTimeout(1.0),
                                MH2.cmd()

                        ));
                MH.atTime("scoreL1").onTrue(m_effector.scoreCoral().withTimeout(1.0));
                MH2.atTime("Load").onTrue(m_effector.slowCoral().withTimeout(2.0));
                return routine;
        }
}
