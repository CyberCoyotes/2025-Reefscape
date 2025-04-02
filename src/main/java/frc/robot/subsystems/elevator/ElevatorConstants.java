package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

@SuppressWarnings("unused")

public final class ElevatorConstants {
    
    public static final double GEAR_RATIO = 9.0;

    // Control Parameters
    public static final double DEADBAND = 0.02;
    // public static final double POSITION_TOLERANCE = 0.02; // Deprecated?

    /******************************
     *  Motion Magic 
     ******************************/ 
    // Tested 160 previously
    public static final double VELOCITY = 120; // Rotations per second
    // Tested 160 previously
    public static final double ACCELERATION = 120; // Rotations per second^2
    
    public static final double JERK = 500; // Rotations per second^3
    
    // PID Values
    // kP tested 8 previously
    public static final double kP = 6.00;
    public static final double kI = 0.01;
    public static final double kD = 0.10;
    public static final double kS = 0.50;
    public static final double kV = 0.12;
    public static final double kG = 0.50;

    
    // Soft Limits (in rotations)
    // This will need updating when new build
    public static final double FORWARD_LIMIT = 4.75;
    public static final double REVERSE_LIMIT = 0;

    
    public class TestMode {
        
        public static final double kP = 1.0;
        public static final double kI = 0.01;
        public static final double kD = 0.10;
        public static final double kS = 0.50;
        public static final double kV = 0.12;
        public static final double kG = 0.50;

        public static final double CRUISE_VELOCITY = 40; // Rotations per second
        public static final double ACCELERATION = 40; // Rotations per second^2
        public static final double JERK = 200; // Rotations per second^3
    }

     private final Slot1Configs safetyGains = new Slot1Configs()
            .withKP(1.0)
            .withKI(0.01)
            .withKD(0.10)
            .withKS(0.25)
            .withKV(0.12)
            .withKG(0.25)
            .withGravityType(GravityTypeValue.Elevator_Static);

    private final Slot2Configs incrementalGains = new Slot2Configs()
            .withKP(1.0)
            .withKI(0.01)
            .withKD(0.10)
            .withKS(0.25)
            .withKV(0.12)
            .withKG(0.12)
            .withGravityType(GravityTypeValue.Elevator_Static);
    
    public static final double INCREMENT_VALUE = 0.1; // Rotations per execution
    public static final double MAX_INCREMENT = 2.0; // Maximum increment "speed" in rotations per second

    // Elevator Positions
    public enum ElevatorPosition {
        HOME(0.00),
        L1(0.00),
        SCORE_ALGAE(0.45), // Previously .5
        L2(0.45),
        ALGAE2(1.3), // Previously was 1.1 but too low
        L3(1.85),
        INTAKE_CORAL(0.15),
        TRAVEL(1.92),
        ALGAE3(2.65), // previously was 2.45 but too low
        L4(4.2); // Previously 4.7 // 4.6 too tall at practice field

        private final double position;

        ElevatorPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }
}
