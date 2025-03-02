package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
    
    public static final double GEAR_RATIO = 9.0;
    
    /* 
    |   Vel |   Acc |   Jer |   Notes
    |-------|-------|-------|-----------------------
    |   160 |   160 |   500 |   Performance Mode
    |   20  |   20  |   100 |   Safety Mode
    */ 
    

    // Motion Magic
    public class EventMode {
        public static final double CRUISE_VELOCITY = 40; // Rotations per second
        public static final double ACCELERATION = 40; // Rotations per second^2
        public static final double JERK = 300; // Rotations per second^3
    }
    
    // Motion Magic
    public class TestMode {
        public static final double CRUISE_VELOCITY = 20; // Rotations per second
        public static final double ACCELERATION = 20; // Rotations per second^2
        public static final double JERK = 200; // Rotations per second^3
        
    }

    // Control Parameters
    public static final double DEADBAND = 0.02;
    public static final double POSITION_TOLERANCE = 0.02;
}