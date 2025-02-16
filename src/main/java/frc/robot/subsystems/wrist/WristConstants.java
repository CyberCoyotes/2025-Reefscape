// WristConstants.java
package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitValue;

public final class WristConstants {
    // CAN ID and hardware config
    public static final int WRIST_ID = 20;
    public static final int WRIST_CANCODER_ID = 14;
    public static final CANBus kCANBUS = new CANBus("rio");
    public static final double GEAR_RATIO = 50.0; // TODO Determine the actual gear ratio; suggested 75.0
    public static final double CANCODER_TO_MECHANISM_RATIO = 1.0; 

    // Mechanical configuration
    public static final boolean MOTOR_INVERTED = false;
    public static final boolean CANCODER_REVERSED = false;
    public static final double MAGNET_OFFSET = 0.1606445; // Magnet offset determined using Tuner X
    
    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;    
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
       
    // Motion Magic Configuration - Normal Mode
    public static final double CRUISE_VELOCITY = 10.0; // rotations per second
    public static final double ACCELERATION = 20.0; // rotations per second squared
    public static final double JERK = 200.0; // rotations per second cubed
    
    //ForwardLimitValue or Positions
    public static final double FORWARD_LIMIT  = 0.70;
    public static final double REVERSE_LIMIT = 0.00;
    public static final double POSE_TOLERANCE = 0.02;   
    
    // Manual Control
    public static final double MANUAL_INCREMENT = 0.02;    // Amount to move per button press
    
    // Tolerance for considering a position achieved
    public static final double POSITION_TOLERANCE = 0.02; // rotations
    public static final double POSITION_TOLERANCE_TEST = 0.05; // rotations, more forgiving in test mode

    // PID and FF Gains
    public static final class Gains0 {
        public static final double kP = 5.00;
        public static final double kI = 0.00;
        public static final double kD = 0.01;
        public static final double kG = 0.50;
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
    }

    // PID and FF Gains for test mode
    public static final class Gains1 {
        public static final double kP = 3.00;
        public static final double kI = 0.00;
        public static final double kD = 0.01;
        public static final double kG = 0.50;
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;

    }
    
    /* 
     * Named Motor positions
     * In theory these will not be used as the CANCoder will be used for position
     */
    public static final class MotorPositions {
        public static final double UP = 0.26;
        public static final double LOAD_CORAL = 0.0;
        public static final double GRAB_ALGAE = 0.50;
        public static final double SCOREL1 = 0.05;
        public static final double SCOREL2 = 0.10; // 0.20
        public static final double SCOREL3 = 0.30;
        public static final double SCOREL4 = 0.40;

    }

    // CANCoder positions
        // TODO May need to Add CANCODER_OFFSET?
        // Range is 0.0 to 1.0 in theory,
        // -0,32 to .70 in practice
        public static final double LOAD_CORAL = 0.00; 
        public static final double VERTICAL = 0.16; // Wrist is in a "C" position 
        public static final double HORIZONTAL = 0.26; // Wrist is in an "U" position
        public static final double GRAB_ALGAE = 0.45;
        public static final double L1 = 0.05; // Slightly forward
        public static final double L2 = 0.20; // More forward
        public static final double L3 = 0.30;
        public static final double L4 = 0.40;

    // var limitConfigs = new HardwareLimitSwitchConfigs();
    // limitConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    // limitConfigs.ForwardLimitRemoteSensorID = m_cancoder.getDeviceID();
    // m_motor.getConfigurator().apply(limitConfigs);

    // Motion constraints
    
    public enum WristState {
        UNSAFE,
        SAFE,
    }
    
}