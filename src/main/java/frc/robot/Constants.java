package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {

    // CAN ID excluding the drivetrain
    // Drivertrain is 1 through 12
    // Pigeon = 15

    public static final int WRIST_MOTOR_ID = 20;
    public static final int WRIST_ENCODER_ID = 21;
    
    public static final int EFFECTOR_MOTOR_ID = 22; 

    public static final int ELEVATOR_FOLLOW_ID = 23;
    public static final int ELEVATOR_LEAD_ID = 24;

    public static final int CLIMBER_ID = 25;

    public static final int CANDLE_ID = 14;

    // public static final int CORAL_LASER_ID = 51;
    public static final int ELEVATOR_LASER_ID = 52;
    
    // Time of flight sensor in end effector, detecting coral
    public static final int CORAL_SENSOR_ID = 42;

    // Time of flight sensor by Playing with Fusion, determine robot distance to nearest object
    public static final int TOF_SENSOR_ID = 41;

    // Time of flight sensor by Playing with Fusion, determine reef branch distance
    public static final int REEF_SENSOR_ID = 102;
    
    public static final double YOU_SHALL_NOT_PASS =    250;

    // Consider setting the elevator to a height that can detect the end of the post.

    public static final double STRAFE_RIGHT = -1.0;
    public static final double STRAFE_LEFT = 1.0;

    // CANBus. Canivore is already integrated into the drivetrain
    public static final CANBus kCANBus = new CANBus("rio");

    /**
     * Fun fact: Traditional Java uses ALL_CAPS_SNAKE_CASE. WPILib and CTRE use kCamelCase convention
     */
}
