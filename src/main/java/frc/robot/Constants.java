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
    public static final int MASER_ID = 54;


    // Time of flight sensor in end effector, detecting coral
    public static final int CORAL_SENSOR_ID = 42;

    // CANBus. Canivore is already integrated into the drivetrain
    public static final CANBus kCANBus = new CANBus("rio");

    // Time of flight sensor by Limelight, determine robot distance
    public static final int TOF_SENSOR_ID = 41;

    public static final double YSNP =    400; // FIXME Test this threshold for post or slope
    public static final double YOU_SHALL_NOT_PASS =    250; // FIXME Test this threshold for post or slope
    // Consider setting the elevator to a height that can detect the end of the post.

    public static final double STRAFE_SPEED_RT = -1.0; // TODO Verify
    public static final double STRAFE_SPEED_LT = 1.0; // TODO Verify


    /**
     * Fun fact: Traditional Java uses ALL_CAPS_SNAKE_CASE. WPILib and CTRE use kCamelCase convention
     */

}
