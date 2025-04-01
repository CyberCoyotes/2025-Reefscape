package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {

    // CAN ID excluding the drivetrain generated by the CTRE Phoenix framework
    // Drivertrain is 1 through 12; Pigeon = 15

    public static final int WRIST_MOTOR_ID = 20;
    public static final int WRIST_ENCODER_ID = 21;
    
    public static final int EFFECTOR_MOTOR_ID = 22;
    public static final int EFFECTOR_MOTOR_TWO_ID = 26; 

    public static final int ELEVATOR_FOLLOW_ID = 23;
    public static final int ELEVATOR_LEAD_ID = 24;

    public static final int CLIMBER_ID = 25;

    public static final int CANDLE_ID = 14;

    public static final int ELEVATOR_LASER_ID = 52;
    
    // Time of flight sensor in end effector, detecting coral
    public static final int CORAL_SENSOR_ID = 42;

    // Time of flight sensor by Playing with Fusion, mounted on right side
    public static final int TOF_SENSOR_ID = 41;

    // Time of flight sensor by Playing with Fusion, determine reef branch distance
    public static final int REEF_SENSOR_ID = 102;
    
    public static final double YOU_SHALL_NOT_PASS =    250; // distance in mm to reef

    // Consider setting the elevator to a height that can detect the end of the post.

    public static final double STRAFE_RIGHT = -1.0;
    public static final double STRAFE_LEFT = 1.0;

    /* Distance value from the reef and speed
     * Distance measured in millimeters (mm)
     * Approximately 330 mm from the elevator post to front of the reef.
    */
    public static final double DISTANCE_TO_OUTER_BUMPER = 330; // distance from the ToF mount to the outside edge of bumpers
    
    // Should stop about 7 cm / 70 mm before the reef to avoid collision
    public static final double DISTANCE_FROM_BUMPER = 0; // distance from the bumper in (mm)

    public static final double AUTO_TARGET_DISTANCE = DISTANCE_FROM_BUMPER + DISTANCE_TO_OUTER_BUMPER; // target distance to reef in mm


    public static final double AUTO_DRIVE_SPEED = 1.0; // 1.0 m/s (meters per second) speed for approach

    // CANBus. Canivore is already integrated into the drivetrain
    public static final CANBus kCANBus = new CANBus("rio");

    /**
     * Fun fact: Traditional Java uses ALL_CAPS_SNAKE_CASE. WPILib and CTRE use kCamelCase convention
     */
}
