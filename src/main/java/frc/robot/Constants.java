package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {

    /*********************************
     * CAN ID excluding the drivetrain
     * Drivertrain motors and encoders are 1 through 12
     * Pigeon = 15
     ********************************/

    public static final int CANDLE_ID = 14;         // "Torch"
    public static final int WRIST_ENCODER_ID = 21;
    public static final int WRIST_MOTOR_ID = 20;
    
    public static final int EFFECTOR_MOTOR_ID = 22;

    public static final int ELEVATOR_FOLLOW_ID = 23;
    public static final int ELEVATOR_LEAD_ID = 24;

    public static final int CLIMBER_ID = 25;

    public static final int TOF_ID = 26;            // Playing with Fusion

    public static final int CORAL_LASER_ID = 51;    // Grapple Robotics
    public static final int ELEVATOR_LASER_ID = 52; // Grapple Robotics


    // CANBus. Canivore is already integrated into the drivetrain
    public static final CANBus kCANBus = new CANBus("rio");

}
