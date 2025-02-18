package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {

    // CAN ID excluding the drivetrain
    // Drivertrain is 1 through 12
    // Pigeon = 13
    // CANivore = 15

    public static final int WRIST_ENCODER_ID = 14; // 21
    public static final int WRIST_MOTOR_ID = 20;
    
    public static final int EFFECTOR_MOTOR_ID = 21; // 22

    public static final int ELEVATOR_FOLLOW_ID = 23;
    public static final int ELEVATOR_LEAD_ID = 24;

    public static final int CLIMBER_ID = 25;

    public static final int CORAL_LASER_ID = 51;
    public static final int ELEVATOR_LASER_ID = 52;

    // CANBus. Canivore is already integrated into the drivetrain
    public static final CANBus kCANBus = new CANBus("rio");

}
