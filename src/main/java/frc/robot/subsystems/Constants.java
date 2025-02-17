package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;

public class Constants {

    // CAN ID excluding the drivetrain
    public static final int WRIST_MOTOR_ID = 20;
    public static final int ELEVATOR_MOTOR_ID = 19;
    public static final int EFFECTOR_MOTOR_ID = 21;

    // CANBus
    public static final CANBus kCANBus = new CANBus("rio");
    // public static final CANBus kCANivore = new CANBus("canivore");


    
}
