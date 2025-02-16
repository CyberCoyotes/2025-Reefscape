package frc.robot.subsystems.wrist;

public enum WristMotorStates {
    // Define the states of the wrist
    LOADING(0.0), // "Up" position for loading, facing towards the back of the robot
    SCORE_L1(0.05),
    SCORE_L2(0.10), // Probably more like 0.25
    SCORE_L3(0.15),
    SCORE_L4(0.20); // Current Safe max position elevator down

    public final double position;

    WristMotorStates(double position) {
        this.position = position;
    }

    public double getWristPosition() {
        return position;
    }

}
