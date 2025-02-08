package frc.robot.subsystems.wrist;

public enum WristStates {
    // Define the states of the wrist
    LOADING(0.0), // "Up" position for loading, facing towards the back of the robot
    SCORE_L1(0.20),
    SCORE_L2(0.20), // Probably more like 0.25
    SCORE_L3(0.20),
    SCORE_L4(0.20);

    public final double position;

    WristStates(double position) {
        this.position = position;
    }

    public double getWristPosition() {
        return position;
    }

}
