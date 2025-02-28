package frc.robot.experimental;

public interface WristIO {

    default void updateInputs(ArmIOInputs inputs) {}

    // default void setLeds(boolean on) {}

    // @AutoLog
    class ArmIOInputs { 

    }

}
