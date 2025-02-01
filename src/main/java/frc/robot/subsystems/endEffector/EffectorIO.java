package frc.robot.subsystems.endEffector;

public interface EffectorIO {

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setLeds(boolean on) {}

    // @AutoLog
    class IntakeIOInputs { 

    }

}
