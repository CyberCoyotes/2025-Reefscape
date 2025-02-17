package frc.robot.subsystems.endEffector;

// EffectorState.java
public enum EffectorState {
    INTAKE_coral(-0.4),   // (-) Power to intake coral
    SCORE_coral(0.4),    // TODO switch for a temp intake (-) Power to score coral, may consider changing power magnitude

    INTAKE_CORAL(-0.4),   // (-) Power to intake coral
    SCORE_CORAL(-0.4),    // (-) Power to score coral, may consider changing power magnitude

    INTAKE_ALGAE(0.4), // Opposite power of INTAKE
    SCORE_ALGAE(-1.0),   // 40% power for scoring
    
    HOLD(-0.05),    // Low power hold
    STOP(0.0);    // Fully stopped

    public final double dutyCycle;

    private EffectorState(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }
}