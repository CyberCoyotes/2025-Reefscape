package frc.robot.subsystems.endEffector;

// EffectorState.java
public enum EffectorState {
    INTAKE_CHORAL(-0.4),   // (-) Power to intake CHORAL
    SCORE_CHORAL(-0.4),    // (-) Power to score CHORAL, may consider changing power magnitude

    INTAKE_ALGAE(0.4), // Opposite power of INTAKE
    SCORE_ALGAE(0.4),   // 40% power for scoring
    
    HOLD(-0.05),    // Low power hold
    STOP(0.0);    // Fully stopped

    public final double dutyCycle;

    private EffectorState(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }
}
