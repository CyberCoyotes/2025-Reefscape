package frc.robot.subsystems.endEffector;

// EffectorState.java
public enum EffectorOutputs {
    INTAKE_CORAL(0.4),
    SCORE_CORAL(0.6),
    SCORE_CORAL_INVERTED(-0.2),    // (-) Power to score coral, may consider changing power magnitude

    INTAKE_ALGAE(-0.4), // Opposite power of INTAKE
    SCORE_ALGAE(0.2),   // 40% power for scoring
    HOLD_ALGAE(0.05),    // Low power hold

    // WAITING, RUNNING, LOADING, INDEXING, LOADED

    STOP(0.0);    // Fully stopped

    public final double dutyCycle;

    private EffectorOutputs(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }

    

    /* Settings before inverting the motor in the ElevatorConstants.

    INTAKE_CORAL(-0.4),   // (-) Power to intake coral
    SCORE_CORAL(-0.2),
    SCORE_CORAL_INVERTED(0.2),    // (-) Power to score coral, may consider changing power magnitude

    INTAKE_ALGAE(0.4), // Opposite power of INTAKE
    SCORE_ALGAE(-0.2),   // 40% power for scoring
    HOLD_ALGAE(-0.05),    // Low power hold
    STOP(0.0);    // Fully stopped

    public final double dutyCycle;

    private EffectorState(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }
    
    * 
    */
}