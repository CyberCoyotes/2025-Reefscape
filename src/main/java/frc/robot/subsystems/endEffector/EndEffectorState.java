package frc.robot.subsystems.endEffector;

// EffectorState.java
public enum EndEffectorState {
    INTAKE_CHORAL(-10.0),
    SCORE_CHORAL(-5.0), 
    HOLD(-0.5),
    STOP(0.0);       // Stop motors

    public final double current;
    
    private EndEffectorState(double current) {
        this.current = current;
    }
}