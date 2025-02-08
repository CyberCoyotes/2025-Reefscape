package frc.robot.subsystems.EndEffector;

// EffectorState.java
public enum EffectorState {
    INTAKE_CHORAL(-10.0),
    SCORE_CHORAL(-5.0), 
    HOLD(0.01),
    STOP(0.0);       // Stop motors

    public final double current;
    
    private EffectorState(double current) {
        this.current = current;
    }
}