package frc.robot.subsystems.endEffector;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.Units;

public class EndEffectorConstants {
    public static final Current INTAKE_CURRENT = Units.Amps.of(-10);  // Fast intake
    public static final Current SCORE_CURRENT = Units.Amps.of(-10);   // Controlled scoring
    public static final Current HOLD_CURRENT = Units.Amps.of(2);     // Light holding force
    public static final Current STOP_CURRENT = Units.Amps.of(0);     // Fully stopped
}
