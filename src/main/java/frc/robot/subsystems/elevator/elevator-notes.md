
# Robot Container for v2
```
public class RobotContainer {
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Set default command to control elevator with left stick Y axis
        m_elevator.setDefaultCommand(
            new MoveElevator(
                m_elevator, 
                () -> -m_operatorController.getLeftY()
            )
        );

        // Add button bindings for preset positions
        m_operatorController.a().onTrue(Commands.runOnce(() -> 
            m_elevator.setPosition(ElevatorSubsystem.BOTTOM_POSITION)));
        m_operatorController.y().onTrue(Commands.runOnce(() -> 
            m_elevator.setPosition(ElevatorSubsystem.TOP_POSITION)));
    }
}
```

Reference [Claude Chat](https://claude.ai/chat/b4e23ed0-c67c-4e1b-b2dc-a6ba6b4c0a51)
