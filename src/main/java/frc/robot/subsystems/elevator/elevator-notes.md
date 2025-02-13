
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

Reference from 2/8 [Claude Chat 2/8](https://claude.ai/chat/0ce7068c-23be-4766-8674-02b8326b7997)

DutyCycleOut 0.1 to maintain a slow, steady climb   |   position of 26-30

Reference [Claude Chat 2/8 5PM](https://claude.ai/chat/dcffea4e-d55c-42a0-92e6-1e2061f5df57)


https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/control-requests-guide.html

https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/Follower.html