
# Subsystems

## Climb

- Clasper mechanism that rotates up (motor) and through the bars of the CAGE with a clasper mechanism (passive mechanical), then is winched back (motor pulley/cabling mcehcanism) towards robot
- (1 Kraken?) Motor for rotating climb mechanism up
- (1 Kraken) Motor with a gearbox `CLIMBER_GEARBOX_RATIO` (? gear ratio) to winch the climb-clasper mechanism back to the robot
- Climber States returns `START_POSITION` and `ATTACH_POSITION` and `RETURN_POSITION`.


## ElevatorSubsystem

- (2 Kraken) Motors in lead and follower modes attached to a `ELEVATOR_GEARBOX_RATIO` (9:1 gear ratio)
- (Playing with Fusion) CAN ToF sensor near the base of elevator to reset the Kraken encoder to zero when the distance is a predetermined value `ELEVATOR_RESET_DISTANCE`(2 cm.)
- Elevator set points for ELEVATOR `HOME`, `L1`, `L2`, `L3`, `L4`
- Allow for manual control of elevator
- Elevator States `ELEVATOR_UP` and `ELEVATOR_HOME`

## EndEffectorSubsystem "The Claw"

- C-shaped structure that handles coral coming in and out, and also handles ALGAE
- (1 Kraken) Motor powers wheels above and below, only reduction is from pulleys
- Power mode in an on/off and bi-directional `EFFECTOR_INTAKE` and `EFFECTOR_SCORE`
- (ThirftyBot) CAN ToF sensor as "coral Detector" to turn off the motor intake direction when "beam break" distance condition is met;
- End Effector States returns `NO_coral` and `coral_DETECTED`

## LEDSubsystem

- Indicate the status of _________________________
- CANdle led controller
- Two light bars (5v? or 12v) with led counts ()

## VisionSubsystem

- Limelight 3
- Primarily aligning to the reef offset positions left, center, right (uses tx parameter in Limelight)
- Ideally auto aligns left-right, distance away (ta or ty), and rotational
- `REEF DETECTED` `REEF_X` `REEF_DISTANCE` and `REEF_ROTATIONAL`

## WristSubsystem

- Mechanical could be considered part of the end effector
- (1 Kraken) Motor to rotate of the wrist of "the claw" and attached to a gearbox (West Coast Products)
- Rotational position for `INTAKE_POSITION` and Rotational Position for `SCORING_POSITION`
- Motor will use power mode bi-directional with rotation position being handled by a separate encoder
- (VEX housing with CTRE Encoder) Use a through-bore encoder to track rotational position
- Wrist States return `INTAKE_READY` and `SCORING_READY`
 