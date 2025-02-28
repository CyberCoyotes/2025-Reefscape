# Tuniing

## kS (Static Friction)
*Currently 0.05V, suggesting 2.5V*

This is the minimum voltage needed to make the elevator start moving
When you hear that "click", it's trying to move but can't overcome friction
Think of it like trying to push a heavy box - you need a certain minimum force just to get it moving
The 2.5V suggestion means "always apply at least 2.5V in the direction of motion"

## kG (Gravity Compensation) 
*Currently 0V, suggesting 4.5V*

This is probably the most important gain for an elevator
It's the constant voltage needed just to hold the elevator in place against gravity
Without this, your elevator is fighting gravity using only position error (kP)
The 4.5V means "always apply 4.5V upward to counter gravity"
Once tuned, your elevator should hold position with minimal kP correction

## kP (Proportional)
*Currently 0.20, suggesting 0.80*

This multiplies your position error to create correction
If you're 1 meter from your target, and kP is 0.8, it adds 0.8V per meter of error
Too low: sluggish response, won't reach target
Too high: oscillation/bouncing around target
Think of it like a spring - stronger spring (higher kP) means faster motion but more bounce

## kD (Derivative) - Currently 0V, suggesting 0.10

This is like adding damping or shock absorbers
It resists rapid changes in velocity
Helps prevent overshooting and oscillation
Works with kP to create smooth motion
Think of it like the shocks on a car - without it, you bounce around

## kV (Velocity Feedforward)

*Currently 0.12, keeping 0.12*

This predicts how much voltage you need for a given velocity
Higher values mean more voltage per unit of velocity
0.12 seems reasonable for an elevator - keeping it

## kI (Integral)

*Keeping at 0*

This accumulates position error over time
Usually not needed if kS and kG are tuned well
Can cause oscillation if used incorrectly
Leaving at 0 for now

## kA (Acceleration Feedforward)

*Keeping at 0*

This predicts voltage needed for acceleration
Less critical for elevators than other mechanisms
Can add later if needed for faster motion

The full system works like this:

kG provides constant upward force to counter gravity
kS overcomes initial friction in whatever direction you're moving
kP and kD work together to smoothly reach the target position
kV helps maintain smooth motion during movement