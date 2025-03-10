package frc.robot.subsystems.led;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/led/LEDConstants.java
public class LEDConstants {

        // public static final int CANDLE_ID = 30; // CAN ID of the LED controller
        public static final int LED_COUNT = 30; // TODO Turn off for now
        public static final double DEFAULT_BRIGHTNESS = 1.0; // TODO Turned off LEDs for now
========
public class LEDConfig {

        // public static final int CANDLE_ID = Constants.ID // CAN ID of the LED controller
        public static final int LED_COUNT = 30; // TODO Turn off for now
        public static final double DEFAULT_BRIGHTNESS = 0.0; // TODO Turned off LEDs for now
>>>>>>>> main:src/main/java/frc/robot/subsystems/led/LEDConfig.java.md

    // Configuration properties
    public int ledCount;
    public double brightness;
    public LEDStripType stripType;
    public boolean statusLedOffWhenActive;
    public VBatOutputMode vBatOutputMode;
    public boolean disableWhenLOS;
    
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/led/LEDConstants.java
    public static LEDConstants defaultConfig() {
        LEDConstants config = new LEDConstants();
========
    public static LEDConfig defaultConfig() {
        LEDConfig config = new LEDConfig();
>>>>>>>> main:src/main/java/frc/robot/subsystems/led/LEDConfig.java.md
        config.ledCount = LED_COUNT; // TODO Count the LEDs for a strip
        config.brightness = DEFAULT_BRIGHTNESS; // 0.0 to 1.0
        config.stripType = LEDStripType.GRB; // TODO Determine actual strip type
        config.statusLedOffWhenActive = true;
        config.vBatOutputMode = VBatOutputMode.Modulated; // TODO Determine actual mode
        config.disableWhenLOS = false;
        return config;
    }

    // Zone definition for LED segments
    public static class Zone {
        public final int startIndex;
        public final int length;
        public final String name;
        
        public Zone(int start, int length, String name) {
            this.startIndex = start;
            this.length = length;
            this.name = name;
        }
    }

    public List<Zone> zones = new ArrayList<>();
}

