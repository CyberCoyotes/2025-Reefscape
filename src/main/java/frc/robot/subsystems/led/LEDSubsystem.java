package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDConfig.Constants;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle;
    private final CANdleConfiguration config = new CANdleConfiguration();
    private Animation activeAnimation = null;

    private static final int LED_COUNT = 30; // Adjust for future LED strips

    public enum LEDColor {
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        CYAN(0, 255, 255),
        ORANGE(255, 165, 0),
        YELLOW(255, 255, 0),
        PURPLE(128, 0, 128),
        WHITE(255, 255, 255),
        PINK(255, 20, 147),

        OFF(0, 0, 0); // Off

        public final int r, g, b;

        LEDColor(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SolidColor
    }

    private AnimationTypes currentAnimation = AnimationTypes.SolidColor;

    public LEDSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID, "rio");
        configureCANdle();
    }

    private void configureCANdle() {
        config.statusLedOffWhenActive = true;
        config.stripType = CANdle.LEDStripType.GRB;
        config.brightnessScalar = 1.0;
        config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        candle.configAllSettings(config);
    }

    public void setSolidColor(int red, int green, int blue) {
        candle.clearAnimation(0);
        candle.setLEDs(red, green, blue);
        activeAnimation = null;
        currentAnimation = AnimationTypes.SolidColor;
    }

    public void setColor(LEDColor color) {
        setSolidColor(color.r, color.g, color.b);
    }

    // Set the LED strip to green
    public void setGreen() {
        setColor(LEDColor.GREEN);
    }

    public void setBrightness(double brightness) {
        brightness = Math.max(0.0, Math.min(1.0, brightness)); // Clamp 0-1
        config.brightnessScalar = brightness;
        candle.configAllSettings(config);
    }

    public void setAnimation(AnimationTypes animationType) {
        currentAnimation = animationType;

        switch (animationType) {
            case ColorFlow:
                activeAnimation = new ColorFlowAnimation(255, 0, 0, 0, 0.5, LED_COUNT,
                        ColorFlowAnimation.Direction.Forward);
                break;
            case Fire:
                activeAnimation = new FireAnimation(0.5, 0.7, LED_COUNT, 0.7, 0.5);
                break;
            case Larson:
                activeAnimation = new LarsonAnimation(0, 255, 0, 0, 1, LED_COUNT, LarsonAnimation.BounceMode.Front, 3);
                break;
            case Rainbow:
                activeAnimation = new RainbowAnimation(1.0, 0.1, LED_COUNT);
                break;
            case RgbFade:
                activeAnimation = new RgbFadeAnimation(0.7, 0.4, LED_COUNT);
                break;
            case SingleFade:
                activeAnimation = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LED_COUNT);
                break;
            case Strobe:
                activeAnimation = new StrobeAnimation(240, 10, 180, 0, 0.3, LED_COUNT);
                break;
            case Twinkle:
                activeAnimation = new TwinkleAnimation(30, 70, 60, 0, 0.4, LED_COUNT,
                        TwinkleAnimation.TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                activeAnimation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LED_COUNT,
                        TwinkleOffAnimation.TwinkleOffPercent.Percent100);
                break;
            case SolidColor:
            default:
                activeAnimation = null;
                break;
        }

        if (activeAnimation != null) {
            candle.animate(activeAnimation);
        } else {
            candle.clearAnimation(0);
        }
    }

    public void stopLEDs() {
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
        activeAnimation = null;
        currentAnimation = AnimationTypes.SolidColor;
    }

    public double getVoltage() {
        return candle.getBusVoltage();
    }

    public double getCurrent() {
        return candle.getCurrent();
    }

    public double getTemperature() {
        return candle.getTemperature();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LEDs/AnimationActive", activeAnimation != null);
        SmartDashboard.putString("LEDs/CurrentAnimation", currentAnimation.name());
        SmartDashboard.putNumber("LEDs/Brightness", config.brightnessScalar);
        SmartDashboard.putNumber("LEDs/Voltage", getVoltage());
        SmartDashboard.putNumber("LEDs/Current", getCurrent());
        SmartDashboard.putNumber("LEDs/Temperature", getTemperature());
    }
}
