package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

public class CANdleSubsystem extends SubsystemBase {

    CANdle leds;
    CANdleConfiguration defaultConfig;

    public enum Color {
        red,
        yellow,
        green,
        off
    }

    public CANdleSubsystem() {
        leds = new CANdle(61);
        defaultConfig = new CANdleConfiguration();
        defaultConfig.stripType = LEDStripType.RGB;
        defaultConfig.brightnessScalar = 0.25;
        leds.configAllSettings(defaultConfig);
    }

    public void solid(Color color) {
        switch (color) {
            case red:
                leds.setLEDs(255, 0, 0);
                break;
            case yellow:
                leds.setLEDs(255, 255, 0);
                break;
            case green:
                leds.setLEDs(0, 255, 0);
                break;
            case off:
                leds.setLEDs(0, 0, 0);
                break;

            default:
                break;
        }
    }

    public void strobe(Color color) {
        switch (color) {
            case red:
            leds.clearAnimation(0);
            leds.animate(new StrobeAnimation(255, 0, 0, 0 , 1, 8));
                break;
            case yellow:
            leds.clearAnimation(0);
                leds.animate(new StrobeAnimation(255, 255, 0, 0, 1, 8));
                break;
            case green:
            leds.clearAnimation(0);
                leds.animate(new StrobeAnimation(0, 255, 0, 0, 1, 8));
                break;

            default:
                break;
        }
    }
    public void clear(){
        leds.clearAnimation(0);
    }
}
