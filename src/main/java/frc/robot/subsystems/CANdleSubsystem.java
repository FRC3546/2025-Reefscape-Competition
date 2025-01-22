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
    StrobeAnimation strobe = new StrobeAnimation(0, 0, 0);

    public enum Color {
        red,
        yellow,
        green,
        off
    }

    public CANdleSubsystem() {
        leds = new CANdle(0);
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
                leds.animate(null);
                strobe.setB(0);
                strobe.setR(255);
                strobe.setG(0);
                leds.animate(strobe);
                break;
            case yellow:
                leds.animate(null);
                strobe.setB(0);
                strobe.setR(255);
                strobe.setG(255);
                leds.animate(strobe);
                break;
            case green:
                leds.animate(null);
                strobe.setB(0);
                strobe.setR(0);
                strobe.setG(255);
                leds.animate(strobe);
                break;

            default:
                break;
        }
    }
}
