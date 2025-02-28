package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LEDSubsystem extends SubsystemBase {

    private CANdle candle;
    private CANdleConfiguration candleConfig;
    private int startIndex;
    private int segmentSize;

    public enum Color {
        Green(0, 255, 0),
        White(255, 255, 255),
        Red(255,0,0),
        Gold(255,215,0),
        Blue(0,0,255);

        private final int r;
        private final int g;
        private final int b;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public int getR() {
            return r;
        }

        public int getG() {
            return g;
        }

        public int getB() {
            return b;
        }
    }

    public LEDSubsystem() {
        candle = new CANdle(6);
        candleConfig = new CANdleConfiguration();
        candleConfig.stripType = LEDStripType.RGB;
        candleConfig.brightnessScalar = 1;
        startIndex = 0;
        segmentSize = 8;
        candle.configAllSettings(candleConfig);
    }

    public void off() {
        candle.setLEDs(0, 0, 0);
    }

    public void setColor(Color color) {
        candle.setLEDs(color.getR(), color.getG(), color.getB());
    }
}