package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;

    private int iter = 0;

    private boolean gamerMode = false;
    private double gamerFrequency = 1;
    private final int length = 284; // 142
    public LEDs() {
        leds = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(length);
        leds.setLength(length);
        leds.start();
    }

    public void setFullStripColor(Color color) {
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setFullStripColorRGB(int r, int g, int b) {
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setFullStripColorHSV(int h, int s, int v) {
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, h, s, v);
        }
    }

    public void setGamerMode(boolean enabled) {
        gamerMode = enabled;
    }

    public void update() {
        if(!gamerMode) return;

        double time = (System.currentTimeMillis() / 1000d) % 1;

        for(int i = 0; i < length / 4; i++) {
            int hue = (int)((time + i / (double)(length / 4)) * gamerFrequency * 180) % 180;
            ledBuffer.setHSV(i, hue, 255, 50);
            ledBuffer.setHSV(length / 2 - 1 - i, hue, 255, 50);
            ledBuffer.setHSV(length / 2 + i, hue, 255, 50);
            ledBuffer.setHSV(length - 1 - i, hue, 255, 50);
        }
    }

    @Override
    public void periodic() {
        if(iter == 2) {
            iter = 0;
            update();
            leds.setData(ledBuffer);
        }
        iter++;
    }
}