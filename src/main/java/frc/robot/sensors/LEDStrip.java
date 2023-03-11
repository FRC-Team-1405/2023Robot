package frc.robot.sensors;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDStrip {
    SPI port; 
    int ledCount; 
    byte[] data; 

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public LEDStrip(int port, int ledCount) {
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start(); 
    }
    
    public AddressableLEDBuffer getLedBuffer() {
        return ledBuffer;
    }

    public void setLedBuffer(AddressableLEDBuffer buffer) {
        ledBuffer = buffer;
    }

    public void displayLEDBuffer() {
        led.setData(ledBuffer);
    }

    public void setColor(Color8Bit color, int index) { 
        if(index >= ledCount)
            return; 
        
        int dataIndex = index * 4 + 4;
        data[dataIndex++] = (byte) 0xEF; 
        data[dataIndex++] = (byte) color.blue; 
        data[dataIndex++] = (byte) color.green; 
        data[dataIndex++] = (byte) color.red;  
    }

   public Color setPercentBrightness(Color color, double brightness) {
     double blue = color.blue * brightness;
     double green = color.green * brightness;
     double red = color.red * brightness;
 
     return new Color(red, green, blue);
   }
}