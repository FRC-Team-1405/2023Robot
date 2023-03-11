// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.LEDs;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.tools.MathTools;

/** Add your docs here. */
public class BalanceLED extends AddressableLEDHelper {
    private int numLEDs;
    private int offset;
    private DoubleSupplier pitch;
    private final static double PITCH_MAX =  17.0;
    private final static double PITCH_MIN = -17.0;

    public BalanceLED(int numLEDs, DoubleSupplier pitch) {
        super(numLEDs);
        this.numLEDs = numLEDs;
        this.pitch = pitch;
    }

    @Override
    public void initialize(int offset) {
        this.offset = offset;
    }

    @Override
    public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer) {
        double pitchValue = pitch.getAsDouble();

        int led = (int) MathTools.map( pitchValue, PITCH_MIN, PITCH_MAX, (double)offset, (double)(offset+numLEDs-1)) ;
        for(int i = offset; i < offset + numLEDs; i++){
            buffer.setLED(i, Color.kBlack);
        }
        int bubble = numLEDs / 2 - 2;
        buffer.setLED(bubble, Color.kYellow);
        buffer.setLED(bubble+1, Color.kYellow);
        buffer.setLED(bubble+2, Color.kYellow);
        buffer.setLED(bubble+3, Color.kYellow);
        buffer.setLED(bubble+4, Color.kYellow);

        buffer.setLED(led, Color.kGreen);
        if (led > 0)
            buffer.setLED(led-1, Color.kGreen);
        if (led < numLEDs-1)
            buffer.setLED(led+1, Color.kGreen);

        return buffer;
    }

}