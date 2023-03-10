// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.LEDs;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class BalanceLED extends AddressableLEDHelper {
    private int numLEDs;
    private int offset;
    private DoubleSupplier pitch;
    private final static double PITCH_MAX =  17.0;
    private final static double PITCH_MIN = -17.0;

    // numLEDs has to be a multiple of 3
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

        int led = (int) map( pitchValue, PITCH_MIN, PITCH_MAX, (double)offset, (double)(offset+numLEDs)) ;

        buffer.setLED(led, Color.kGreen);

        return buffer;
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
    }
}