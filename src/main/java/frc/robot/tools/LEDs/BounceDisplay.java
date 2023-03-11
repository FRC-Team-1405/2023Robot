// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.LEDs;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.tools.MathTools;

/** Add your docs here. */
public class BounceDisplay extends AddressableLEDHelper {
    private int numLEDs;
    private int offset;
    private int pos = 0;
    private int delay = 0;
    private int direction = 1;

    public BounceDisplay(int numLEDs) {
        super(numLEDs);
        this.numLEDs = numLEDs;
    }

    @Override
    public void initialize(int offset) {
        this.offset = offset;
    }

    @Override
    public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer) {
        if (delay++ % 5 != 0)
            return buffer;

        Color backGround = super.setPercentBrightness(Color.kGreen, 0.2);
        for(int i = offset; i < offset + numLEDs; i++){
            buffer.setLED(i, backGround);
        }

        Color ball = super.setPercentBrightness(Color.kYellow, 0.2);
        buffer.setLED(pos, ball);

        pos += direction;
        if (pos == (numLEDs-1))
            direction = -1;
        else if (pos == 0)
            direction = 1;
        return buffer;
        }
}