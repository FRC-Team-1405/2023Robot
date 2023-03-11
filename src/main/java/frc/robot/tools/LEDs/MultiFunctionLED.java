 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class MultiFunctionLED implements IAddressableLEDHelper{

    private AddressableLEDHelper[] ledHelpers;
    private static int mode = 0;
    private static int MAX_MODE = 0;
    public MultiFunctionLED(AddressableLEDHelper... ledHelpers) {
        this.ledHelpers = ledHelpers;
        MultiFunctionLED.MAX_MODE = ledHelpers.length;
    }
    
    public int getLedCount() {
        return ledHelpers[mode].getLedCount();
    }

    public static void setMode(int mode) {
        if (mode >= 0 && mode < MultiFunctionLED.MAX_MODE) {
            MultiFunctionLED.mode = mode;
        }
    }
    // Sets the offset to apply to buffer set calls
    public void initialize(int offset) {
        ledHelpers[mode].initialize(offset);
    }

    // Basically like a execute call
    public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer) {
        return ledHelpers[mode].writeData(buffer);
    }

}
