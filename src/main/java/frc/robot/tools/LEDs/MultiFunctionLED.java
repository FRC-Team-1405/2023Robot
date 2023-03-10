 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class MultiFunctionLED implements IAddressableLEDHelper{

    private AddressableLEDHelper[] ledHelpers;
    private int mode = 0;
    public MultiFunctionLED(AddressableLEDHelper... ledHelpers) {
        this.ledHelpers = ledHelpers;
    }
    
    public int getLedCount() {
        return ledHelpers[mode].getLedCount();
    }

    public void setMode(int mode) {
        this.mode = mode;
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
