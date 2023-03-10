// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public interface IAddressableLEDHelper {
    // Sets the offset to apply to buffer set calls
    public void initialize(int offset);

    // Basically like a execute call
    public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer);

    public int getLedCount();
}
