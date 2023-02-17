// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class DigitalToggle implements BooleanSupplier{
    private DigitalInput toggle;
    private boolean value;
    public DigitalToggle(int slot) {
        toggle = new DigitalInput(slot);
        value = toggle.get();
    }
    @Override
    public boolean getAsBoolean() {
        boolean value = toggle.get();
        if(this.value == value){
            return false;
        }
        this.value = value;
        return true;
    }
}
