// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class SwerveType {
    public enum Type{
        Standard,
        Flipped
    }

    private static Type swerveType;
    static {
        DigitalInput input = new DigitalInput(9);    
        swerveType = input.get() ? Type.Standard : Type.Flipped;
        // input.close();
    }

    public static boolean isStandard(){
        return swerveType == Type.Standard;
    }
    public static boolean isFlipped(){
        return swerveType == Type.Flipped;
    }
}
