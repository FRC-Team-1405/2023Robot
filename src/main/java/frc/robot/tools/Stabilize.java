// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

/** Add your docs here. */
public class Stabilize {
    private int stopCount = 0;
    private int count = 0;
    private boolean value;

    public Stabilize(int stopCount){
        this.stopCount = stopCount;
        reset();
    }

    public void reset(){
        count = 0;
    }
    public boolean calculate(boolean value){
        if (this.value == value){
            count += 1;
        }
        else {
            count = 0;
        }
        
        if (count >= stopCount){
            this.value = value;
        }

        return value;
    }
}
