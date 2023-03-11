// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.tools.LEDs.AddressableLEDHelper;
import frc.robot.tools.LEDs.IAddressableLEDHelper;
import frc.robot.sensors.LEDStrip;

public class LEDManager extends CommandBase {
  private IAddressableLEDHelper[] ledCommands;
  private LEDStrip ledStrip;

  public LEDManager(int port, IAddressableLEDHelper[] ledCommands) {
    this.ledCommands = ledCommands;

    // Set the offset for writing to the AddressableLEDBuffer
    int ledCount = 0;
    for (IAddressableLEDHelper ledCommand : ledCommands) {
      //Give the ledcount/ledOffset to the ledCommands
      ledCommand.initialize(ledCount);
      ledCount += ledCommand.getLedCount();
    }

    ledStrip = new LEDStrip(port, ledCount);
  }

  @Override
  public void execute() {
    for (IAddressableLEDHelper ledCommand : ledCommands) {
      ledStrip.setLedBuffer(ledCommand.writeData(ledStrip.getLedBuffer()));
    }
    ledStrip.displayLEDBuffer();
  }

  @Override 
  public boolean runsWhenDisabled() {
    return true;
  }
}