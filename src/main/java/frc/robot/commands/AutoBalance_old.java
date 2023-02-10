// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public class AutoBalance_old extends SequentialCommandGroup {
  
  /** Creates a new AutoBalance. */
  public AutoBalance_old(SwerveDrive swerve, boolean forward) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand()); 
    BooleanSupplier atZero = () -> (Math.abs(swerve.getPitch()) <= 1.0); 
    addCommands(new DriveToPitch(swerve, forward),                     
                new WaitCommand(.5), 
                new Balance(swerve, 0.1), 
                new WaitCommand(.75), 
                new ConditionalCommand(new PrintCommand("Ballanced"), new Balance(swerve, 0.08), atZero), 
                new WaitCommand(.75),
                new ConditionalCommand(new PrintCommand("Ballanced"), new Balance(swerve, 0.06), atZero), 
                new WaitCommand(.75),
                new ConditionalCommand(new PrintCommand("Ballanced"), new Balance(swerve, 0.04), atZero)
                );
  }
}
