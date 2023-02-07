// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;
import java.util.function.IntSupplier;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestCommands {
  
  public static class TestCommand extends CommandBase {
    protected IntSupplier countDown;
    protected int count;

    /** Creates a new TestCommand. */
    public TestCommand(IntSupplier countDown) {
      this.countDown = countDown;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      count = countDown.getAsInt();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      count -= 1;
      System.out.printf("TestCommand: execute: %d\n", count);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return count == 0;
    }
  }

  public static class TestTrigger extends CommandBase {
    private Random rand = new Random();
    private boolean done;

    public void initialize() {
      System.out.println("\nStarting trigger.\n\n");
      done = false;
    }
    @Override
    public boolean isFinished() {
      done = rand.nextInt(100) == 0;
      return done;
    }
  }

  public static class TestGroup extends SequentialCommandGroup {
    public TestGroup(IntSupplier countDown) {
      IntSupplier countSupplier = new IntSupplier() {
        private int count = 12;
        public int getAsInt(){
          if (count <= 4) {
            return count;
          }
          count -= 2;
          return count;
        }
      };
  
      addCommands(new TestCommand(countSupplier),
                  new PrintCommand("\nInsert additional commands."),
                  new TestTrigger()
                  );
    }
    }
}
