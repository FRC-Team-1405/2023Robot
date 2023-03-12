// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.cert.Extension;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;

/** Add your docs here. */
public class ScoreCommand extends SequentialCommandGroup{


    public CommandBase setHighPostition = new InstantCommand( () -> {position = Arm.Position.High;} );
    public CommandBase setMiddlePosition = new InstantCommand( () -> {position = Arm.Position.Middle;});
    public CommandBase setLowPostition = new InstantCommand( () -> {position = Arm.Position.Low;} );
    public CommandBase setCustomPosition = new InstantCommand( () -> {position = Arm.Position.Custom;});



    private Arm arm;
    protected Arm.Position position = Position.Home;

    public ScoreCommand(Arm arm){
        this.arm = arm;
        addRequirements(arm);

        addCommands( new InstantCommand(() -> { arm.closedClaw(); }),
                     new ArmAngle(this.arm, () -> { return this.position;} ),
                     new ArmExtension(this.arm, () -> { return this.position;} ) );    
    }
    
    public ScoreCommand(Arm arm, Arm.Position position){ 
        this.arm = arm;
        addRequirements(arm);

        this.position = position;

        addCommands( new InstantCommand(() -> { arm.closedClaw(); }),
                     new ArmAngle(this.arm, () -> { return this.position;} ),
                     new ArmExtension(this.arm, () -> { return this.position;} ) );    
    }

    private static class ArmAngle extends CommandBase{
        private Arm arm;
        private Supplier<Position> position;
        public ArmAngle(Arm arm, Supplier<Position> position){
            this.arm = arm;
            this.position = position;
        }
    
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            arm.setElbowPosition(position.get());
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
        return arm.atElbowPosition();
        }
    }
    private static class ArmExtension extends CommandBase{
        private Arm arm;
        private Supplier<Position> position;
        public ArmExtension(Arm arm, Supplier<Position> position){
            this.arm = arm;
            this.position = position;
        }
            
        @Override
        public void initialize() {

            arm.setExtensionPosition(position.get());
        }
    
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
        return arm.atExtensionPosition();
        }
    }

}