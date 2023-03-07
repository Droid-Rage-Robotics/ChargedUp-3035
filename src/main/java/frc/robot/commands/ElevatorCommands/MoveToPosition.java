// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Pivot;
public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(Elevator elevator, Pivot pivot, Intake intake) {
        addCommands(
            intake.toggleCommand(),
            switch(elevator.getTargetPosition()) {
                case AUTOMIDCONE->Commands.none();
                case HIGHCONE, HIGHCUBE->Commands.sequence(elevator.moveLow(), pivot.moveLow());
                case HOLD->Commands.none();
                case INTAKEHIGH->Commands.none(); //TODO: we need two positons for this probably
                case INTAKELOW->Commands.none();
                case LOWCONE, LOWCUBE->Commands.sequence(elevator.moveLow(), pivot.moveLow());
                case MIDCONE, MIDCUBE->Commands.sequence(elevator.moveMid(), pivot.moveMid());
                case START->Commands.none();
                
            }
        );
    }
}
