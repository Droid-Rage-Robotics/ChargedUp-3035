// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot2;
import frc.robot.subsystems.Position.Positions;
public class MoveElevator extends SequentialCommandGroup {//TODO:Test
    public MoveElevator(Elevator elevator, Pivot pivot, Positions position) {//TODO: Make sure all elevator Positions are in here

    switch(position){
        case AUTOMIDCONE:
            Commands.sequence(elevator.moveAutoMid(), new WaitCommand(1.4), pivot.moveMid());
            break;
        case HIGHCONE:
        case HIGHCUBE:
            Commands.sequence(elevator.moveHigh(), pivot.moveHigh());
            break;
        case HOLD:
            Commands.sequence(elevator.moveHold(), pivot.moveHold());
        break;
        case INTAKEHIGH1CONE:
        case INTAKEHIGH1CUBE:
            Commands.sequence(elevator.moveIntake1High(), pivot.moveIntake1High());
            break;
        case INTAKEHIGH2CONE:
        case INTAKEHIGH2CUBE:
            Commands.sequence(elevator.moveIntake2High(), pivot.moveIntake2High());
            break;
        case INTAKELOWCONE:
        case INTAKELOWCUBE:
            Commands.sequence(elevator.moveIntakeLow(), pivot.moveIntakeLow());
            break;
        case LOWCONE:
        case LOWCUBE:
            Commands.sequence(elevator.moveLow(), pivot.moveLow());
            break;
        case MIDCONE:
        case MIDCUBE:
            Commands.sequence(elevator.moveMid(), pivot.moveMid());
            break;
        case START:
            Commands.none();
            break;
        
        }
    }
}
