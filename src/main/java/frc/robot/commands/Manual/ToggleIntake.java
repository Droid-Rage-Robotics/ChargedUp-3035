// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manual;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot2;
import frc.robot.subsystems.EnumPositions.Position.Positions;
public class ToggleIntake extends SequentialCommandGroup {
    Map<Object, Command> commandMap = new HashMap<>();
    public ToggleIntake(Elevator elevator, Pivot pivot, Intake intake) {//TODO: Make sure all elevator Positions are in here

        commandMap.put(Positions.AUTOMIDCONE, Commands.none());

        commandMap.put(Positions.HIGHCONE, Commands.sequence(elevator.moveHigh(), pivot.moveHigh()));
        commandMap.put(Positions.HIGHCUBE, Commands.sequence(elevator.moveHigh(), pivot.moveHigh()));

        commandMap.put(Positions.HOLD, Commands.sequence(elevator.moveHold(), pivot.moveHold()));

        commandMap.put(Positions.INTAKEHIGH1CONE, Commands.sequence(elevator.moveIntake1High(), pivot.moveIntake1High()));
        commandMap.put(Positions.INTAKEHIGH1CUBE, Commands.sequence(elevator.moveIntake1High(), pivot.moveIntake1High()));

        commandMap.put(Positions.INTAKEHIGH2CONE, Commands.sequence(elevator.moveIntake2High(), pivot.moveIntake2High()));
        commandMap.put(Positions.INTAKEHIGH2CUBE, Commands.sequence(elevator.moveIntake2High(), pivot.moveIntake2High()));

        commandMap.put(Positions.INTAKELOWCONE, Commands.sequence(elevator.moveIntakeLow(), pivot.moveIntakeLow()));
        commandMap.put(Positions.INTAKELOWCUBE, Commands.sequence(elevator.moveIntakeLow(), pivot.moveIntakeLow()));

        commandMap.put(Positions.LOWCONE, Commands.sequence(elevator.moveLow(), pivot.moveLow()));
        commandMap.put(Positions.LOWCUBE, Commands.sequence(elevator.moveLow(), pivot.moveLow()));

        commandMap.put(Positions.MIDCONE, Commands.sequence(elevator.moveMid(), pivot.moveMid()));
        commandMap.put(Positions.MIDCUBE, Commands.sequence(elevator.moveMid(), pivot.moveMid()));

        commandMap.put(Positions.START, Commands.none());

        addCommands(
            intake.toggleCommand(),
            // Commands.sequence(
            //     Commands.run(null, null)
            // )
            Commands.select(commandMap, elevator::getTargetPosition)
        );
    }
}
