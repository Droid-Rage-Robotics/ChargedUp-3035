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
import frc.robot.subsystems.Elevator.ElevatorPosition;
public class ToggleIntake extends SequentialCommandGroup {
    Map<Object, Command> commandMap = new HashMap<>();
    public ToggleIntake(Elevator elevator, Pivot pivot, Intake intake) {//TODO: Make sure all elevator Positions are in here

        commandMap.put(ElevatorPosition.AUTOMIDCONE, Commands.none());

        commandMap.put(ElevatorPosition.HIGHCONE, Commands.sequence(elevator.moveHigh(), pivot.moveHigh()));
        commandMap.put(ElevatorPosition.HIGHCUBE, Commands.sequence(elevator.moveHigh(), pivot.moveHigh()));

        commandMap.put(ElevatorPosition.HOLD, Commands.sequence(elevator.moveHold(), pivot.moveHold()));

        commandMap.put(ElevatorPosition.INTAKEHIGH1CONE, Commands.sequence(elevator.moveIntake1High(), pivot.moveIntake1High()));
        commandMap.put(ElevatorPosition.INTAKEHIGH1CUBE, Commands.sequence(elevator.moveIntake1High(), pivot.moveIntake1High()));

        commandMap.put(ElevatorPosition.INTAKEHIGH2CONE, Commands.sequence(elevator.moveIntake2High(), pivot.moveIntake2High()));
        commandMap.put(ElevatorPosition.INTAKEHIGH2CUBE, Commands.sequence(elevator.moveIntake2High(), pivot.moveIntake2High()));

        commandMap.put(ElevatorPosition.INTAKELOWCONE, Commands.sequence(elevator.moveIntakeLow(), pivot.moveIntakeLow()));
        commandMap.put(ElevatorPosition.INTAKELOWCUBE, Commands.sequence(elevator.moveIntakeLow(), pivot.moveIntakeLow()));

        commandMap.put(ElevatorPosition.LOWCONE, Commands.sequence(elevator.moveLow(), pivot.moveLow()));
        commandMap.put(ElevatorPosition.LOWCUBE, Commands.sequence(elevator.moveLow(), pivot.moveLow()));

        commandMap.put(ElevatorPosition.MIDCONE, Commands.sequence(elevator.moveMid(), pivot.moveMid()));
        commandMap.put(ElevatorPosition.MIDCUBE, Commands.sequence(elevator.moveMid(), pivot.moveMid()));

        commandMap.put(ElevatorPosition.START, Commands.none());

        addCommands(
            intake.toggleCommand(),
            // Commands.sequence(
            //     Commands.run(null, null)
            // )
            Commands.select(commandMap, elevator::getTargetPosition)
        );
    }
}
