// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot2;
import frc.robot.subsystems.EnumPositions.TrackedElement;
import frc.robot.subsystems.EnumPositions.TrackedElement.Element;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class Outtake extends SequentialCommandGroup {
    public Outtake(Elevator elevator, Pivot pivot, Intake intake) {
    	addCommands(Commands.either(
            new DropCone(elevator, pivot, intake), // on true
            new DropCube(elevator, pivot, intake), //on false
            TrackedElement::isCone
        ));
    }
}
