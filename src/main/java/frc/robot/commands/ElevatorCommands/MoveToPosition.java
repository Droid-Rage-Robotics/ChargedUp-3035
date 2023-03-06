// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Pivot;
public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(Elevator elevator, Pivot pivot) {
        ElevatorPosition position = Elevator.ElevatorPosition.get();
        switch(position) {
            case START:
            case INTAKELOW:
            case AUTOMIDCONE:
                return;
            case LOWCONE:
            case LOWCUBE:
                addCommands(
                    elevator.moveLow(),
                    pivot.moveLow()
                );
            case MIDCONE:
            case MIDCUBE:
                addCommands(
                        elevator.moveMid(),
                        pivot.moveMid()
                    );

            case HIGHCONE:
            case HIGHCUBE:
                addCommands(
                        elevator.moveHigh(),
                        pivot.moveHigh()
                    );

            case INTAKEHIGH:
                addCommands(
                        elevator.moveIntakeHigh(),
                        pivot.moveIntakeHigh()
                    );

            case HOLD:
                addCommands(
                        elevator.moveHold(),
                        pivot.moveHold()
                    );
        }
    }
}
