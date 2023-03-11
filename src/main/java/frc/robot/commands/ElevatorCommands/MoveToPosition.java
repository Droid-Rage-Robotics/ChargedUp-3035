// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot2;
public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(Elevator elevator, Pivot pivot, Intake intake) {
        addCommands(
            intake.toggleCommand(),
            switch(elevator.getTargetPosition()) {
                case AUTOMIDCONE -> Commands.none();
                case HIGHCONE, HIGHCUBE-> Commands.sequence(elevator.moveLow(), pivot.moveLow());
                case HOLD -> Commands.sequence(elevator.moveHold(), pivot.moveHold());
                case INTAKEHIGH1CONE, INTAKEHIGH1CUBE ->Commands.sequence(elevator.moveIntake1High(), pivot.moveIntake1High()); //TODO: we need two positons for this probably
                case INTAKEHIGH2CONE, INTAKEHIGH2CUBE ->Commands.sequence(elevator.moveIntake2High(), pivot.moveIntake2High());
                case INTAKELOWCONE, INTAKELOWCUBE ->Commands.sequence(elevator.moveIntakeLow(), pivot.moveIntakeLow());
                case LOWCONE, LOWCUBE ->Commands.sequence(elevator.moveLow(), pivot.moveLow());
                case MIDCONE, MIDCUBE ->Commands.sequence(elevator.moveMid(), pivot.moveMid());
                case START->Commands.none();
                
            }
        );
    }
}
