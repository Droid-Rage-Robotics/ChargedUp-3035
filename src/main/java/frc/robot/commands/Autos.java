package frc.robot.commands;

import frc.robot.commands.Drive.PathPlannerFollow;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static CommandBase top(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return PathPlannerFollow.create(drive, "Top")
            .setMaxVelocity(0.3)
            .setAcceleration(0.4)
            .addMarker("preloadDrop", new SequentialCommandGroup(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(1),
                new DropCone(elevator, pivot, intake)
                ))
            .addMarker("pickUp", new SequentialCommandGroup(
                new MoveIntakeLow(elevator, pivot),
                new IntakeCube(pivot, intake)
                ))
            .addMarker("dropOne", new SequentialCommandGroup(
                new MoveHigh(elevator,pivot),
                new WaitCommand(1),
                new DropCone(elevator, pivot, intake)
                ))
                // drive.resetHeading(270)
            .build();
    }

    public static CommandBase mid(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(3),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(3)
                ),
            PathPlannerFollow.create(drive, "Middle")
                .setMaxVelocity(0.9)
                .setAcceleration(0.96)
                .build()

            // Commands.sequence(new MoveIntakeLow(elevator, pivot),
            //     new IntakeCone(pivot, intake))
        );
    }

    public static CommandBase bottom(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return PathPlannerFollow.create(drive, "Bottom")
            .setMaxVelocity(0.)
            .setAcceleration(0.4)
            .addMarker("preloadDrop", new SequentialCommandGroup(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(1),
                new DropCone(elevator, pivot, intake)
                ))
            .addMarker("pickUp", new SequentialCommandGroup(
                new MoveIntakeLow(elevator, pivot),
                new IntakeCube(pivot, intake)
                ))
            .build();
    }
    public static CommandBase strafeRight(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(3),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(3)
                ),
            PathPlannerFollow.create(drive, "StrafeBlue")
                .setMaxVelocity(0.7)
                .setAcceleration(0.9)
                .build()
        );
    }
    public static CommandBase charge(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(3),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(3)
                ),
            PathPlannerFollow.create(drive, "Charge")
                .setMaxVelocity(0.8)
                .setAcceleration(0.95)
                .build()
        );
    }
    public static CommandBase intake(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(3),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(3)
                ),
            PathPlannerFollow.create(drive, "Intake")
                .setMaxVelocity(0.65)
                .setAcceleration(0.85)
                .addMarker("intake", new SequentialCommandGroup(
                    new MoveIntakeLow(elevator, pivot),
                    new IntakeCube(pivot, intake)
                    ))
                .build()
        );
    }

    private Autos() {}
}
