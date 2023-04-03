package frc.robot.commands.arm.IntakeAndOuttake;

import frc.robot.commands.SuppliedCommand;
import frc.robot.commands.arm.IntakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.commands.arm.IntakeAndOuttake.autoDrop.DropAutoCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoOuttake extends SequentialCommandGroup {
    public AutoOuttake(Arm arm, Intake intake) {
    	addCommands(
            SuppliedCommand.create(
                () -> switch(TrackedElement.get()) {
                    case CONE -> new DropAutoCone(arm, intake);
                    case CUBE -> new DropAutoCube(arm, intake);
                }
            ),
            SuppliedCommand.create(
                () -> switch(TrackedElement.get()) {
                    case CONE -> intake.runOnce(()->intake.close(true));
                    case CUBE -> intake.runOnce(()->intake.close(true));
                }
            )
        );
    }
}