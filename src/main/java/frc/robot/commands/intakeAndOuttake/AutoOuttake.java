package frc.robot.commands.intakeAndOuttake;

import frc.robot.commands.SuppliedCommand;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoOuttake extends SequentialCommandGroup {
    public AutoOuttake(Arm arm, Intake intake) {
    	addCommands(
            intake.runOnce(()->intake.close(false)),
            SuppliedCommand.create(
                () -> switch(TrackedElement.get()) {
                    case CONE -> new DropAutoCone(arm, intake);
                    case CUBE -> new DropAutoCube(arm, intake);
                }
            )
            // SuppliedCommand.create(
            //     () -> switch(TrackedElement.get()) {
            //         case CONE -> intake.runOnce(()->intake.close(true));
            //         case CUBE -> intake.runOnce(()->intake.open(true));
            //     }
            // )
        );
    }
}