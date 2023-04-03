package frc.robot.commands.arm.IntakeAndOuttake;

import frc.robot.commands.SuppliedCommand;
import frc.robot.commands.arm.IntakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.commands.arm.IntakeAndOuttake.autoDrop.DropAutoCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Outtake extends SequentialCommandGroup {
    public Outtake(Arm arm, Intake intake) {
    	addCommands(
            SuppliedCommand.create(() -> switch(TrackedElement.get()) {
                case CONE -> new DropAutoCone(arm, intake);
                case CUBE -> new DropAutoCube(arm, intake);
            })
        );
    }
}