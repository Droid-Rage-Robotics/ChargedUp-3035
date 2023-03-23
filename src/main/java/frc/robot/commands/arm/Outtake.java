package frc.robot.commands.arm;

import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Outtake extends SequentialCommandGroup {
    public Outtake(Arm arm, Intake intake) {
    	addCommands(
            new SuppliedCommand(() -> switch(TrackedElement.get()) {
                case CONE -> new DropCone(arm, intake);
                case CUBE -> new DropCube(arm, intake);
            })
        );
    }
}
