package frc.robot.commands.ElevatorCommands;

import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TrackedElement;
import frc.robot.subsystems.Intake.Intake;
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
