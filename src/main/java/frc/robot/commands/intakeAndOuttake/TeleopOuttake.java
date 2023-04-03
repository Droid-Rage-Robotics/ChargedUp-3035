package frc.robot.commands.intakeAndOuttake;

import frc.robot.commands.SuppliedCommand;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCone;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TeleopOuttake extends SequentialCommandGroup {
    public TeleopOuttake(Arm arm, Intake intake) {
    	addCommands(
            SuppliedCommand.create(
                () -> switch(TrackedElement.get()) {
                    case CONE -> new DropTeleopCone(arm, intake);
                    case CUBE -> new DropTeleopCube(arm, intake);
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