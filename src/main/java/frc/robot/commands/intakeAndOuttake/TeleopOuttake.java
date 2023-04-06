package frc.robot.commands.intakeAndOuttake;

import frc.robot.commands.SuppliedCommand;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCone;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.Arm;

public class TeleopOuttake extends SuppliedCommand {
    public TeleopOuttake(Arm arm, Intake intake) {
        super(
                () -> switch(TrackedElement.get()) {
                    case CONE -> new DropTeleopCone(arm, intake);
                    case CUBE -> new DropTeleopCube(arm, intake);
                }
            // Commands.runOnce(
            //     () -> switch(TrackedElement.get()) {
            //         case CONE -> new DropTeleopCone(arm, intake);
            //         case CUBE -> new DropTeleopCube(arm, intake);
            //     })
        );

        // switch(TrackedElement.get()){
        //     case CONE: addCommands(new DropTeleopCone(arm, intake));
        //         break;
        //     case CUBE:addCommands(new DropTeleopCube(arm, intake));
        //         break;
        // }

            // SuppliedCommand.create(
            //     () -> switch(TrackedElement.get()) {
            //         case CONE -> intake.runOnce(()->intake.close(true));
            //         case CUBE -> intake.runOnce(()->intake.open(true));
            //     }
            // )
        // );
    }
}