package frc.robot.commands.intakeAndOuttake.intakeAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;

public class AutoIntakeCube extends SequentialCommandGroup {//TODO:FIX - The commands don't finish if the bot keeps driving
    //TODO: Maybe make a position where the pivot intakes downwards so cube doesn't roll away
    public AutoIntakeCube(Arm arm, Intake intake, double wait) {
        addCommands(
            arm.setPositionCommand(Position.INTAKE_LOW),
            intake.runOnce(()->intake.open(true))
        );
    }
}

