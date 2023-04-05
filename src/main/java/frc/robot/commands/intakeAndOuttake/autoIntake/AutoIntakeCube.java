package frc.robot.commands.intakeAndOuttake.autoIntake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;

public class AutoIntakeCube extends SequentialCommandGroup {//TODO:FIX - The commands don't finish if the bot keeps driving
    public AutoIntakeCube(Arm arm, Intake intake, double wait) {
        addCommands(//OPen the intake earlier
            arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
            // intake.runOnce(()->intake.open(true)),
            intake.runIntakeFor(wait),
            new WaitCommand(0.2),
            arm.setPositionCommand(Position.HOLD)
        );
    }
}

