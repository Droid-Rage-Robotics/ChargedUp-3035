// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.TrackedElement.Element;
public class ToggleElement extends SequentialCommandGroup {
    public ToggleElement() {
        if(TrackedElement.get()==Element.CUBE) TrackedElement.set(Element.CONE);
            else TrackedElement.set(Element.CUBE);
    }
}
