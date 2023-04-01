package frc.robot.subsystem.arm.elevator;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.SafeCanSparkMax;
import frc.robot.utility.ShuffleboardValue;
import frc.robot.utility.SafeMotor.IdleMode;

public class VerticalElevatorSetPower extends SubsystemBase{
    private final ShuffleboardValue<Boolean> isEnabled = ShuffleboardValue.create(true, "Is Enabled", VerticalElevator.class.getSimpleName())
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .build();

    private final ShuffleboardValue<Double> voltage = ShuffleboardValue.create(0.0, "Voltage", VerticalElevator.class.getSimpleName())
    .build();

    private final SafeCanSparkMax leftMotor = new SafeCanSparkMax(
        16, 
        MotorType.kBrushless,
        isEnabled,
        voltage
    );

    // private final SafeCanSparkMax rightMotor = new SafeCanSparkMax(
    //     15, 
    //     MotorType.kBrushless,
    //     isEnabled,
    //     voltage
    // );

    public VerticalElevatorSetPower() {
        leftMotor.setIdleMode(IdleMode.Coast);
        // rightMotor.setIdleMode(IdleMode.Coast);
        leftMotor.setInverted(false);
        // rightMotor.setInverted(true);
    }

    public void setPower() {
        leftMotor.setPower(01);
        // rightMotor.setPower(1);
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        // rightMotor.setPower(1);
    }
    public void stop() {
        leftMotor.setPower(0);
        // rightMotor.setPower(0);
    }
    
}
