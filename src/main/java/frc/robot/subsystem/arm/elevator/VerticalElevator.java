package frc.robot.subsystem.arm.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.subsystem.arm.pivot.Pivot;
import frc.robot.utility.SafeCanSparkMax;
import frc.robot.utility.ShuffleboardValue;

public class VerticalElevator extends Elevator {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double GEAR_DIAMETER_INCHES = 1.88;
        public static final double COUNTS_PER_PULSE = 1; // 2048 bc rev through bore
        public static final double ROT_TO_INCHES = (COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);
    }
    private final PIDController controller = new PIDController(0.2, 0, 0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);
    private final SafeCanSparkMax leftMotor = new SafeCanSparkMax(
        16, 
        MotorType.kBrushless,
        ShuffleboardValue.create(true, "Is Enabled", VerticalElevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .build(),
        ShuffleboardValue.create(0.0, "Voltage", Pivot.class.getSimpleName())
            .build()
    );
    private final SafeCanSparkMax rightMotor = new SafeCanSparkMax(
        15, 
        MotorType.kBrushless,
        ShuffleboardValue.create(true, "Is Enabled", VerticalElevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .build(),
        ShuffleboardValue.create(0.0, "Voltage", Pivot.class.getSimpleName())
            .build()
    );

    private final RelativeEncoder encoder;

    public VerticalElevator() {
        super();
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, false);

        controller.setTolerance(0.1);

        encoder = leftMotor.getEncoder();
        encoder.setPositionConversionFactor(Constants.ROT_TO_INCHES);
    }

    @Override
    protected PIDController getController() {
        return controller;
    }

    @Override
    protected ElevatorFeedforward getFeedforward() {
        return feedforward;
    }

    @Override
    protected SafeCanSparkMax getMotor() {
        return leftMotor;
    }

    @Override
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public double getEncoderPosition() {
        double position = encoder.getPosition();
        encoderPositionWriter.write(position);
        return position;
    }
    
}
