package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.utilities.SafeCanSparkMax;
import frc.robot.utilities.ShuffleboardValue;

public class HorizontalElevator extends Elevator {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double GEAR_DIAMETER_INCHES = 1.4;
        public static final double COUNTS_PER_PULSE = 1; // 2048 bc rev through bore
        public static final double ROT_TO_INCHES = (COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);
    }
    private final PIDController controller = new PIDController(0.2, 0, 0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);
    private final SafeCanSparkMax motor = new SafeCanSparkMax(
        16, 
        MotorType.kBrushless,
        ShuffleboardValue.create(true, "Is Enabled", HorizontalElevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .build(),
        ShuffleboardValue.create(0.0, "Voltage", Pivot.class.getSimpleName())
            .build()
    );

    private final RelativeEncoder encoder;

    public HorizontalElevator() {
        super();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);

        controller.setTolerance(0.1);

        encoder = motor.getEncoder();
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
        return motor;
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
