package frc.robot.subsystem.arm.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.commands.DisabledCommand;
import frc.robot.utility.ComplexWidgetBuilder;
import frc.robot.utility.SafeCanSparkMax;
import frc.robot.utility.ShuffleboardValue;
import frc.robot.utility.SafeMotor.IdleMode;

public class VerticalElevator extends Elevator {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double GEAR_DIAMETER_INCHES = 1.88;
        public static final double COUNTS_PER_PULSE = 1; // 2048 bc rev through bore
        public static final double ROT_TO_INCHES = (COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);
        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 16.2;
    }
    private final PIDController controller = new PIDController(2.4, 0, 0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.1, 0.2, 0, 0);
    // private final ShuffleboardValue<Boolean> isEnabledLeft = ShuffleboardValue.create(true, "Is Enabled Left", VerticalElevator.class.getSimpleName())
    //     .withWidget(BuiltInWidgets.kToggleSwitch)
    //     .build();
    // private final ShuffleboardValue<Boolean> isEnabledRight = ShuffleboardValue.create(false, "Is Enabled R", VerticalElevator.class.getSimpleName())
    //     .withWidget(BuiltInWidgets.kToggleSwitch)
    //     .build();
    private final ShuffleboardValue<Double> voltage = ShuffleboardValue.create(0.0, "Voltage", VerticalElevator.class.getSimpleName())
        .build();

    private final SafeCanSparkMax leftMotor;

    private final SafeCanSparkMax rightMotor;

    protected final ShuffleboardValue<Double> encoderPositionWriter = ShuffleboardValue.create(0.0, "Encoder Position", VerticalElevator.class.getSimpleName())
        .withSize(1, 3)
        .build();

    protected final ShuffleboardValue<Boolean> isMovingManually = ShuffleboardValue.create(false, "Moving manually", VerticalElevator.class.getSimpleName())
        .build();

    private final RelativeEncoder encoder;

    public VerticalElevator(Boolean isEnabledLeft, Boolean isEnabledRight) {
        leftMotor = new SafeCanSparkMax(
            16, 
            MotorType.kBrushless,
            ShuffleboardValue.create(isEnabledLeft, "Is Enabled Left", VerticalElevator.class.getSimpleName())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .build(),
            voltage
        );

        rightMotor = new SafeCanSparkMax(
            15, 
            MotorType.kBrushless,
            ShuffleboardValue.create(isEnabledRight, "Is Enabled Right", VerticalElevator.class.getSimpleName())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .build(),
            voltage
        );
        leftMotor.setIdleMode(IdleMode.Brake);
        rightMotor.setIdleMode(IdleMode.Brake);
        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
        
        // rightMotor.setInverted(true);

        controller.setTolerance(0.1);

        encoder = leftMotor.getEncoder();
        // encoder = rightMotor.getEncoder();
        encoder.setPositionConversionFactor(Constants.ROT_TO_INCHES);

        ComplexWidgetBuilder.create(getController(), " PID Controller", VerticalElevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        ComplexWidgetBuilder.create(DisabledCommand.create(runOnce(this::resetEncoder)), "Reset Encoder", VerticalElevator.class.getSimpleName());
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
    protected void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        // rightMotor.setVoltage(voltage);
    }

    @Override
    protected ShuffleboardValue<Boolean> getIsMovingManually() {
        return isMovingManually;
    }

    @Override
    public double getMaxPosition() {
        return Constants.MAX_POSITION;
    }

    @Override
    public double getMinPosition() {
        return Constants.MIN_POSITION;
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
