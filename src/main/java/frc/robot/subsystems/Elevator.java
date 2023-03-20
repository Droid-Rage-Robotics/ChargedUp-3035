package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.SafeCanSparkMax;
import frc.robot.utilities.ShuffleboardValue;

public class Elevator extends SubsystemBase {
    public static class Constants {
        public static final double VERTICAL_GEAR_RATIO = 1 / 1;
        public static final double VERTICAL_GEAR_DIAMETER_INCHES = 1.88;
        public static final double VERTICAL_COUNTS_PER_PULSE = 1; // 2048 bc rev through bore
        public static final double VERTICAL_ROT_TO_INCHES = (VERTICAL_COUNTS_PER_PULSE * VERTICAL_GEAR_RATIO) / (VERTICAL_GEAR_DIAMETER_INCHES * Math.PI);

        public static final double HORIZONTAL_GEAR_RATIO = 1 / 1;
        public static final double HORIZONTAL_GEAR_DIAMETER_INCHES = 1.4;
        public static final double HORIZONTAL_DISTANCE_PER_PULSE = 1; // 1 bc built in encoder
        public static final double HORIZONTAL_ROT_TO_INCHES = (HORIZONTAL_DISTANCE_PER_PULSE * HORIZONTAL_GEAR_RATIO) / (HORIZONTAL_GEAR_DIAMETER_INCHES * Math.PI);
    }
    
    private final SafeCanSparkMax verticalLeftMotor, verticalRightElevator, horizontalMotor;
    private final RelativeEncoder verticalEncoder;
    private final RelativeEncoder horizontalEncoder;
    private final PIDController verticalController;
    private final PIDController horizontalController;
    
    private final ShuffleboardValue<Double> verticalEncoderPositionWriter = ShuffleboardValue.create(0.0, "Vertical Encoder Position (Inches)", Elevator.class.getSimpleName())
        .withSize(1, 3)
        .build();
    private final ShuffleboardValue<Double> horizontalEncoderPositionWriter = ShuffleboardValue.create(0.0, "Horizontal Encoder Position (Inches)", Elevator.class.getSimpleName())
        .withSize(1, 3)
        .build();
    
    public Elevator() {
        ShuffleboardValue<Boolean> verticalEnabled = ShuffleboardValue.create(true, "Vertical Enabled", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .build();

        ShuffleboardValue<Double> verticalPower = ShuffleboardValue.create(0.0, "Vertical Power", Elevator.class.getSimpleName())
            .build();

        verticalLeftMotor = new SafeCanSparkMax(
            16, 
            MotorType.kBrushless,
            verticalEnabled,
            verticalPower
        );

        verticalRightElevator = new SafeCanSparkMax(
            15, 
            MotorType.kBrushless,
            verticalEnabled,
            verticalPower
        );

        horizontalMotor = new SafeCanSparkMax(
            17,
            MotorType.kBrushless,
            ShuffleboardValue.create(true, "Horizontal Enabled", Elevator.class.getSimpleName())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .build(),
            ShuffleboardValue.create(0.0, "Horizontal Power", Elevator.class.getSimpleName())
                .build()
        );

        verticalLeftMotor.setIdleMode(IdleMode.kBrake);
        verticalRightElevator.setIdleMode(IdleMode.kBrake);
        horizontalMotor.setIdleMode(IdleMode.kBrake);
        horizontalMotor.setInverted(true);

        verticalRightElevator.follow(verticalLeftMotor, true);
  
        verticalEncoder = verticalLeftMotor.getEncoder();
        verticalEncoder.setPositionConversionFactor(Constants.VERTICAL_ROT_TO_INCHES);

        horizontalEncoder = horizontalMotor.getEncoder();
        horizontalEncoder.setPositionConversionFactor(Constants.HORIZONTAL_ROT_TO_INCHES);

        verticalController = new PIDController(0.2, 0, 0);//0.15
        verticalController.setTolerance(0.1); // inches 
        horizontalController = new PIDController(0.2, 0, 0);//0.1
        horizontalController.setTolerance(0.10); // inches

        ComplexWidgetBuilder.create(verticalController, "Vertical PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);
        ComplexWidgetBuilder.create(horizontalController, "Horizontal PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        ComplexWidgetBuilder.create(runOnce(this::resetEncoders), "Reset Elevator Encoders", Elevator.class.getSimpleName());
    }

    @Override
    public void periodic() {
        horizontalMotor.set(horizontalController.calculate(getHorizontalEncoderPosition()));
        verticalLeftMotor.set(verticalController.calculate(getVerticalEncoderPosition()));
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public double getTargetVerticalHeight() {
        return verticalController.getSetpoint();
    }

    public double getTargetHorizontalDistance() {
        return horizontalController.getSetpoint();
    }

    public void setPositions(double horizontalPosition, double verticalPosition) {
        verticalController.setSetpoint(verticalPosition);
        horizontalController.setSetpoint(horizontalPosition);
    }

    public void resetEncoders() {
        verticalEncoder.setPosition(0);
        horizontalEncoder.setPosition(0);
    }

    public double getVerticalEncoderPosition() {
        double verticalEncoderPosition = verticalEncoder.getPosition();
        verticalEncoderPositionWriter.set(verticalEncoderPosition);
        return verticalEncoderPosition;
    }
    public double getHorizontalEncoderPosition() {
        double encoderPosition = horizontalEncoder.getPosition();
        horizontalEncoderPositionWriter.set(encoderPosition);
        return encoderPosition;
    }

    public double getVerticalTargetPosition() {
        return verticalController.getSetpoint();
    }

    public double getHorizonotalTargetPosition() {
        return horizontalController.getSetpoint();
    }

    public void setTargetPositionsManually(double x, double y) {
        verticalController.setSetpoint(getVerticalTargetPosition() + (y*0.1));
    }

}  