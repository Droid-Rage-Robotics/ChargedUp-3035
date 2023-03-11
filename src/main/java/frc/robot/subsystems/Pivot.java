package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.MutableBoolean;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.SimpleWidgetBuilder;
import frc.robot.utilities.WriteOnlyDouble;

public class Pivot extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 3 / 1;
        public static final double READINGS_PER_REVOLUTION = 1;
        public static final double ROTATIONS_TO_DEGREES = (GEAR_RATIO * READINGS_PER_REVOLUTION) / 360;
    }

    public enum PivotPosition {
        START(0),
        INTAKELOWCUBE(53),//TODO:Change
        INTAKELOWCONE(56),

        LOWCONE(40.8),
        LOWCUBE(43.7),

        MIDCONE(LOWCONE.degrees.get()),
        MIDCUBE(45),

        HIGHCONE(LOWCONE.degrees.get()),
        HIGHCUBE(30),

        INTAKEHIGHCUBE(43), //parallel
        INTAKEHIGHCONE(43),
        HOLD(0), // straight up

        //AUTO
        INTAKEAUTOCUBE(49.7), //Intaking as far down as possible for dube
        INTAKRAUTOCONE(50), //intake auto position for cone
        ;

        private final MutableDouble degrees;

        private PivotPosition(double degrees) {
            this.degrees = new MutableDouble(degrees, PivotPosition.class.getSimpleName()+"/"+name()+" (Degrees)", Pivot.class.getSimpleName());
        }
    }
    private final CANSparkMax pivotMotor;
    private final PIDController controller;
    private volatile PivotPosition position = PivotPosition.START;
    // private final AbsoluteEncoder pivotAbsoluteEncoder;
    private final RelativeEncoder pivotRelativeEncoder;
    

    // private final WriteOnlyDouble targetPositionWriter = new WriteOnlyDouble(0, "Target Position (Degrees)", Pivot.class.getSimpleName());
    private final WriteOnlyDouble encoderPositionWriter = new WriteOnlyDouble(0, "Encoder Position (Degrees)", Pivot.class.getSimpleName());

    private final MutableBoolean isEnabled = new SimpleWidgetBuilder<Boolean>(true, "Is Enabled", Pivot.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();

    private double offset = 0;
    
    public Pivot() {
        pivotMotor = new CANSparkMax(18, MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotRelativeEncoder = pivotMotor.getEncoder();
        // pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); //TODO:Test // 8192
        // pivotAbsoluteEncoder.setPositionConversionFactor(Constants.ROTATIONS_TO_DEGREES);
        // armAbsoluteEncoder.setInverted(false);
        // armMotor.getForwardLimitSwitch(null);//What does this do
  

        controller = new PIDController(0.023, 0, 0);
        controller.setTolerance(0.10); // meters

        new ComplexWidgetBuilder(controller, "PID Controller", Pivot.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        setTargetPosition(PivotPosition.START);

        new ComplexWidgetBuilder(resetClawEncoder(), "Reset claw encoder", Pivot.class.getSimpleName());
        pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 15);
        pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);//TODO:Test
    }

    @Override
    public void periodic() {
        encoderPositionWriter.set(getPosition());
        setPower(controller.calculate(getPosition()));
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }


    public double getPosition() {
        return pivotRelativeEncoder.getPosition() + offset;
    }

    public double getRawPosition() {
        return pivotRelativeEncoder.getPosition();
    }

    public double getTargetPosition() {
        return controller.getSetpoint();
    }

    private CommandBase setTargetPosition(PivotPosition position) {
        return runOnce(() -> {
            this.position = position;
            controller.setSetpoint(position.degrees.get());
        });
    }

    private void setTargetPosition(double position) {

            controller.setSetpoint(position);
    }
    public void setCurrentPositionManually(double position) {

            setTargetPosition(getTargetPosition() + position);
    }
    // public CommandBase setTargetPosition(double position) {
    //     return runOnce(() -> {
    //         this.position.degrees.set(position);
    //         controller.setSetpoint(position);
    //     });
    // }

    public CommandBase moveIntakeLow() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.INTAKELOWCONE;
                case CUBE -> PivotPosition.INTAKELOWCUBE;
                case NONE -> PivotPosition.INTAKELOWCUBE;
            }
        );
    }
    public CommandBase moveIntakeHigh() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.INTAKEHIGHCONE;
                case CUBE -> PivotPosition.INTAKEHIGHCUBE;
                case NONE -> PivotPosition.INTAKEHIGHCUBE;
            }
        );
    }
    
    public CommandBase moveLow() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.LOWCONE;
                case CUBE -> PivotPosition.LOWCUBE;
                case NONE -> PivotPosition.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.MIDCONE;
                case CUBE -> PivotPosition.MIDCUBE;
                case NONE -> PivotPosition.MIDCUBE;
            }
        );
    }

    public CommandBase moveHigh() {
        return setTargetPosition(
            switch(TrackedElement.get()) {
                case CONE -> PivotPosition.HIGHCONE;
                case CUBE -> PivotPosition.HIGHCUBE;
                case NONE -> PivotPosition.HIGHCUBE;
            }
        );
    }

    public CommandBase moveHold() {
        return setTargetPosition(PivotPosition.HOLD);
    }

    private void setPower(double power) {
        // if (!isEnabled.get()) return;
        pivotMotor.set(power);
    }

    public CommandBase resetClawEncoder() {
        return runOnce(() -> {
            offset = -getRawPosition();
        });
    }

    public CommandBase setPowerC(double power) {
        return runOnce(() ->setPower(power));
    }
}  