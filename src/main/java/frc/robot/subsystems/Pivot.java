package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TrackedElement.Element;
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
        INTAKELOWCUBE(52),//TODO:Change
        INTAKELOWCONE(54),

        LOWCONE(30),
        LOWCUBE(43.1),

        MIDCONE(LOWCONE.degrees.get()),
        MIDCUBE(53),

        HIGHCONE(LOWCONE.degrees.get()),
        HIGHCUBE(33),

        INTAKEHIGH1CUBE(42),
         //parallel - pickup
        INTAKEHIGH1CONE(42),

        INTAKEHIGH2CUBE(15), //- drop
        INTAKEHIGH2CONE(15),
        HOLD(-22), // straight up

        //AUTO
        INTAKEAUTOCUBE(49.7), //Intaking as far down as possible for cube
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
    private final WriteOnlyDouble Cube = new WriteOnlyDouble(0, "Cube", Pivot.class.getSimpleName());

    // private final WriteOnlyDouble targetPositionWriter = new WriteOnlyDouble(0, "Target Position (Degrees)", Pivot.class.getSimpleName());
    private final WriteOnlyDouble encoderPositionWriter = new WriteOnlyDouble(0, "Encoder Position (Degrees)", Pivot.class.getSimpleName());

    private final MutableBoolean isEnabled = SimpleWidgetBuilder.create(true, "Is Enabled", Pivot.class.getSimpleName())
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
  

        controller = new PIDController(0.024, 0, 0);
        controller.setTolerance(0.10); // meters

        ComplexWidgetBuilder.create(controller, "PID Controller", Pivot.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        setTargetPosition(() -> PivotPosition.START);

        ComplexWidgetBuilder.create(resetPivotEncoder(), "Reset claw encoder", Pivot.class.getSimpleName());
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

    public PivotPosition getPivotPosition() {
        return position;
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

    public CommandBase setTargetPosition(Supplier<PivotPosition> position) {
        return runOnce(() -> {
            this.position = position.get();
            controller.setSetpoint(position.get().degrees.get());
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
            () -> switch(TrackedElement.get()) {
                case CONE -> PivotPosition.INTAKELOWCONE;
                case CUBE -> PivotPosition.INTAKELOWCUBE;
                case NONE -> PivotPosition.INTAKELOWCUBE;
            }
        );
    }
    public CommandBase moveIntake1High() {//The pickup
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> PivotPosition.INTAKEHIGH1CONE;
                case CUBE -> PivotPosition.INTAKEHIGH1CUBE;
                case NONE -> PivotPosition.INTAKEHIGH1CUBE;
            }
        );
    }

    public CommandBase moveIntake2High() {//The drop
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> PivotPosition.INTAKEHIGH2CONE;
                case CUBE -> PivotPosition.INTAKEHIGH2CUBE;
                case NONE -> PivotPosition.INTAKEHIGH2CUBE;
            }
        );
    }
    
    public CommandBase moveLow() {
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> PivotPosition.LOWCONE;
                case CUBE -> PivotPosition.LOWCUBE;
                case NONE -> PivotPosition.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        if(TrackedElement.get()==Element.CUBE){
            Cube.set(1);
        }
        if(TrackedElement.get()==Element.CONE){
            Cube.set(2);
        }
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> PivotPosition.MIDCONE;
                case CUBE -> PivotPosition.MIDCUBE;
                case NONE -> PivotPosition.MIDCUBE;
            }
        );
    }

    public CommandBase moveHigh() {
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> PivotPosition.HIGHCONE;
                case CUBE -> PivotPosition.HIGHCUBE;
                case NONE -> PivotPosition.HIGHCUBE;
            }
        );
    }

    public CommandBase moveHold() {
        return setTargetPosition(() -> PivotPosition.HOLD);
    }

    private void setPower(double power) {
        // if (!isEnabled.get()) return;
        pivotMotor.set(power);
    }

    public CommandBase resetPivotEncoder() {
        return runOnce(() -> {
            offset = -getRawPosition();//Offset?  
        });
    }

    public CommandBase setPowerC(double power) {
        return runOnce(() ->setPower(power));
    }
}  