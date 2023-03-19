package frc.robot.subsystems;

import java.math.RoundingMode;
import java.util.function.Supplier;

import org.ejml.data.ZMatrixRMaj;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EnumPositions.Position;
import frc.robot.subsystems.EnumPositions.TrackedElement;
import frc.robot.subsystems.EnumPositions.Position.Positions;
import frc.robot.subsystems.EnumPositions.TrackedElement.Element;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.ShuffleboardValue;
import frc.robot.utilities.ShuffleboardValueBuilder;
import frc.robot.utilities.ShuffleboardValueEnum;

public class Pivot extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 3 / 1;
        public static final double READINGS_PER_REVOLUTION = 1;
        public static final double ROTATIONS_TO_DEGREES = (GEAR_RATIO * READINGS_PER_REVOLUTION) / 360;
    }

    // public enum PivotPosition {
    //     START(0),
    //     INTAKELOWCUBE(49.9),//TODO:Change
    //     INTAKELOWCONE(51.),

    //     LOWCONE(31.4),
    //     LOWCUBE(43.1),

    //     MIDCONE(LOWCONE.degrees.get()),
    //     MIDCUBE(53),

    //     HIGHCONE(LOWCONE.degrees.get()),
    //     HIGHCUBE(31),

    //     INTAKEHIGH1CUBE(36.9),
    //      //parallel - pickup
    //     INTAKEHIGH1CONE(37),

    //     INTAKEHIGH2CUBE(20), //- drop
    //     INTAKEHIGH2CONE(20),
    //     HOLD(-22), // straight up
    //     //AUTO
    //     // INTAKEAUTOCUBE(49.7), //Intaking as far down as possible for cube
    //     // INTAKRAUTOCONE(50), //intake auto position for cone
    //     ;

    //     private final MutableDouble degrees;

    //     private PivotPosition(double degrees) {
    //         this.degrees = new MutableDouble(degrees, PivotPosition.class.getSimpleName()+"/"+name()+" (Degrees)", Pivot.class.getSimpleName());
    //     }
    // }
    private final CANSparkMax pivotMotor;
    private final PIDController controller;
    // private volatile PivotPosition position = PivotPosition.START;
    private volatile Positions position = Position.Positions.START;
    // private final AbsoluteEncoder pivotAbsoluteEncoder;
    private final RelativeEncoder pivotRelativeEncoder;
    private final ShuffleboardValue<Double> cubeWriter = ShuffleboardValue.create(0.0, "Cube", Pivot.class.getSimpleName()).build();

    // private final WriteOnlyDouble targetPositionWriter = new WriteOnlyDouble(0, "Target Position (Degrees)", Pivot.class.getSimpleName());
    private final ShuffleboardValue<Double> encoderPositionWriter = ShuffleboardValue.create(0.0, "Encoder Position (Degrees)", Pivot.class.getSimpleName()).build();

    private final ShuffleboardValue<Boolean> isEnabled = ShuffleboardValue.create(true, "Is Enabled", Pivot.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .build();

    private double offset = 0;
    
    public Pivot() {
        pivotMotor = new CANSparkMax(18, MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotRelativeEncoder = pivotMotor.getEncoder();
  

        controller = new PIDController(0.024, 0, 0);
        controller.setTolerance(0.10); // meters

        ComplexWidgetBuilder.create(controller, "PID Controller", Pivot.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        setTargetPosition(() -> Positions.START);

        ComplexWidgetBuilder.create(resetPivotEncoder(), "Reset claw encoder", Pivot.class.getSimpleName());
        // pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 70);
        // pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, -25);//TODO:Test
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

    // public PivotPosition getPivotPosition() {
    //     return position;
    // }

    public double getPosition() {
        return pivotRelativeEncoder.getPosition() + offset;
    }

    public double getRawPosition() {
        return pivotRelativeEncoder.getPosition();
    }

    public double getTargetPosition() {
        return controller.getSetpoint();
    }

    // public CommandBase setTargetPosition(Supplier<PivotPosition> position) {
    //     return runOnce(() -> {
    //         this.position = position.get();
    //         controller.setSetpoint(position.get().degrees.get());
    //     });
    // }

    public CommandBase setTargetPosition(Supplier<Positions> position) {
        return runOnce(() -> {
            this.position = position.get();
            Position.set(this.position);
            controller.setSetpoint(Position.getPivotDegrees(this.position));
        });
    }

    // private void setTargetPosition(double position) {
    //         controller.setSetpoint(position);
    // }

    public void setCurrentPositionManually(double position) {
            setTargetPosition(getTargetPosition() + position);
    }
    public CommandBase setTargetPosition(double position) {
        return runOnce(() -> {
            // this.position.degrees.set(position);
            controller.setSetpoint(position);
        });
    }

    public CommandBase moveIntakeLow() {
        // return setTargetPosition(offset)
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> Positions.INTAKELOWCONE;
                case CUBE -> Positions.INTAKELOWCUBE;
            }
        );
    }
    public CommandBase moveIntake1High() {//The pickup
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> Positions.INTAKEHIGH1CONE;
                case CUBE -> Positions.INTAKEHIGH1CUBE;
            }
        );
    }

    public CommandBase moveIntake2High() {//The drop
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> Positions.INTAKEHIGH2CONE;
                case CUBE -> Positions.INTAKEHIGH2CUBE;
            }
        );
    }
    
    public CommandBase moveLow() {
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> Positions.LOWCONE;
                case CUBE -> Positions.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        // if(TrackedElement.get()==Element.CUBE){
        //     Cube.set(1);
        // }
        // if(TrackedElement.get()==Element.CONE){
        //     Cube.set(2);
        // }
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> Positions.MIDCONE;
                case CUBE -> Positions.MIDCUBE;
            }
        );
    }

    public CommandBase moveHigh() {
        return setTargetPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> Positions.HIGHCONE;
                case CUBE -> Positions.HIGHCUBE;
            }
        );
    }

    public CommandBase moveHold() {
        return setTargetPosition(() -> Positions.HOLD);
    }

    private void setPower(double power) {
        pivotMotor.set(power);
    }

    public CommandBase resetPivotEncoder() {
        return runOnce(() -> {
            offset = -getRawPosition();//Offset? - should work 
        });
    }

    public CommandBase setPowerC(double power) {
        return runOnce(() ->setPower(power));
    }
}  