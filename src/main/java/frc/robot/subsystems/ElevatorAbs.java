package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.WriteOnlyDouble;
import frc.robot.utilities.WriteOnlyString;

public class ElevatorAbs extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double GEAR_DIAMETER_METERS = 0.0481; // 1.893 inches
        public static final double ROT_TO_METER = (GEAR_RATIO * Math.PI * GEAR_DIAMETER_METERS);
    }

    private enum Position {
        START(0,0),//TODO:Input Values

        INTAKELOW(0.3,0.23),
       
        LOWCONE(3, 0.23),
        LOWCUBE(11,0.23),
        
        MIDCONE(9,0.56),
        MIDCUBE(12,0.56),
        
        HIGHCONE(12.7,1.1),
        HIGHCUBE(13,1.1),

        INTAKEHIGH(0,0)
        
        ;

        private MutableDouble verticalMeters;
        private MutableDouble horizontalMeters;

        private Position(double verticalMeters, double horizontalMeters) {
            this.verticalMeters = new MutableDouble(verticalMeters, Position.class.getSimpleName()+"/"+name()+"/Vertical (Meters)", Elevator.class.getSimpleName());
            this.horizontalMeters = new MutableDouble(horizontalMeters, Position.class.getSimpleName()+"/"+name()+"/Horizontal (Meters)", Elevator.class.getSimpleName());
        }
    }
    
    private final CANSparkMax leftElevator, rightElevator, horizMotor;
    private final RelativeEncoder horizEncoder;
    private final PIDController vertController;
    private final PIDController horizController;
    private volatile Position position = Position.START;
    private final SparkMaxAbsoluteEncoder leftAbsoluteEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    public boolean isMovingManually;
    private final WriteOnlyString positionWriter = new WriteOnlyString(position.name(), "Elevator Position", Elevator.class.getSimpleName());
    
    private final WriteOnlyDouble verticalEncoderPositionWriter = new WriteOnlyDouble(0, "Vertical Encoder Position (Meters)", Elevator.class.getSimpleName());
    private final WriteOnlyDouble horizontalEncoderPositionWriter = new WriteOnlyDouble(0, "Horizontal Encoder Position (Meters)", Elevator.class.getSimpleName());
    
    // private final WriteOnlyDouble verticalTargetPositionWriter = new WriteOnlyDouble(0, "Vertical Target Position (Meters)", Elevator.class.getSimpleName());
    // private final WriteOnlyDouble horizontalTargetPositionWriter = new WriteOnlyDouble(0, "Horizontal Target Position (Meters)", Elevator.class.getSimpleName());
    
    public ElevatorAbs() {
        leftElevator = new CANSparkMax(16, MotorType.kBrushless);
        rightElevator = new CANSparkMax(15, MotorType.kBrushless);//TODO: Where is it plugged in?
        horizMotor = new CANSparkMax(17, MotorType.kBrushless);

        leftElevator.setIdleMode(IdleMode.kBrake);
        rightElevator.setIdleMode(IdleMode.kBrake);
        horizMotor.setIdleMode(IdleMode.kBrake);

        rightElevator.follow(leftElevator, true);
  
        horizEncoder = horizMotor.getEncoder();
        horizEncoder.setPositionConversionFactor(Constants.ROT_TO_METER);

        vertController = new PIDController(0.5, 0, 0);
        vertController.setTolerance(0.1); // meters//TODO:How much tolerence?
        horizController = new PIDController(0.5, 0, 0);
        horizController.setTolerance(0.10); // meters

        new ComplexWidgetBuilder(vertController, "Vertical PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController);
        new ComplexWidgetBuilder(horizController, "Horizontal PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController);

        setPosition(Position.START);

        leftAbsoluteEncoder = rightElevator.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);//TODO:TEST
        // leftAbsoluteEncoder = new SparkMaxAbsoluteEncoder(1, SparkMaxAbsoluteEncoder.Type.kDutyCycle);
absoluteEncoder = new DutyCycleEncoder(0);
        leftAbsoluteEncoder.setPositionConversionFactor(Constants.ROT_TO_METER);

        isMovingManually = false;
    }

    @Override
    public void periodic() {
        // verticalEncoderPositionWriter.set(leftAbsoluteEncoder.getPosition());
        // horizontalEncoderPositionWriter.set(horizAbsoluteEncoder.getPosition());

        horizMotor.set(horizController.calculate(horizEncoder.getPosition()));
        // leftElevator.set(vertController.calculate(leftAbsoluteEncoder.getPosition()));
        leftElevator.set(vertController.calculate(absoluteEncoder.getAbsolutePosition()));
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public Position getTargetPosition() {
        return position;
    }

    public double getTargetVerticalHeight() {
        return position.verticalMeters.get();
    }

    public double getTargetHorizontalDistance() {
        return position.horizontalMeters.get();
    }

    public CommandBase setPosition(Position position) {
        return runOnce(() -> {
            this.position = position;

            vertController.setSetpoint(getTargetVerticalHeight());
            horizController.setSetpoint(getTargetHorizontalDistance());

            positionWriter.set(getTargetPosition().name());
        });
    }
    public CommandBase setPosition(double vertPosition, double horizPosition) {
        return runOnce(() -> {
            vertController.setSetpoint(vertPosition);
            horizController.setSetpoint(horizPosition);

            positionWriter.set(getTargetPosition().name()+" (Modified)");
        });
    }

    public CommandBase moveIntakeLow() {
        return setPosition(Position.INTAKELOW);
    }
    public CommandBase moveIntakeHigh() {
        return setPosition(Position.INTAKEHIGH);
    }

    public CommandBase moveLow() {
        return setPosition(
            Position.LOWCONE
            // switch(TrackedElement.get()) {
            //     case CONE -> Position.LOWCONE;
            //     case CUBE -> Position.LOWCUBE;
            //     case NONE -> Position.LOWCUBE;
            // }
        );
    }

    public CommandBase moveMid() {
        return setPosition(
            Position.MIDCONE
            // switch(TrackedElement.get()) {
            //     case CONE -> Position.MIDCONE;
            //     case CUBE -> Position.MIDCUBE;
            //     case NONE -> Position.MIDCUBE;
            // }
        );
    }

    public CommandBase moveHigh() {
        if(isMovingManually){//TODO:Test
            changePosition();
        }
        return setPosition(
            Position.HIGHCONE
            // switch(TrackedElement.get()) {
            //     case CONE -> Position.HIGHCONE;
            //     case CUBE -> Position.HIGHCUBE;
            //     case NONE -> Position.HIGHCUBE;
            // }
        );
    }

    public void changePosition() {
        position.verticalMeters.set(getTargetVerticalHeight());
        position.horizontalMeters.set(getTargetHorizontalDistance());
    }
}  
