package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;

public class Elevator extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double GEAR_DIAMETER_METERS = 0.0481; // 1.893 inches
        public static final double ROT_TO_METER = (GEAR_RATIO * Math.PI * GEAR_DIAMETER_METERS);
    }

    private enum Position {
        START(0,0),//TODO:Input Values

        INTAKELOW(0.3,0.23),
       
        LOWCONE(0.4, 0.23),
        LOWCUBE(0.4,0.23),
        
        MIDCONE(0.95,0.56),
        MIDCUBE(0.64,0.56),
        
        HIGHCONE(1.3,1.1),
        HIGHCUBE(1,1.1),

        INTAKEHIGH(0,0)
        
        ;

        private  double verticalMeters;
        private double horizontalMeters;

        private Position(double verticalMeters, double horizontalMeters) {
            this.verticalMeters = verticalMeters;
            this.horizontalMeters = horizontalMeters;
        }
    }
    
    private final CANSparkMax leftElevator, rightElevator, horizMotor;
    // private final RelativeEncoder encoder;
    private final PIDController vertController;
    private final PIDController horizController;
    private volatile Position position;
    private final SparkMaxAbsoluteEncoder rightAbsoluteEncoder, horizAbsoluteEncoder;
    
    public Elevator() {
        leftElevator = new CANSparkMax(16, MotorType.kBrushless);
        rightElevator = new CANSparkMax(15, MotorType.kBrushless);//TODO: Where is it plugged in?
        horizMotor = new CANSparkMax(17, MotorType.kBrushless);

        leftElevator.setIdleMode(IdleMode.kBrake);
        rightElevator.setIdleMode(IdleMode.kBrake);
        horizMotor.setIdleMode(IdleMode.kBrake);

        rightElevator.follow(leftElevator, true);
  
        // encoder = leftElevator.getEncoder();  //TODO: Where is it plugged in?
        // encoder.setPositionConversionFactor(Constants.ROT_TO_METER);

        vertController = new PIDController(0, 0, 0);
        vertController.setTolerance(0.10); // meters
        horizController = new PIDController(0, 0, 0);
        horizController.setTolerance(0.10); // meters

        setPosition(Position.START);

        rightAbsoluteEncoder = rightElevator.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);//TODO:TEST
        rightAbsoluteEncoder.setPositionConversionFactor(Constants.ROT_TO_METER);
        horizAbsoluteEncoder = horizMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);//TODO:TEST
        horizAbsoluteEncoder.setPositionConversionFactor(Constants.ROT_TO_METER);
    }

    @Override
    public void periodic() {
        
        DroidRageConstants.putNumber("Vertical Elevator/Height", rightAbsoluteEncoder.getPosition());
        DroidRageConstants.putNumber("Horizontal Distance", horizAbsoluteEncoder.getPosition());
        // DroidRageConstants.putNumber("Vertical Elevator/Height", enco 3der.getPosition());

        vertController.setPID(
                DroidRageConstants.getNumber("Vertical Eleveator/P", vertController.getP()),
                DroidRageConstants.getNumber("Vertical Eleveator/I", vertController.getI()),
                DroidRageConstants.getNumber("Vertical Eleveator/D", vertController.getD())
            );
            vertController.setTolerance(
                DroidRageConstants.getNumber("Vertical Eleveator/Tolerance", vertController.getPositionTolerance())
            );
 
            horizController.setPID(
                DroidRageConstants.getNumber("Horizontal Eleveator/P", horizController.getP()),
                DroidRageConstants.getNumber("Horizontal Eleveator/I", horizController.getI()),
                DroidRageConstants.getNumber("Horizontal Eleveator/D", horizController.getD())
            );
            horizController.setTolerance(
                DroidRageConstants.getNumber("Horizontal Eleveator/Tolerance", horizController.getPositionTolerance())
            );
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public double getTargetVerticalHeight() {
        return DroidRageConstants.getNumber("Vertical Elevator/Position/"+ position.name(), position.verticalMeters);
    }

    public double getTargetHorizontalDistance() {
        return DroidRageConstants.getNumber("Horizontal Elevator/Position/"+ position.name(), position.horizontalMeters);
    }

    public CommandBase setPosition(Position position) {
        return runOnce(() -> {
            this.position = position;
            DroidRageConstants.putString("Elevator Position", position.name());

            vertController.setSetpoint(getTargetVerticalHeight());
            horizController.setSetpoint(getTargetHorizontalDistance());
        });
    }
    public CommandBase setPosition(double vertPosition, double horizPosition) {
        return runOnce(() -> {
            DroidRageConstants.putNumber("Vertical Elevator Position ", vertPosition);
            DroidRageConstants.putNumber("Vertical Elevator Position ", horizPosition);

            vertController.setSetpoint(vertPosition);
            horizController.setSetpoint(horizPosition);
        });
    }

    public CommandBase moveIntakeLow() {
        return setPosition(Position.INTAKELOW);
    }
    public CommandBase moveIntakeHigh() {
        return setPosition(Position.INTAKEHIGH);
    }

    public CommandBase moveLow() {
        switch (TrackedElement.get()) {
            case CONE:
                return setPosition(Position.LOWCONE);
            case CUBE:
            case NONE:
            default:
                return setPosition(Position.LOWCUBE);
        }
    }

    public CommandBase moveMid() {
        switch (TrackedElement.get()) {
            case CONE:
                return setPosition(Position.MIDCONE);
            case CUBE:
            case NONE:
            default:
                return setPosition(Position.MIDCUBE);
        }
    }

    public CommandBase moveHigh() {
        switch (TrackedElement.get()) {
            case CONE:
                return setPosition(Position.HIGHCONE);
            case CUBE:
            case NONE:
            default:
                return setPosition(Position.HIGHCUBE);
        }
    }


    // public CommandBase moveToPosition() {
    //     return runOnce(() -> leftElevator.set(vertController.calculate(encoder.getPosition())));
    // }

    public CommandBase changePosition(){
        return runOnce(() ->{
            Position orgPos = position;
            orgPos.verticalMeters = getTargetVerticalHeight();
            orgPos.horizontalMeters = getTargetHorizontalDistance();
        });
    }
}  
