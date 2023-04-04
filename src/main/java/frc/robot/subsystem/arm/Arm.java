package frc.robot.subsystem.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.elevator.HorizontalElevator;
import frc.robot.subsystem.arm.elevator.VerticalElevator;
import frc.robot.subsystem.arm.pivot.Pivot;
import frc.robot.utility.ShuffleboardValue;

public class Arm {
    public enum Position {
        START(Value.START, Value.START),
        INTAKE_LOW(Value.INTAKE_LOW_CONE, Value.INTAKE_LOW_CUBE),
        LOW(Value.LOW_CONE, Value.LOW_CUBE),
        MID(Value.MID_CONE, Value.MID_CUBE),
        AUTO_MID(Value.AUTO_MID_CONE, Value.AUTO_MID_CUBE),
        HIGH(Value.HIGH_CONE, Value.HIGH_CUBE),
        INTAKE_HIGH_DOUBLE_SUBSTATION(Value.INTAKE_HIGH_DOUBLE_SUBSTATION_CONE, Value.INTAKE_HIGH_DOUBLE_SUBSTATION_CUBE),
        INTAKE_HIGH_SINGLE_SUBSTATION(Value.INTAKE_HIGH_SINGLE_SUBSTATION_CONE, Value.INTAKE_HIGH_SINGLE_SUBSTATION_CUBE),
        HOLD(Value.HOLD, Value.HOLD),
        ;
        private final Value coneValue;
        private final Value cubeValue;

        private Position(Value coneValue, Value cubeValue) {
            this.coneValue = coneValue;
            this.cubeValue = cubeValue;
        }

        public Value getCurrentValue() {
            return switch (TrackedElement.get()) {
                case CONE -> coneValue;
                case CUBE -> cubeValue;
            };
        }

        public double getVertical() {
            return getCurrentValue().getVertical();
        }

        public double getHorizontal() {
            return getCurrentValue().getHorizontal();
        }

        public double getPivotDegrees() {
            return getCurrentValue().getPivotDegrees();
        }
    }
    public enum Value {//16-17 is MAXXXXXX for vert ; 11 is for horiz
        START(0,0,0),

        INTAKE_LOW_CONE(0,0, 208),//215 or 219 (shoot COnes)
        INTAKE_LOW_CUBE(0,0, 210),
       
        LOW_CONE(0, 0, 200),
        LOW_CUBE(0,0, 145),

        MID_CONE(13.2,11.3,LOW_CONE.getPivotDegrees()), // LOWCONE.pivotAngle.get() 
        MID_CUBE(0,0, 125),
        // MID_CUBE(13.4,10.4, 190),//DON'T REMOVE! - Old Mid Cube

        AUTO_MID_CONE(15.2, 11.5, LOW_CONE.getPivotDegrees()),
        AUTO_MID_CUBE(13,1, 135),
        // AUTO_MID_CUBE(MID_CUBE.getVertical(), MID_CUBE.getHorizontal(), MID_CUBE.getPivotDegrees()), // Should be same as MID_CUBE
        // AUTOMIDCUBE(13.4,10.4, MIDCONE.pivotAngle.get()),

        HIGH_CONE(16.2,12.5, 145.5),
        HIGH_CUBE(15.1,12.5, 131),

        INTAKE_HIGH_DOUBLE_SUBSTATION_CONE(14.9,0, 110),
        INTAKE_HIGH_DOUBLE_SUBSTATION_CUBE(14.9,0, 120),

        INTAKE_HIGH_SINGLE_SUBSTATION_CONE(13.5,0, 110),
        INTAKE_HIGH_SINGLE_SUBSTATION_CUBE(13.5,0,115),

        HOLD(0,0, 50),
        
        ;

        private final ShuffleboardValue<Double> verticalInches;
        private final ShuffleboardValue<Double> horizontalInches;
        private final ShuffleboardValue<Double> pivotAngle;

        private Value(double verticalInches, double horizontalInches, double pivotAngleDegrees) {
            this.verticalInches = ShuffleboardValue.create(verticalInches, Position.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", "Misc")
                .withSize(1, 3)
                .build();
            this.horizontalInches = ShuffleboardValue.create(horizontalInches, Position.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", "Misc")
                .withSize(1, 3)
                .build();
            this.pivotAngle = ShuffleboardValue.create(pivotAngleDegrees, Position.class.getSimpleName()+"/"+name()+"/Pivot Angle", "Misc")
                .withSize(1, 3)
                .build();
        }

        public double getVertical() {
            return verticalInches.get();
        }

        public double getHorizontal() {
            return horizontalInches.get();
        }

        public double getPivotDegrees() {
            return pivotAngle.get();
        }

        public void setVertical(double inches) {
            verticalInches.set(inches);
        }

        public void setHorizontal(double inches) {
            horizontalInches.set(inches);
        }

        public void setPivotDegrees(double degrees) {
            pivotAngle.set(degrees);
        }
    }

    private final VerticalElevator verticalElevator;
    private final HorizontalElevator horizontalElevator;
    private final Pivot pivot;
    private final Intake intake;
    private static Position position = Position.START;
    private final ShuffleboardValue<String> positionWriter = ShuffleboardValue.create(position.name(), "Current Arm Position", "Misc")
        .withSize(1, 3)
        .build();

    public Arm(VerticalElevator verticalElevator, HorizontalElevator horizontalElevator, Pivot pivot, Intake intake) {
        this.verticalElevator = verticalElevator;
        this.horizontalElevator = horizontalElevator;
        this.pivot = pivot;
        this.intake = intake;
    }

    private void logPosition(Position targetPosition) {
        position = targetPosition;
        positionWriter.write(targetPosition.name());
    }

    public static Position getPosition() {
        return position;
    }

    public CommandBase setPositionCommand(Position targetPosition) {// TODO: Fix position to include the intake close
        //TODO: Put in parallel COMAND!!!!!
        return SuppliedCommand.create(() -> Commands.sequence(
            Commands.runOnce(() -> logPosition(targetPosition)),
            switch (targetPosition) {
                case AUTO_MID -> //Commands.sequence(
                    new ParallelCommandGroup(//TODO:ONl mve pivot fgor the cube not cone
                        // verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                        // horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                        // Commands.waitSeconds(1.4),
                        pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                        );
                    
               // );

                case INTAKE_HIGH_DOUBLE_SUBSTATION -> Commands.sequence(
                    verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                    horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                );
                case INTAKE_HIGH_SINGLE_SUBSTATION -> Commands.sequence(
                    verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                    horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                );
                case INTAKE_LOW -> Commands.sequence(
                    verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                    horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                );
                case START -> Commands.sequence(
                    verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                    horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                );
                
                default -> Commands.sequence(
                    // Commands.run(() -> { // todo fix this please
                    //     Value currentValue = position.getCurrentValue();
                        
                    //     if (horizontalElevator.isMovingManually()) 
                    //         currentValue.setHorizontal(horizontalElevator.getTargetPosition());

                    //     if (verticalElevator.isMovingManually()) 
                    //         currentValue.setVertical(verticalElevator.getTargetPosition());

                    //     if (pivot.isMovingManually())
                    //         currentValue.setPivotDegrees(pivot.getTargetPosition());
                    // }),
                    verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                    horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                    // intake.runOnce(()->intake.close(false))//TODO: FIXXXXXX
                );
            }
        ));
    }

    public CommandBase lowerElevatorCommand() {
        return verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(verticalElevator.getTargetPosition() - 7));
    }
}
