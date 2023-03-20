package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.utilities.ShuffleboardValue;

public class Arm {
    public enum Position {
        START(Value.START, Value.START),
        INTAKE_LOW(Value.INTAKELOWCONE, Value.INTAKELOWCUBE),
        LOW(Value.LOWCONE, Value.LOWCUBE),
        MID(Value.MIDCONE, Value.MIDCUBE),
        AUTO_MID(Value.AUTOMIDCONE, Value.AUTOMIDCUBE),
        HIGH(Value.HIGHCONE, Value.HIGHCUBE),
        INTAKE_HIGH_1(Value.INTAKEHIGH1CONE, Value.INTAKEHIGH1CUBE),
        INTAKE_HIGH_2(Value.INTAKEHIGH2CONE, Value.INTAKEHIGH2CUBE),
        HOLD(Value.HOLD, Value.HOLD),
        ;
        private final Value coneValue;
        private final Value cubeValue;

        private Position(Value coneValue, Value cubeValue) {
            this.coneValue = coneValue;
            this.cubeValue = cubeValue;
        }
        private Value getCurrentValue() {
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
    private enum Value {//16-17 is MAXXXXXX for vert ; 11 is for horiz
        START(0,0,0),

        INTAKELOWCONE(0,0, 51),
        INTAKELOWCUBE(0,0, 49.9),
       
        LOWCONE(0, 0, 31.4),
        LOWCUBE(0,0, 43.1),
        
        MIDCONE(13.2,11,31.4), // LOWCONE.pivotAngle.get() 
        MIDCUBE(13.4,10.4, 53),

        AUTOMIDCONE(15.2, 11.5, 31.4),// MIDCONE.pivotAngle.get() // up first then out
        AUTOMIDCUBE(15.2, 11.5, 31.4),// MIDCONE.pivotAngle.get() // up first then out
        // AUTOMIDCUBE(13.4,10.4, MIDCONE.pivotAngle.get()),

        HIGHCONE(15.5,11, 31.4),// LOWCONE.pivotAngle.get()
        HIGHCUBE(15,11, 31),

        INTAKEHIGH1CONE(14.9,0, 37),
        INTAKEHIGH1CUBE(14.9,0, 36.9),

        INTAKEHIGH2CONE(13.5,0, 20),
        INTAKEHIGH2CUBE(13.5,0,20),

        HOLD(0,0, -22),
        
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
    }

    private final Elevator elevator;
    private final Pivot pivot;
    private Position position = Position.START;
    private final ShuffleboardValue<String> positionWriter = ShuffleboardValue.create(position.name(), "Current Arm Position", "Misc")
        .withSize(1, 3)
        .build();

    public Arm(Elevator elevator, Pivot pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
    }

    private void logPosition(Position targetPosition) {
        position = targetPosition;
        positionWriter.write(targetPosition.name());
    }

    public Position getPosition() {
        return position;
    }

    public CommandBase setPositionCommand(Position targetPosition) {
        return new SuppliedCommand(() -> Commands.sequence(
            Commands.runOnce(() -> logPosition(targetPosition)),
            switch (targetPosition) {
                case AUTO_MID -> Commands.sequence(
                    elevator.runOnce(() -> elevator.setPositions(targetPosition.getHorizontal(), targetPosition.getVertical())),
                    Commands.waitSeconds(1.4),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                ); 
                
                default -> Commands.sequence(
                    elevator.runOnce(() -> elevator.setPositions(targetPosition.getHorizontal(), targetPosition.getVertical())),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                );
            }
        ));
    }

    public CommandBase lowerElevatorCommand() {
        return elevator.runOnce(() -> elevator.setPositions(position.getHorizontal(), position.getVertical() - 7));
    }
}
