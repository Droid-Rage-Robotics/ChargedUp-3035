package frc.robot.utilities;

import java.lang.reflect.Method;
import java.lang.reflect.Type;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public abstract class ShuffleboardValueBuilder<T> {
    protected SimpleWidget simpleWidget;

    protected ShuffleboardValueBuilder(T defaultValue, String title, String tab) {
        this.simpleWidget = Shuffleboard.getTab(tab)
            .add(title, defaultValue);
    }

    public abstract ShuffleboardValue<T> build();

    protected abstract ShuffleboardValueBuilder<T> add(SimpleWidget simpleWidget);

    public ShuffleboardValueBuilder<T> withPosition(int columnIndex, int rowIndex) {
        return add(
            simpleWidget.withPosition(columnIndex, rowIndex)
        );
    }

    public ShuffleboardValueBuilder<T> withSize(int height, int width) {
        return add(
            simpleWidget.withSize(width, height)
        );
    }

    public ShuffleboardValueBuilder<T> withProperties(Map<String, Object> properties) {
        return add(
            simpleWidget.withProperties(properties)
        );
    }

    public ShuffleboardValueBuilder<T> withWidget(WidgetType widgetType) {
        return add(
            simpleWidget.withWidget(widgetType)
        );
    }
}