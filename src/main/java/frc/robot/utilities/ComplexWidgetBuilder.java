package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class ComplexWidgetBuilder {
    private final ComplexWidget complexWidget;

    public ComplexWidgetBuilder(Sendable toAdd, String title, String tab) {
        this.complexWidget = Shuffleboard.getTab(tab)
            .add(title, toAdd);
    }

    private ComplexWidgetBuilder(ComplexWidget complexWidget) {
        this.complexWidget = complexWidget;
    }

    public ComplexWidgetBuilder withPosition(int columnIndex, int rowIndex) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withPosition(columnIndex, rowIndex)
            );
    }

    public ComplexWidgetBuilder withSize(int height, int width) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withSize(height, width)
            );
    }

    public ComplexWidgetBuilder withProperties(Map<String, Object> properties) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withProperties(properties)
            );
    }

    public ComplexWidgetBuilder withWidget(WidgetType widgetType) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withWidget(widgetType)
            );
    }
}