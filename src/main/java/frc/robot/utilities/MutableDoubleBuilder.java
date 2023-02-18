package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class MutableDoubleBuilder {
        private final SimpleWidget simpleWidget;
        private final double defaultValue;

        public MutableDoubleBuilder(double defaultValue, String title, String tab) {
            this.defaultValue = defaultValue;
            this.simpleWidget = Shuffleboard.getTab(tab)
                .addPersistent(title, defaultValue);
        }

        private MutableDoubleBuilder(SimpleWidget simpleWidget, double defaultValue) {
            this.simpleWidget = simpleWidget;
            this.defaultValue = defaultValue;
        }

        public MutableDoubleBuilder withPosition(int columnIndex, int rowIndex) {
            return new MutableDoubleBuilder(
                this.simpleWidget.withPosition(columnIndex, rowIndex), 
                this.defaultValue
                );
        }

        public MutableDoubleBuilder withSize(int height, int width) {
            return new MutableDoubleBuilder(
                this.simpleWidget.withSize(height, width), 
                this.defaultValue
                );
        }

        public MutableDoubleBuilder withProperties(Map<String, Object> properties) {
            return new MutableDoubleBuilder(
                this.simpleWidget.withProperties(properties), 
                this.defaultValue
                );
        }

        public MutableDoubleBuilder withWidget(WidgetType widgetType) {
            return new MutableDoubleBuilder(
                this.simpleWidget.withWidget(widgetType), 
                this.defaultValue
                );
        }

        public MutableDouble build() {
            return new MutableDouble(simpleWidget.getEntry(), defaultValue);
        }
    }