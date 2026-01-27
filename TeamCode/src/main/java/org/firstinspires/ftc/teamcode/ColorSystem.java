package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DecimalFormat;

@Configurable
public class ColorSystem {

    // Declare the HuskyLens
    RevColorSensorV3 colorSensor;

    // Color sensor variables
    public enum SensedColors {
        PURPLE,
        GREEN,
        UNKNOWN
    }
    public static double colorSensorGain = 5; // static so that it can be tuned via Panels


    /** Initializes and sets up the Color Sensor
     * @param hardwareMap the hardwareMap from OpMode that should be looked to when initializing the sensor */
    public void init(HardwareMap hardwareMap) {

        // Get HuskyLens using hardware map passed in from an OpMode
        colorSensor = hardwareMap.get(RevColorSensorV3.class,"color_sensor");

    }


    /** Gets the color of the artifact that the Color Sensor is aimed at */
    public SensedColors getSensedColor() {

        // Read the color data
        NormalizedRGBA rawColors = colorSensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue( rawColors.toColor() );

        // Decide what color is being sensed
        if (hue > 200) {
            return SensedColors.PURPLE;
        } else if (hue > 100) {
            return SensedColors.GREEN;
        } else {
            return SensedColors.UNKNOWN;
        }
    }

    public boolean artifactIsSensed() {
        return getSensedColor() != SensedColors.UNKNOWN;
    }

    public void logSensedData(TelemetryManager telemetryM, DecimalFormat formatter) {

        // Read the color data
        NormalizedRGBA rawColors = colorSensor.getNormalizedColors();

        telemetryM.addData("Sensor red", formatter.format( rawColors.red ));
        telemetryM.addData("Sensor green", formatter.format( rawColors.green ));
        telemetryM.addData("Sensor blue", formatter.format( rawColors.blue ));
        telemetryM.addData("Hue", JavaUtil.colorToHue(rawColors.toColor()));
        telemetryM.addData("Saturation", formatter.format( JavaUtil.colorToSaturation(rawColors.toColor()) ));
        telemetryM.addData("Value", formatter.format( JavaUtil.colorToValue(rawColors.toColor()) ));
        telemetryM.addData("Alpha", formatter.format( rawColors.alpha ));
        telemetryM.addData("Sensor distance", formatter.format( colorSensor.getDistance(DistanceUnit.INCH) ));

        //  Set a goBilda indicator light to show the sensed hue
        //double hue = JavaUtil.colorToHue( rawColors.toColor() );
        //double huePercent = Math.min(/270, 1);
        //light1.setPosition(0.277 + (0.445 * huePercent));
        //0.277 = goBilda Red = HSV 0
        //0.722 = goBilda Purple = HSV 270
    }

}
