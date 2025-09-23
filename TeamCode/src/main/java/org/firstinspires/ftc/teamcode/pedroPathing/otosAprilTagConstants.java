package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the otosAprilTagConstants class. It holds many constants and parameters for the OTOS Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class otosAprilTagConstants {

    /** The name of the OTOS sensor in the hardware map
     * Default Value: "sensor_otos" */
    public  String otosHardwareMapName = "sensor_otos";

    /** The name of the camera in the hardware map
     * Default Value: "Webcam 1" */
    public  String cameraHardwareMapName = "Webcam 1";

    /** The linear unit of the OTOS sensor
     * Default Value: DistanceUnit.INCH */
    public  DistanceUnit linearUnit = DistanceUnit.INCH;

    /** The angle unit of the OTOS sensor
     * Default Value: AngleUnit.RADIANS */
    public  AngleUnit angleUnit = AngleUnit.RADIANS;

    /** The offset of the OTOS sensor from the center of the robot
     * For the OTOS, left/right is the y axis and forward/backward is the x axis, with left being positive y and forward being positive x.
     * PI/2 radians is facing forward, and clockwise rotation is negative rotation.
     * Default Value: new Pose2D(0, 0, Math.PI / 2) */
    public  SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);

    /** The linear scalar of the OTOS sensor
     * Default Value: 1.0 */
    public  double linearScalar = 1.0;

    /** The angular scalar of the OTOS sensor
     * Default Value: 1.0 */
    public  double angularScalar = 1.0;

    /**
     * This creates a new otosAprilTagConstants with default values.
     */
    public otosAprilTagConstants() {
        defaults();
    }

    public otosAprilTagConstants otosHardwareMapName(String otosHardwareMapName) {
        this.otosHardwareMapName = otosHardwareMapName;
        return this;
    }

    public otosAprilTagConstants cameraHardwareMapName(String cameraHardwareMapName){
        this.cameraHardwareMapName = cameraHardwareMapName;
        return this;
    }

    public otosAprilTagConstants linearUnit(DistanceUnit linearUnit) {
        this.linearUnit = linearUnit;
        return this;
    }

    public otosAprilTagConstants angleUnit(AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
        return this;
    }

    public otosAprilTagConstants offset(SparkFunOTOS.Pose2D offset) {
        this.offset = offset;
        return this;
    }

    public otosAprilTagConstants linearScalar(double linearScalar) {
        this.linearScalar = linearScalar;
        return this;
    }

    public otosAprilTagConstants angularScalar(double angularScalar) {
        this.angularScalar = angularScalar;
        return this;
    }

    public void defaults() {
        otosHardwareMapName = "sensor_otos";
        cameraHardwareMapName = "Webcam 1";
        linearUnit = DistanceUnit.INCH;
        angleUnit = AngleUnit.RADIANS;
        offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);
        linearScalar = 1.0;
        angularScalar = 1.0;
    }
}
