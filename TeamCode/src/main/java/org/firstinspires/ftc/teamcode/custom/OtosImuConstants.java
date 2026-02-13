package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the OTOSConstants class. It holds many constants and parameters for the OTOS Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class OtosImuConstants {

    /** The name of the OTOS sensor in the hardware map
     * Default Value: "sensor_otos" */
    public  String otosHardwareMapName = "otos";

    /** The name of the IMU sensor in the hardware map
     * Default Value: "sensor_otos" */
    public  String imuHardwareMapName = "imu";

    public IMU.Parameters imuParameters = new IMU.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.LEFT, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD));

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
     * This creates a new OTOSConstants with default values.
     */
    public OtosImuConstants() {
        defaults();
    }

    public OtosImuConstants otosHardwareMapName(String hardwareMapName) {
        this.otosHardwareMapName = hardwareMapName;
        return this;
    }

    public OtosImuConstants imuHardwareMapName(String hardwareMapName) {
        this.imuHardwareMapName = imuHardwareMapName;
        return this;
    }

    public OtosImuConstants imuParameters(IMU.Parameters imuParameters){
        this.imuParameters = imuParameters;
        return this;
    }

    public OtosImuConstants linearUnit(DistanceUnit linearUnit) {
        this.linearUnit = linearUnit;
        return this;
    }

    public OtosImuConstants angleUnit(AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
        return this;
    }

    public OtosImuConstants offset(SparkFunOTOS.Pose2D offset) {
        this.offset = offset;
        return this;
    }

    public OtosImuConstants linearScalar(double linearScalar) {
        this.linearScalar = linearScalar;
        return this;
    }

    public OtosImuConstants angularScalar(double angularScalar) {
        this.angularScalar = angularScalar;
        return this;
    }

    public void defaults() {
        otosHardwareMapName = "otos";
        imuHardwareMapName = "imu";
        imuParameters = new IMU.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.LEFT, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD));
        linearUnit = DistanceUnit.INCH;
        angleUnit = AngleUnit.RADIANS;
        offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);
        linearScalar = 1.0;
        angularScalar = 1.0;
    }
}
