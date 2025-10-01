package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.DriveUtil;
import org.firstinspires.ftc.teamcode.util.MecanumUtil;

import java.util.ArrayList;
import java.util.List;

public class RobotContext {

    public OpMode opMode;

    public RobotDescriptor descriptor;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public DriveUtil driveUtil;

    public int alliance;

    public WebcamName webcam;

    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    public RobotContext(OpMode opMode, RobotDescriptor descriptor) {
        this.opMode = opMode;
        this.descriptor = descriptor;
        this.driveUtil = new MecanumUtil();

        //new TwoWheelTrackingLocalizer(opMode.hardwareMap,this.descriptor);
        //blue is negative one, red is positive one
        this.alliance = -1;
        this.webcam = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public OpMode getOpMode() {
        return opMode;
    }

    public RobotDescriptor getDescriptor() {
        return descriptor;
    }



    public DriveUtil getDriveUtil() {
        return driveUtil;
    }

    public int getAlliance() {
        return alliance;
    }

    public WebcamName getWebcam() {
        return webcam;
    }

    /*public enum Alliance{
        BLUE(1,-90),
        RED(-1,90);

        int value;
        int rotation;

        Alliance(int value, int rotation) {
            this.value = value;
            this.rotation = rotation;
        }

        public int getValue() {
            return value;
        }

        public int getRotation() {
            return rotation;
        }
    }*/

    /**
     * Represents A component that knows how to obtain the robot's current position.
     */


}
