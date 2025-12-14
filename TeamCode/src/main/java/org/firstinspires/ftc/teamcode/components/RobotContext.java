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

    public DriveUtil driveUtil;

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    public boolean alliance;

    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    public RobotContext(OpMode opMode, RobotDescriptor descriptor) {
        this(opMode, descriptor, true);
    }

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    public RobotContext(OpMode opMode, RobotDescriptor descriptor, boolean alliance) {
        this.opMode = opMode;
        this.descriptor = descriptor;
        this.driveUtil = new MecanumUtil();

        //new TwoWheelTrackingLocalizer(opMode.hardwareMap,this.descriptor);
        //blue is true, red is false
        this.alliance = alliance;
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


    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     * @return alliance as a boolean
     */
    public boolean getAlliance() {
        return alliance;
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
}
