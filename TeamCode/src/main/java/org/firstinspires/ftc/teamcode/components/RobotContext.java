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

    /**
     * 0 - unknown pattern
     * 1 - G P P
     * 2 - P G P
     * 3 - P P G
     */
    public int pattern;

    /** The number of balls in the classifier*/
    public int ballsClassified;

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
    public boolean getAlliance() {return alliance;}
    public void setAlliance(boolean newAlliance) {this.alliance = newAlliance;}

    /**
     * 0 - unknown pattern <br>
     * 1 - G P P <br>
     * 2 - P G P <br>
     * 3 - P P G
     */
    public int getPattern() {return this.pattern;}
    public void setPattern(int newPattern) {this.pattern = newPattern;}

    /**
     * @return the number of balls the robot believes are in the classifier
     */
    public int getBallsClassified() {return this.ballsClassified;}
    public void setBallsClassified(int newBallsClassified) {this.ballsClassified = newBallsClassified;}
    public int incrementBallsClassified(int ballsShot) {return this.ballsClassified += ballsShot;}

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
