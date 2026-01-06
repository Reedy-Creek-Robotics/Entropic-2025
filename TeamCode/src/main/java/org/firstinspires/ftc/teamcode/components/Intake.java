package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
public class Intake extends BaseComponent{

    LogCatUtil log;
    HardwareUtil hardwareUtil;

    DcMotorEx intakeMotor;

    Robot robot;

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    boolean alliance;

    /**
     * Will be appended to the prefix defined in LogCatUtil
     */
    static String logTag = "Intake";

    public Intake(RobotContext context, Robot robot) {
        super(context);

        log = new LogCatUtil(logTag);

        hardwareUtil = new HardwareUtil(log, hardwareMap);

        this.alliance = context.alliance;

        this.robot = robot;

        intakeMotor = hardwareUtil.getMotorEx("intake");
    }

    @Override
    public void init() {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }

    public void runIntakeCommand(double power){
        robot.executeCommand(new RunIntake(power));
    }

    public void timedIntake(double power, double timeMs){
        robot.executeCommand(new TimedIntake(power, timeMs));
    }

    private class RunIntake implements Command {
        double power;

        public RunIntake(double power){
            this.power = power;
        }

        @Override
        public void start() {
            intakeMotor.setPower(power);
        }

        @Override
        public void stop() {

        }

        @Override
        public boolean update() {
            return true;
        }
    }

    private class TimedIntake implements Command {
        ElapsedTime timer;
        double power;
        double timeMs;

        public TimedIntake(double power, double timeMs){
            this.power = power;
            this.timeMs = timeMs;
            timer = new ElapsedTime();
        }

        @Override
        public void start() {
            intakeMotor.setPower(power);
        }

        @Override
        public void stop() {
            intakeMotor.setPower(0);
        }

        @Override
        public boolean update() {
            return timer.milliseconds() >= timeMs;
        }
    }

}
