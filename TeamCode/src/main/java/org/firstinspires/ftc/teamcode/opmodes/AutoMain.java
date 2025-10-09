package org.firstinspires.ftc.teamcode.opmodes;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class AutoMain extends LinearOpMode {
    protected Robot robot;
    protected Follower follower;
    protected PathBuilder builder;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        builder = follower.pathBuilder();

        telemetry.log().add("Wait for start", "");

        waitForStart();

        runPath();

        robot.savePositionToDisk();
    }

    public void initRobot(){
        robot = new Robot(this);
        robot.init();

    }

    public void runPath(){

        robot.waitForCommandsToFinish();
    }
}
