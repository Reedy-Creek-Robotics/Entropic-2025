
/*
ToDo Alliance selection in code
ToDo Save position auto > tele op2
 */



package org.firstinspires.ftc.teamcode.pedro; // make sure this aligns with class location

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

@Autonomous(name = "Test Auto", group = "!Pedro Pathing")
public class TestAuto extends OpMode {

    Robot robot;

    LogCatUtil log;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    
    TelemetryManager panelsTelemetry;

    private final Pose startPose = new Pose(112, 134.5, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(96, 96, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1PoseStart = new Pose(96, 84, Math.toRadians(0));
    private final Pose pickup1PoseEnd = new Pose(120, 84, Math.toRadians(0));
    private final Pose endPose = new Pose(96, 60, Math.toRadians(0));

    @Override
    public void init() {
        log = new LogCatUtil("OpMode");

        robot = new Robot(this, false);
        robot.init();
        
        panelsTelemetry = robot.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = robot.getDriveTrain().getFollower();
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        robot.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        panelsTelemetry.addData("path state", pathState);
        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("heading", follower.getPose().getHeading());
        panelsTelemetry.addData("robot busy", robot.isBusy());
    }

    private Path scorePreload;
    private PathChain startPickup1, grabPickup1, scorePickup1, preparePickup2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        startPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseStart.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .setLinearHeadingInterpolation(pickup1PoseStart.getHeading(), pickup1PoseEnd.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseEnd, scorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose.getHeading())
                .build();

        preparePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                log.info("Switch to path state: " + pathState);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    log.info("Done with Path State " + pathState);
                    robot.getIntake().setIntakePower(0.2);
                    robot.getTransfer().rollersForTime(1, 5000);
                    setPathState(2);
                }
                break;

            case 2:
                if(!robot.isBusy()) {
                    robot.getIntake().setIntakePower(0);
                    log.info("Done with Path State " + pathState);
                    follower.followPath(startPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    log.info("Done with Path State " + pathState);
                    robot.getIntake().runIntakeCommand(1);
                    follower.followPath(grabPickup1, 0.25,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    log.info("Done with Path State " + pathState);
                    robot.getIntake().runIntakeCommand(0);
                    follower.followPath(scorePickup1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    log.info("Done with Path State " + pathState);
                    robot.getIntake().setIntakePower(0.2);
                    robot.getTransfer().rollersForTime(1, 10000);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!robot.isBusy()) {
                    log.info("Done with Path State " + pathState);
                    robot.getIntake().setIntakePower(0);
                    follower.followPath(preparePickup2,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    log.info("Done with Path State " + pathState);
                    stop();
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}