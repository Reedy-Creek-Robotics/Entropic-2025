package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto", group = "Auto Modes")
public class AutoBlueFiveWeeks extends OpMode {

    private Follower follower;
    private PathBuilder builder;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private final Pose startPose = new Pose(28, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48, 96, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1PosePrepare = new Pose(48, 84, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(31, 82, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PosePrepare = new Pose(42, 60, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PosePrepare = new Pose(49, 135, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Robot robot;
    private PathChain scorePreload;
    private PathChain end;
    private int pathState;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new Robot(this);
        robot.init();
        follower = Constants.createFollower(hardwareMap);
        builder = follower.pathBuilder();
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
            // These loop the movements of the robot, these must be called continuously in order to work
            robot.update();
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
    }

    private void buildPaths(){
        scorePreload = builder
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        end = builder
                .addPath(new BezierLine(scorePose, pickup1PosePrepare))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PosePrepare.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    int targetVelocity = robot.getShooter1().velocityFromDistance(46);
                    robot.getShooter1().setVelocity(targetVelocity);

                    if(robot.getShooter1().getVelocity() > targetVelocity - 50 && robot.getShooter1().getVelocity() < targetVelocity + 50){
                        robot.getIntake().driveIntake(1);
                        robot.getTransfer1().timedTransfer(5000);

                        robot.waitForCommandsToFinish(6000);
                        robot.getIntake().driveIntake(0);
                        setPathState(2);
                    }

                }
            case 2:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(end,true);
//                    setPathState(2);
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