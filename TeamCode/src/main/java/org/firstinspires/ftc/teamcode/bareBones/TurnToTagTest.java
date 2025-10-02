package org.firstinspires.ftc.teamcode.bareBones;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.AprilTag;
import org.firstinspires.ftc.teamcode.components.BaseComponent;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.RobotContext;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class TurnToTagTest extends LinearOpMode {
    private DriveTrain driveTrain;
    private RobotContext context;
    private AprilTag aprilTag;
    Controller controller;

    static Follower follower;
    public static PathBuilder builder;

    private double drive = 0, turn = 0, strafe = 0;

    int homeGoal;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        builder = follower.pathBuilder();
        controller = new Controller(gamepad1);

        context = BaseComponent.createRobotContext(this);

        while(opModeInInit()){
            telemetry.addLine("Press A to change to blue alliance");
            telemetry.addLine("Press B to change to red alliance");
            telemetry.addData("Current Alliance: ", context.alliance == -1 ? "Blue" : "Red");
            telemetry.update();

            if (controller.isPressed(Controller.Button.A)) {
                context.setAlliance(-1);
                break;
            } else if (controller.isPressed(Controller.Button.B)) {
                context.setAlliance(1);
                break;
            }
        }

        homeGoal = context.alliance == -1 ? 20 : 24;
        driveTrain = new DriveTrain(context);
        controller = new Controller(gamepad1);
        aprilTag = new AprilTag(context);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Alliance", context.alliance);
            drive = controller.leftStickY();
            strafe = controller.leftStickX();
            turn = controller.rightStickX();

            driveTrain.drive(drive, strafe, turn, 1);

            if (controller.isPressed(Controller.Button.A)) {
                driveTrain.drive(0, 0, 0, 0);
                double startHeading = 0;
                AprilTagDetection detection = aprilTag.getDetection(homeGoal);
                if(detection.id == -1){
                    telemetry.addLine("NULL DETECTION");
                    break;
                }
                double goalHeading = detection.ftcPose.bearing;
                follower.setStartingPose(new Pose());

                PathChain path = builder
                        .addPath(new BezierLine(new Pose(), new Pose()))
                        .setLinearHeadingInterpolation(startHeading, Math.toRadians(startHeading - goalHeading))
                        .build();
                follower.followPath(path);
                double startTime = getRuntime();
                while (follower.isBusy() && opModeIsActive()) {
                    telemetry.addData("Seconds", getRuntime() - startTime);
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("Start Heading", startHeading);
                    telemetry.addData("Goal Heading", Math.toRadians(startHeading - goalHeading));
                    telemetry.addData("Cur Heading", follower.getPose().getHeading());
                    telemetry.addData("T Value", follower.getCurrentTValue());
                    telemetry.update();

                    follower.update();
                }
                telemetry.clear();
            }
            telemetry.update();
        }
    }
}