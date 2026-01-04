package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.game.Controller.AnalogControl.*;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.Robot;

import org.firstinspires.ftc.teamcode.components.BaseComponent;
import org.firstinspires.ftc.teamcode.components.RobotContext;
import org.firstinspires.ftc.teamcode.components.Transfer;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
public class TeloOpMain extends OpMode {

    RobotContext robotContext;
    Robot robot;

    protected Controller driver;

    double drive, strafe, turn;
    
    Transfer transfer;

    SparkFunOTOS.Pose2D startPose      = new SparkFunOTOS.Pose2D(96, 48, Math.toRadians(0)); //ToDo Set this
    SparkFunOTOS.Pose2D largeZoneReset = new SparkFunOTOS.Pose2D(96, 48, Math.toRadians(0)); // share button
    SparkFunOTOS.Pose2D smallZoneReset = new SparkFunOTOS.Pose2D(24, 72, Math.toRadians(0)); // options button

    @Override
    public void init() {
        robotContext = BaseComponent.createRobotContext(this);

        robot = new Robot(this, false);
        driver = new Controller(gamepad1);

        transfer = robot.getTranstake();
        
        robot.init();

        robot.getDriveTrain().getOtos().setPosition(startPose);
    }

    @Override
    public void loop() {
        drive = driver.analogValue(LEFT_STICK_Y);
        strafe = driver.analogValue(LEFT_STICK_X);
        turn = driver.analogValue(RIGHT_STICK_X);

        robot.getDriveTrain().drive(drive, strafe, turn);

        transfer.runIntake(driver.analogValue(RIGHT_TRIGGER) - driver.analogValue(LEFT_TRIGGER));

        if(driver.isButtonDown(Controller.Button.NORTH) || driver.isButtonDown(Controller.Button.RIGHT_BUMPER)){
            transfer.runFrontRoller(1);
        }
        else if(driver.isButtonDown(Controller.Button.SOUTH)){
            transfer.runFrontRoller(-1);
        }else{
            transfer.runFrontRoller(0);
        }

        if(driver.isButtonDown(Controller.Button.DPAD_UP) || driver.isButtonDown(Controller.Button.LEFT_BUMPER)){
            transfer.runRearRoller(1);
        }else if(driver.isButtonDown(Controller.Button.DPAD_DOWN)){
            transfer.runRearRoller(-1);
        }else{
            transfer.runRearRoller(0);
        }

        if(driver.isPressed(Controller.Button.SHARE)){
            robot.getDriveTrain().getOtos().setPosition(largeZoneReset);

        }else if(driver.isPressed(Controller.Button.OPTIONS)){
            robot.getDriveTrain().getOtos().setPosition(smallZoneReset);
        }

        telemetry.addData("alliance", robotContext.getAlliance());
        robot.update();
    }
}
