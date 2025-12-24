package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.game.Controller.AnalogControl.*;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.Robot;

import org.firstinspires.ftc.teamcode.components.BaseComponent;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.RobotContext;
import org.firstinspires.ftc.teamcode.components.Transtake;
import org.firstinspires.ftc.teamcode.game.Controller;


/*
Need to set up:
ToDo: Alliance based distance calculation from OTOS
ToDo:  
 */


@TeleOp
public class TeloOpMain extends OpMode {

    RobotContext robotContext;
    Robot robot;

    protected Controller driver;

    double drive, strafe, turn;
    
    Transtake transtake;

    @Override
    public void init() {
        robotContext = BaseComponent.createRobotContext(this);

        robot = new Robot(this, false);
        driver = new Controller(gamepad1);

        transtake = robot.getTranstake();
        
        robot.init();

        robot.getDriveTrain().getOtos().setPosition(new SparkFunOTOS.Pose2D(96, 96, Math.toRadians(0)));
    }

    @Override
    public void loop() {
        drive = driver.analogValue(LEFT_STICK_Y);
        strafe = driver.analogValue(LEFT_STICK_X);
        turn = driver.analogValue(RIGHT_STICK_X);

        robot.getDriveTrain().drive(drive, strafe, turn);

        transtake.runIntake(driver.analogValue(RIGHT_TRIGGER) - driver.analogValue(LEFT_TRIGGER));

        if(driver.isButtonDown(Controller.Button.NORTH)){
            transtake.runFrontRoller(1);
        }
        else if(driver.isButtonDown(Controller.Button.SOUTH)){
            transtake.runFrontRoller(-1);
        }else{
            transtake.runFrontRoller(0);
        }

        if(driver.isButtonDown(Controller.Button.DPAD_UP)){
            transtake.runRearRoller(1);
        }else if(driver.isButtonDown(Controller.Button.DPAD_DOWN)){
            transtake.runRearRoller(-1);
        }else{
            transtake.runRearRoller(0);
        }
        
        robot.update();
    }
}
