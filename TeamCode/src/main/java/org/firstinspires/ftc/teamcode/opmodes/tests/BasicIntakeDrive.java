package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.Transtake;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
public class BasicIntakeDrive extends OpMode {
    Robot robot;
    Transtake transtake;
    Controller controller;

    @Override
    public void init() {
        robot = new Robot(this);
        transtake = robot.getTranstake();
        controller = new Controller(gamepad1);

        robot.init();
    }

    @Override
    public void loop() {
        transtake.runIntake(controller.analogValue(Controller.AnalogControl.RIGHT_TRIGGER)-controller.analogValue(Controller.AnalogControl.LEFT_TRIGGER));
        if(controller.isButtonDown(Controller.Button.NORTH)){
            transtake.runFrontRoller(1);
        }
        else if(controller.isButtonDown(Controller.Button.SOUTH)){
            transtake.runFrontRoller(-1);
        }else{
            transtake.runFrontRoller(0);
        }

        if(controller.isButtonDown(Controller.Button.DPAD_UP)){
            transtake.runRearRoller(1);
        }else if(controller.isButtonDown(Controller.Button.DPAD_DOWN)){
            transtake.runRearRoller(-1);
        }else{
            transtake.runRearRoller(0);
        }

        robot.getDriveTrain().drive(
                controller.analogValue(Controller.AnalogControl.LEFT_STICK_Y),
                controller.analogValue(Controller.AnalogControl.LEFT_STICK_X),
                controller.analogValue(Controller.AnalogControl.RIGHT_STICK_X),
                1
        );

        robot.update();
    }
}
