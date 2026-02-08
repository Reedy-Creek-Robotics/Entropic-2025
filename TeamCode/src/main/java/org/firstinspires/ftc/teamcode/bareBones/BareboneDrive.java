package org.firstinspires.ftc.teamcode.bareBones;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.BaseComponent;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp(name = "Barebones Drive", group = "Extras")
public class BareboneDrive extends OpMode {
    private DriveTrain driveTrain;
    Controller controller;

    private double drive = 0, turn = 0, strafe = 0;

    @Override
    public void init() {
        driveTrain = new DriveTrain(BaseComponent.createRobotContext(this));
        driveTrain.init();
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        drive = controller.leftStickY();
        strafe = controller.leftStickX();
        turn = controller.rightStickX();

        driveTrain.drive(drive,strafe,turn,1);
//        telemetry.addLine(driveTrain.getFollower().getPose().toString());
        telemetry.update();
    }
}
