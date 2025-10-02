package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.components.BaseComponent;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
public class intakeTest extends OpMode {
    private DcMotorEx intake;
    private DriveTrain driveTrain;
    Controller controller;

    private double drive = 0, turn = 0, strafe = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTrain = new DriveTrain(BaseComponent.createRobotContext(this));
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        drive = controller.leftStickY();
        strafe = controller.leftStickX();
        turn = controller.rightStickX();

        driveTrain.drive(drive,strafe,turn,1);

        intake.setPower(controller.analogValue(Controller.AnalogControl.LEFT_TRIGGER));
    }
}
