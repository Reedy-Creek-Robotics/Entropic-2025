package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
public class transferTest extends OpMode {

    Controller controller;

    Servo servo1;
    Servo servo2;

    double power1 = 0;
    double power2 = 0;

    boolean reverse1;
    boolean reverse2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        power1 = ((controller.analogValue(Controller.AnalogControl.LEFT_TRIGGER) * (reverse1 ? -1 : 1)) + 1) / 2;
        power2 = ((controller.analogValue(Controller.AnalogControl.LEFT_TRIGGER) * (reverse2 ? -1 : 1)) + 1) / 2;

        if(controller.isPressed(Controller.Button.LEFT_BUMPER)){
            reverse1 = !reverse1;
        }
        if(controller.isPressed(Controller.Button.RIGHT_BUMPER)){
            reverse2 = !reverse2;
        }

        servo1.setPosition(power1);
        servo2.setPosition(power2);

        telemetry.addData("Power1", power1);
        telemetry.addData("Power2", power2);
        telemetry.addData("Reverse1", reverse1);
        telemetry.addData("Reverse2", reverse2);
        telemetry.update();
    }
}
