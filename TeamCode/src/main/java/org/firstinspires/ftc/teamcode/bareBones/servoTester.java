package org.firstinspires.ftc.teamcode.bareBones;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(group = "Barebone Component Testing")
public class servoTester extends OpMode {
    public CRServo right;
    public CRServo left;

    @Override
    public void init() {
        right = hardwareMap.crservo.get("RightIntake");
        left = hardwareMap.crservo.get("LeftIntake");
    }

    @Override
    public void loop() {
        if(gamepad1.square) {
            right.setPower(-1);
            left.setPower(1);
        }else if(gamepad1.circle) {
            right.setPower(1);
            left.setPower(-1);
        }else if(gamepad1.triangle) {
            right.setPower(0);
            left.setPower(0);
        }
    }
}
