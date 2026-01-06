package org.firstinspires.ftc.teamcode.opmodes.componentTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;

@TeleOp
@Disabled
public class TurretTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        telemetry.addLine("Ensure the HardwareMap only defines the turret, otos, and camera");
        telemetry.addLine("LogCat should throw several errors for Components missing on start");
        telemetry.addLine("When you are ready, start the OpMode.");
        telemetry.addLine();
        telemetry.addLine("The turret will start auto tracking based on the otos, method");
        telemetry.addLine("can be changed from Panels.");
        telemetry.addLine();
        telemetry.addLine("Press START when you are ready");
    }

    @Override
    public void start() {
        super.start();
        robot = new Robot(this);
        robot.init();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
