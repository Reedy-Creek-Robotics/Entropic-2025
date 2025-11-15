package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.teamcode.util.log.DataLog;

import java.text.SimpleDateFormat;

@TeleOp
public class ShooterLogger extends OpMode {
    TelemetryManager telem;

    Servo roller1;
    Servo roller2;

    DcMotorEx shooter;
    DataLog dataLog;
    Controller controller;
    VoltageSensor volt;
    int identifier = 0;

    static int graphMin = 0;
    static int graphMax = 0;

    double velocity = 2000;

    @Override
    public void init() {
        String timeStamp = new SimpleDateFormat("yyyyMMdd-HHmmss").format(new java.util.Date());
        dataLog = new DataLog("ShooterLog_" + timeStamp);

        telem = PanelsTelemetry.INSTANCE.getTelemetry();

        roller1 = hardwareMap.get(Servo.class, "roller1");
        roller2 = hardwareMap.get(Servo.class, "roller2");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new Controller(gamepad1);

        volt = hardwareMap.voltageSensor.iterator().next();

        roller1.setPosition(1);
        roller2.setPosition(-1);
    }

    @Override
    public void loop() {
        if(controller.isPressed(Controller.Button.DPAD_UP)){
            velocity += 100;
        }

        if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            velocity -= 100;
        }

        if(controller.isPressed(Controller.Button.DPAD_RIGHT)){
            velocity += 20;
        }

        if(controller.isPressed(Controller.Button.DPAD_LEFT)){
            velocity -= 20;
        }

        if(controller.isPressed(Controller.Button.RIGHT_BUMPER)){
            identifier++;
        }

        if(controller.isPressed(Controller.Button.LEFT_BUMPER)){
            identifier--;
        }

        if(controller.isPressed(Controller.Button.CIRCLE)){
            velocity = 0;
        }

        if(controller.isPressed(Controller.Button.CROSS)){
            velocity = 2000;
        }

        shooter.setVelocity(velocity);
        
        telem.addLine("IDENTIFIER:");
        telem.addLine("1 - NO WEIGHT | 2 - LIGHT WEIGHT");
        telem.addLine("3 - HEAVY WEIGHT | 2 - BOTH WEIGHT");
        telem.addLine("");
        telem.addData("Identifier", identifier);
        telem.addData("Set Vel", velocity);
        telem.addData("Real Vel", shooter.getVelocity());
        telem.addData("Graph Min", graphMin);
        telem.addData("Graph Max", graphMax);
        telem.update();

        dataLog.identifier.set(identifier);
        dataLog.current.set(shooter.getCurrent(CurrentUnit.AMPS));
        dataLog.ticks.set(shooter.getCurrentPosition());
        dataLog.setVelocity.set(velocity);
        dataLog.velocity.set(shooter.getVelocity());
        dataLog.motorType.set(shooter.getMotorType().getDescription());
        dataLog.batteryVoltage.set(volt.getVoltage());
        dataLog.writeLine();
    }
}
