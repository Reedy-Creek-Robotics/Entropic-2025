package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.teamcode.util.log.DataLog;

import java.text.SimpleDateFormat;

@TeleOp
@Configurable
public class ShooterLogger extends OpMode {
    TelemetryManager telem;

    Servo roller1;
    Servo roller2;

    DcMotorEx shooter;
    DataLog dataLog;
    Controller controller;
    VoltageSensor volt;
    int identifier = 0;

    static double graphMin = 0;
    double graphMax = 0;

    static double velocity = 2000;

    static boolean logging = true;

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

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
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

        if(controller.isPressed(Controller.Button.EAST)){
            velocity = 0;
        }

        if(controller.isPressed(Controller.Button.SOUTH)){
            velocity = velocity;
        }

        if(controller.isPressed(Controller.Button.NORTH)){
            logging = !logging;
        }

        if(graphMax - 100 < shooter.getVelocity()){
            graphMax = shooter.getVelocity() + 100;
        }

        shooter.setVelocity(velocity);
        
        telem.addLine("CHANGE IDENTIFIER WITH BUMPERS:");
        telem.addLine("1 - NO WEIGHT    | 2 - LIGHT WEIGHT");
        telem.addLine("3 - HEAVY WEIGHT | 4 - BOTH WEIGHT");
        telem.addLine("Press EAST to set velocity to 0");
        telem.addLine("Press SOUTH to set velocity to 2000");
        telem.addLine("-- Press NORTH to start logging --");
        telem.addData("Logging", logging);
        telem.addData("Identifier", identifier);
        telem.addData("Set Vel", velocity);
        telem.addData("Real Vel", shooter.getVelocity());
        // These are flat lines to keep the graph scale consistent
        telem.addData("Graph Min", graphMin); // graph min will always be 0
        telem.addData("Graph Max", graphMax); // graph max will be set to the highest velocity recorded + 100
        telem.update();


        if (logging) {
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
}
