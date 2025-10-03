package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Transfer extends BaseComponent{

    CRServo roller;
    ColorSensor color;
    String servoHardwareName;
    String colorHardwareName;

    /*
    This component will take care of all:
        Transfer servo actions
        Color sensors
        LED illumination
     */

    public Transfer(RobotContext context, String servoHardwareName, String colorHardwareName) {
        super(context);
        this.servoHardwareName = servoHardwareName;
        this.colorHardwareName = colorHardwareName;
    }

    @Override
    public void init() {
        roller = hardwareMap.get(CRServo.class, servoHardwareName);
        color = hardwareMap.get(ColorSensor.class, colorHardwareName);
    }

    public void runRoller(double power){
        roller.setPower(power);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        roller.setDirection(direction);
    }
}
