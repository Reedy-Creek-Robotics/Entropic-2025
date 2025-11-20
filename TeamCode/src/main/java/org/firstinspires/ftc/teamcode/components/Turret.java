package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret extends BaseComponent {
    //INFO: an encoder tick value of 0 is the turret pointing straight towards the front of the robot.
    private DcMotorEx turretMotor;
    double ticksPerDeg =  300.0 / 360; //TODO: Change this

    double toleranceDeg = 5;

    public Turret(RobotContext context) {
        super(context);
    }

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setTargetPositionTolerance((int) (ticksPerDeg * toleranceDeg));
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update(){
        telemetry.addData("Turret Pos:", getPositionTicks());
    }

    public void goToDeg(double targetDeg) {
        turretMotor.setTargetPosition((int) (targetDeg * ticksPerDeg));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getPositionTicks(){
        return turretMotor.getCurrentPosition();
    }

    public double getPositionDeg(){
        return getPositionTicks() / ticksPerDeg;
    }


}
