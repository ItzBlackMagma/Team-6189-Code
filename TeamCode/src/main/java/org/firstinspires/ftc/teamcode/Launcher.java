package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    DcMotor spin,load,angle;
    Servo push;

    private final double FIRE_POS = 1, RELOAD_POS = .8, ANGLE_CPR = 28;
    public boolean isReady = true;

    public void init(HardwareMap hm){
        load = hm.dcMotor.get("loader");
        spin = hm.dcMotor.get("right spin");
        angle = hm.dcMotor.get("angle");
        push = hm.get(Servo.class, "push");

        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        push.setDirection(Servo.Direction.FORWARD);

        spin.setDirection(DcMotorSimple.Direction.FORWARD);
        load.setDirection(DcMotorSimple.Direction.FORWARD);
        angle.setDirection(DcMotorSimple.Direction.FORWARD);

        reload();
    }

    public void fire(){
        if(isReady) {
            push.setPosition(FIRE_POS);
            isReady = false;
        }
    }

    public void reload(){
        push.setPosition(RELOAD_POS);
        isReady = true;
    }

    public void launch(double power){
        spin.setPower(power);
    }

    public void load(double power){
        load.setPower(power);
    }

    public void rotateToAngle(double theta, double power){
        angle.setPower(power);
        angle.setTargetPosition((int)((ANGLE_CPR * theta) / 360));
        angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
