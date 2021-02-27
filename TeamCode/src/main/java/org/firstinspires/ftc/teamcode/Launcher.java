package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    DcMotor spin,load;
    Servo push;
    private final double FIRE_POS = 1, RELOAD_POS = .8;
    public boolean isReady = true;

    public void init(HardwareMap hm){
        load = hm.dcMotor.get("loader");
        spin = hm.dcMotor.get("right spin");
        push = hm.get(Servo.class, "push");

        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        push.setDirection(Servo.Direction.FORWARD);

        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        load.setDirection(DcMotorSimple.Direction.FORWARD);

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

}
