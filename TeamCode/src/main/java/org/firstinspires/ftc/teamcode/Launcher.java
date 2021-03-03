package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    DcMotor spin,load,angle;
    Servo push;

    private final double FIRE_POS = 1, RELOAD_POS = .8, ANGLE_CPR = 14, MAX_ANGLE = 45;
    private double power = 0;

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

        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        load.setDirection(DcMotorSimple.Direction.FORWARD);
        angle.setDirection(DcMotorSimple.Direction.FORWARD);

        angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reload();
    }

    public void fire(){push.setPosition(FIRE_POS);}

    public void reload(){push.setPosition(RELOAD_POS);}

    public void launch(double power){spin.setPower(power);}

    public void load(double power){load.setPower(power);}

    public void rotateToAngle(double theta, int inv){
        int pos = (int) ((MAX_ANGLE) - (ANGLE_CPR * theta / 360));

        if(angle.getCurrentPosition() < pos){
            power *= inv;
        } else if(angle.getCurrentPosition() > pos){
            power *= -inv;
        } else {
            power = 0;
        }
        angle.setPower(power);
    }

    public void rotateToAngle(double theta){rotateToAngle(theta,1);}

    public void setAnglePower(double power){this.power = power;}

}
