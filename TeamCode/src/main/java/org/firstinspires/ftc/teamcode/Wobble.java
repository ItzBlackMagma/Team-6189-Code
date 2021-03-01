package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    DcMotor lifter;
    Servo gripper;
    private double power = 0;

    public void init(HardwareMap hm){
        lifter = hm.dcMotor.get("wobble");
        gripper = hm.servo.get("lock");

        lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper.setDirection(Servo.Direction.FORWARD);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power){this.power = power;}

    public void grip(double pos){gripper.setPosition(pos);}

    public void setPos(double pos, int inv){
        if(lifter.getCurrentPosition() < pos){
            power *= inv;
        } else if(lifter.getCurrentPosition() > pos){
            power *= -inv;
        } else {
            power = 0;
        }
        lifter.setPower(power);
    }

    public void setPos(double pos){setPos(pos,1);}

}
