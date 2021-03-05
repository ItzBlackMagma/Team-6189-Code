package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    DcMotor lifter;
    Servo gripper;

    public final double PulsesPerDeg = 28/360, extendedAngle = 220, extendedPos = 202;// extendedAngle * PulsesPerDeg;

    public void init(HardwareMap hm){
        lifter = hm.dcMotor.get("wobble");
        gripper = hm.servo.get("lock");

        lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper.setDirection(Servo.Direction.FORWARD);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void grip(double pos){gripper.setPosition(pos);}

    public void extend(double power){
        lifter.setPower(power);
        lifter.setTargetPosition((int) extendedPos);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // setPos(extendedPos, power);
    }

    public void fold(double power){

        lifter.setPower(power);
        lifter.setTargetPosition(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
        // setPos(0, power);}

    public void setPos(double pos, double power){
        if(lifter.getCurrentPosition() < pos - 1){
            power *= 1;
        } else if(lifter.getCurrentPosition() > pos + 1){
            power *= -1;
        } else {
            power = 0;
        }
        lifter.setPower(power);
    }
}
