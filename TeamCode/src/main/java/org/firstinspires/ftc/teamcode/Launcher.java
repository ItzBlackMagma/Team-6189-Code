package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {
    DcMotor spin,load;

    public void init(HardwareMap hm){
        load = hm.dcMotor.get("loader");
        spin = hm.dcMotor.get("right spin");

        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        load.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void launch(double power){
        spin.setPower(power);
    }

    public void load(double power){
        load.setPower(power);
    }

}
