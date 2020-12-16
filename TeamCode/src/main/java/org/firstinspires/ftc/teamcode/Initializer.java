package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Initializer {

    Telemetry telemetry;
    //public OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Crossbow module
    public DcMotor sideways, vertical, drawback;
    public DcMotor motor1,motor2,motor3,motor4; //starts with left front and moves clockwise
    //public DcMotor verticalLeft, verticalRight, horizontal; //Odometry motors

    public Servo lock;

    //Chassis module
    public WebcamName webcamName;

    public BNO055IMU imu;

    public Initializer(Telemetry telemetry, DcMotor sideways, DcMotor vertical, DcMotor drawback, DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, Servo lock, WebcamName webcamName, BNO055IMU imu) {
        this.telemetry = telemetry;
        this.sideways = sideways;
        this.vertical = vertical;
        this.drawback = drawback;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor3 = motor3;
        this.motor4 = motor4;
        this.lock = lock;
        this.webcamName = webcamName;
        this.imu = imu;
    }

    //Set everything up
    public void init(HardwareMap hardwareMap){
        telemetry.addData("/> SYSTEM", "Beginning Initialization Process...");

        initIMU(hardwareMap);

        initChassis(hardwareMap);
        //initBow(hardwareMap);
        //initServos(hardwareMap);

        //webcamName = hardwareMap.get(WebcamName.class, "robo eye");

        telemetry.addData("/> INIT", "Webcam Initialized...");
        telemetry.addData("/> SYSTEM", "Initialization Complete");
        telemetry.update();
    }

    private void initIMU(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("/> INIT", "Imu Initialized");

    }

    private void initChassis(HardwareMap hardwareMap) {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("/> INIT", "Chassis Motors Initialized");
    }

    private void initBow(HardwareMap hardwareMap){
        sideways = hardwareMap.dcMotor.get("vertical");
        vertical = hardwareMap.dcMotor.get("horizontal");
        drawback = hardwareMap.dcMotor.get("drawback");

        sideways.setDirection(DcMotor.Direction.FORWARD);
        vertical.setDirection(DcMotor.Direction.FORWARD);
        drawback.setDirection(DcMotor.Direction.FORWARD);

        sideways.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drawback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoders
        sideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drawback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("/> INIT", "Motors Initialized...");
        telemetry.update();
    }

    private void initServos(HardwareMap hardwareMap){
        lock = hardwareMap.servo.get("lock");
        telemetry.addData("/> INIT", "Servos Initialized...");
        telemetry.update();
    }
}
