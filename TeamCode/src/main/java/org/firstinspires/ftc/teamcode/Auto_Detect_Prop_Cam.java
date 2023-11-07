package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto_Detect_Prop_Cam")
public class Auto_Detect_Prop_Cam extends LinearOpMode {

    ElapsedTime timmer = new ElapsedTime();
    DcMotor LB_Drive;
    DcMotor RB_Drive;
    DcMotor LF_Drive;
    DcMotor RF_Drive;
    double power = 0.3;
    double park = 0;
    double distanceSensor = 500;
    static final double FEET_PER_METER = 3.28084;
    double current_position = 0;
    double Elevator_Position = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motor Mapping
        LB_Drive = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");

        //setting the direction of motors
        LB_Drive.setDirection(DcMotor.Direction.FORWARD);
        RB_Drive.setDirection(DcMotor.Direction.REVERSE);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.REVERSE);

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        int run_time = 1000;
        int LF_encoder = 0;
        int RF_encoder = 0;
        timmer.reset();
        while(timmer.milliseconds() < run_time){
            LB_Drive.setPower(-power);
            RB_Drive.setPower(-power);
            LF_Drive.setPower(-power);
            RF_Drive.setPower(-power);

            LF_encoder = LF_Drive.getCurrentPosition();
            RF_encoder = RF_Drive.getCurrentPosition();

            telemetry.addData("Forward_Encoder", LF_encoder);
            telemetry.addData("Side_Encoder", RF_encoder);
            telemetry.update();
        }

    }

    }