package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
    public class Colorsensor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;
    private RevBlinkinLedDriver lights;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");
        lights = hardwareMap.get(RevBlinkinLedDriver.class,"Lights");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            //nothing R27 G24 B16
            //green R136 G251 B108 to R328 G644 B277
            //white R1907 G2209 B1852
            //yellow R786 G620 B284 to R1783 G1499 B662
            //purple R1122 G1286 B1520

            String col = "";
            double RtoG = (double)color.red() / (double)color.green();
            double GtoB = (double)color.green() / (double)color.blue();
            double BtoR = (double)color.blue() / (double)color.red();
            if(color.red() < 100 && color.green() < 100 && color.blue() < 100){
                col = "nothing";
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
            else if(color.red() > 1500 && color.green() > 1500 && color.blue() > 1500){
                col = "white";
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
            else if(RtoG < .6) {
                col = "green";
                //ham
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if(BtoR > 1.3){
                col = "purple";
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            else{
                col = "yellow";
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }


            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("RtoG", RtoG);
            telemetry.addData("GtoB", GtoB);
            telemetry.addData("BtoR", BtoR);
            telemetry.addData("Color", col);



            telemetry.update();
        }
    }
}