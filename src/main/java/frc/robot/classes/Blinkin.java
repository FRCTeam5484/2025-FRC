package frc.robot.classes;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {
    public static final Spark blinkin = new Spark(0);
    public static void off(){
        blinkin.set(0.99);
    }    
    public static void white(){
        blinkin.set(0.93);
    }
    public static void blue(){
        blinkin.set(0.87);
    }
    public static void aqua(){
        blinkin.set(0.81);
    }
    public static void green(){
        blinkin.set(0.77);
    }
    public static void yellow(){
        blinkin.set(0.69);
    }
    public static void orange(){
        blinkin.set(0.65);
    }
    public static void red(){
        blinkin.set(0.61);
    }
    public static void teamColorsWaves(){
        blinkin.set(0.53);
    }
    public static void teamColorsSparkle(){
        blinkin.set(0.37);
    }
    public static void strobeWhite(){
        blinkin.set(-0.05);
    }
    public static void strobeGold(){
        blinkin.set(-0.07);
    }
    public static void strobeBlue(){
        blinkin.set(-0.09);
    }
    public static void strobeRed(){
        blinkin.set(-0.11);
    }   
    public static void breathGray(){
        blinkin.set(-0.13);
    }
    public static void breathBlue(){
        blinkin.set(-0.15);
    }
    public static void breathRed(){
        blinkin.set(-0.17);
    } 
    public static void LightChaseGray(){
        blinkin.set(-0.27);
    }
    public static void LightChaseBlue(){
        blinkin.set(-0.29);
    }
    public static void LightChaseRed(){
        blinkin.set(-0.31);
    }
    public static void LarsonGray(){
        blinkin.set(-0.33);
    }
    public static void LarsonRed(){
        blinkin.set(-0.35);
    }
    public static void OceanColorWaves(){
        blinkin.set(-0.41);
    }
    public static void OceanTwinkles(){
        blinkin.set(-0.51);
    }
    public static void OceanSinelon(){
        blinkin.set(-0.75);
    }
    public static void ShotWhite(){
        blinkin.set(-0.81);
    }
    public static void ShotBlue(){
        blinkin.set(-0.83);
    }
    public static void ShotRed(){
        blinkin.set(-0.85);
    }
    public static void OceanRainbox(){
        blinkin.set(-0.95);
    }
}
