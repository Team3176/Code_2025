package com.team3176.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ReefScapeConstants {
    public static final double STALK_A_XPOS = 2.389;
    public static final double STALK_A_YPOS = 4.203;
    public static final double STALK_A_DEG = -90;

    public static final double STALK_B_XPOS = 2.389;
    public static final double STALK_B_YPOS = 3.797;
    public static final double STALK_B_DEG = -90;

    public static final double STALK_C_XPOS = 3.779;
    public static final double STALK_C_YPOS = 3.097;
    public static final double STALK_C_DEG = -30;

    public static final double STALK_D_XPOS = 3.907;
    public static final double STALK_D_YPOS = 2.942;
    public static final double STALK_D_DEG = -30;

    public static final double STALK_E_XPOS = 4.903;
    public static final double STALK_E_YPOS = 2.942;
    public static final double STALK_E_DEG = 30;

    public static final double STALK_F_XPOS = 5.175;
    public static final double STALK_F_YPOS = 3.097;
    public static final double STALK_F_DEG = 30;

    public static final double STALK_G_XPOS = 5.611;
    public static final double STALK_G_YPOS = 3.797;
    public static final double STALK_G_DEG = 90;

    public static final double STALK_H_XPOS = 5.611;
    public static final double STALK_H_YPOS = 4.203;
    public static final double STALK_H_DEG = 90;

    public static final double STALK_I_XPOS = 5.175;
    public static final double STALK_I_YPOS = 4.903;
    public static final double STALK_I_DEG = 150;

    public static final double STALK_J_XPOS = 4.903;
    public static final double STALK_J_YPOS = 5.058;
    public static final double STALK_J_DEG = 150;

    public static final double STALK_K_XPOS = 3.907;
    public static final double STALK_K_YPOS = 5.058;
    public static final double STALK_K_DEG = -150;

    public static final double STALK_L_XPOS = 3.779;
    public static final double STALK_L_YPOS = 4.903;
    public static final double STALK_L_DEG = -150;

    public static final double LEFT_HF_XPOS = 1.289;
    public static final double LEFT_HF_YPOS = 7.07;
    public static final double LEFT_HF_DEG = -145;

    public static final double RIGHT_HF_XPOS = 1.289;
    public static final double RIGHT_HF_YPOS = 0.93;
    public static final double RIGHT_HF_DEG = -35;

    //ID BLUE 14 / RED 5
    public static final double LEFTSIDE_LEFTCLIMB_XPOS = 8;
    public static final double LEFTSIDE_LEFTCLIMB_YPOS = 7.25;
    public static final double LEFTSIDE_LEFTCLIMB_DEG = 0;

    public static final double LEFTSIDE_MIDDLECLIMB_XPOS = 8;
    public static final double LEFTSIDE_MIDDLECLIMB_YPOS = 6.15;
    public static final double LEFTSIDE_MIDDLECLIMB_DEG = 0;

    public static final double LEFTSIDE_RIGHTCLIMB_XPOS = 8;
    public static final double LEFTSIDE_RIGHTCLIMB_YPOS = 5.05;
    public static final double LEFTSIDE_RIGHTCLIMB_DEG = 0;

    //ID BLUE 15 / RED 4
    public static final double RIGHTSIDE_LEFTCLIMB_XPOS = 8;
    public static final double RIGHTSIDE_LEFTCLIMB_YPOS = 2.95;
    public static final double RIGHTSIDE_LEFTCLIMB_DEG = 0;

    public static final double RIGHTSIDE_MIDDLECLIMB_XPOS = 8;
    public static final double RIGHTSIDE_MIDDLECLIMB_YPOS = 1.85;
    public static final double RIGHTSIDE_MIDDLECLIMB_DEG = 0;

    public static final double RIGHTSIDE_RIGHTCLIMB_XPOS = 8;
    public static final double RIGHTSIDE_RIGHTCLIMB_YPOS = 0.75;
    public static final double RIGHTSIDE_RIGHTCLIMB_DEG = 0;

    //ID BLUE 18 / RED 7
    public static final double REEFFACE_1_XPOS = (STALK_A_XPOS+STALK_B_XPOS)/2;
    public static final double REEFFACE_1_YPOS = (STALK_A_YPOS+STALK_B_YPOS)/2;
    public static final double REEFFACE_1_DEG = -90;
    
    //ID BLUE 17 / RED 8
    public static final double REEFFACE_2_XPOS = (STALK_C_XPOS+STALK_D_XPOS)/2;
    public static final double REEFFACE_2_YPOS = (STALK_C_YPOS+STALK_D_YPOS)/2;
    public static final double REEFFACE_2_DEG = -30;

    //ID BLUE 22 / RED 9
    public static final double REEFFACE_3_XPOS = (STALK_E_XPOS+STALK_F_XPOS)/2;
    public static final double REEFFACE_3_YPOS = (STALK_E_YPOS+STALK_F_YPOS)/2;
    public static final double REEFFACE_3_DEG = 30;

    //ID BLUE 21 / RED 10
    public static final double REEFFACE_4_XPOS = (STALK_G_XPOS+STALK_H_XPOS)/2;
    public static final double REEFFACE_4_YPOS = (STALK_G_YPOS+STALK_H_YPOS)/2;
    public static final double REEFFACE_4_DEG = 90;

    //ID BLUE 20 / RED 11
    public static final double REEFFACE_5_XPOS = (STALK_I_XPOS+STALK_J_XPOS)/2;
    public static final double REEFFACE_5_YPOS = (STALK_I_YPOS+STALK_J_YPOS)/2;
    public static final double REEFFACE_5_DEG = 150;

    //ID BLUE 19 / RED 6
    public static final double REEFFACE_6_XPOS = (STALK_K_XPOS+STALK_L_XPOS)/2;
    public static final double REEFFACE_6_YPOS = (STALK_K_YPOS+STALK_L_YPOS)/2;
    public static final double REEFFACE_6_DEG = -150;

    //POSE FOR STALK, HUMANFEED, AND DE-ALGAE

    //STALKS A-L
    public static final Pose2d STALK_A_POSE = 
        new Pose2d(new Translation2d(STALK_A_XPOS, STALK_A_YPOS), new Rotation2d(Math.toRadians(STALK_A_DEG))); 
        
    public static final Pose2d STALK_B_POSE = 
        new Pose2d(new Translation2d(STALK_B_XPOS, STALK_B_YPOS), new Rotation2d(Math.toRadians(STALK_B_DEG)));

    public static final Pose2d STALK_C_POSE = 
        new Pose2d(new Translation2d(STALK_C_XPOS, STALK_C_YPOS), new Rotation2d(Math.toRadians(STALK_C_DEG))); 
        
    public static final Pose2d STALK_D_POSE = 
        new Pose2d(new Translation2d(STALK_D_XPOS, STALK_D_YPOS), new Rotation2d(Math.toRadians(STALK_D_DEG))); 

    public static final Pose2d STALK_E_POSE = 
        new Pose2d(new Translation2d(STALK_E_XPOS, STALK_E_YPOS), new Rotation2d(Math.toRadians(STALK_E_DEG))); 

    public static final Pose2d STALK_F_POSE = 
        new Pose2d(new Translation2d(STALK_F_XPOS, STALK_F_YPOS), new Rotation2d(Math.toRadians(STALK_F_DEG))); 

    public static final Pose2d STALK_G_POSE = 
        new Pose2d(new Translation2d(STALK_G_XPOS, STALK_G_YPOS), new Rotation2d(Math.toRadians(STALK_G_DEG))); 

    public static final Pose2d STALK_H_POSE = 
        new Pose2d(new Translation2d(STALK_H_XPOS, STALK_H_YPOS), new Rotation2d(Math.toRadians(STALK_H_DEG))); 

    public static final Pose2d STALK_I_POSE = 
        new Pose2d(new Translation2d(STALK_I_XPOS, STALK_I_YPOS), new Rotation2d(Math.toRadians(STALK_I_DEG))); 

    public static final Pose2d STALK_J_POSE = 
        new Pose2d(new Translation2d(STALK_J_XPOS, STALK_J_YPOS), new Rotation2d(Math.toRadians(STALK_J_DEG))); 

    public static final Pose2d STALK_K_POSE = 
        new Pose2d(new Translation2d(STALK_K_XPOS, STALK_K_YPOS), new Rotation2d(Math.toRadians(STALK_K_DEG))); 

    public static final Pose2d STALK_L_POSE = 
        new Pose2d(new Translation2d(STALK_L_XPOS, STALK_L_YPOS), new Rotation2d(Math.toRadians(STALK_L_DEG))); 


    //HUMANFEED LEFT AND RIGHT
    public static final Pose2d LEFT_HF_POSE = 
        new Pose2d(new Translation2d(LEFT_HF_XPOS, LEFT_HF_YPOS), new Rotation2d(Math.toRadians(LEFT_HF_DEG))); 

    public static final Pose2d RIGHT_HF_POSE = 
        new Pose2d(new Translation2d(RIGHT_HF_XPOS, RIGHT_HF_YPOS), new Rotation2d(Math.toRadians(RIGHT_HF_DEG))); 


    //CLIMBS LEFT AND RIGHT 
    public static final Pose2d LEFTSIDE_LEFTCLIMB_POSE =
        new Pose2d(new Translation2d(LEFTSIDE_LEFTCLIMB_XPOS, LEFTSIDE_LEFTCLIMB_YPOS), new Rotation2d(Math.toRadians(LEFTSIDE_LEFTCLIMB_DEG)));

    public static final Pose2d LEFTSIDE_MIDDLECLIMB_POSE = 
        new Pose2d(new Translation2d(LEFTSIDE_MIDDLECLIMB_XPOS, LEFTSIDE_MIDDLECLIMB_YPOS), new Rotation2d(Math.toRadians(LEFTSIDE_MIDDLECLIMB_DEG)));

    public static final Pose2d LEFTSIDE_RIGHTCLIMB_POSE =
        new Pose2d(new Translation2d(LEFTSIDE_RIGHTCLIMB_XPOS, LEFTSIDE_RIGHTCLIMB_YPOS), new Rotation2d(Math.toRadians(LEFTSIDE_RIGHTCLIMB_DEG)));

    public static final Pose2d RIGHTSIDE_LEFTCLIMB_POSE = 
        new Pose2d(new Translation2d(RIGHTSIDE_LEFTCLIMB_XPOS, RIGHTSIDE_LEFTCLIMB_YPOS), new Rotation2d(Math.toRadians(RIGHTSIDE_LEFTCLIMB_DEG)));

    public static final Pose2d RIGHTSIDE_MIDDLECLIMB_POSE = 
        new Pose2d(new Translation2d(RIGHTSIDE_MIDDLECLIMB_XPOS, RIGHTSIDE_MIDDLECLIMB_YPOS), new Rotation2d(Math.toRadians(RIGHTSIDE_MIDDLECLIMB_DEG)));

    public static final Pose2d RIGHTSIDE_RIGHTCLIMB_POSE = 
        new Pose2d(new Translation2d(RIGHTSIDE_RIGHTCLIMB_XPOS, RIGHTSIDE_RIGHTCLIMB_YPOS), new Rotation2d(Math.toRadians(RIGHTSIDE_RIGHTCLIMB_DEG)));

        
    //DE-ALGAE FACES 1-6
    public static final Pose2d REEFFACE_17_POSE = 
        new Pose2d(new Translation2d(REEFFACE_2_XPOS, REEFFACE_2_YPOS), new Rotation2d(Math.toRadians(REEFFACE_2_DEG))); 

    public static final Pose2d REEFFACE_18_POSE = 
        new Pose2d(new Translation2d(REEFFACE_1_XPOS, REEFFACE_1_YPOS), new Rotation2d(Math.toRadians(REEFFACE_1_DEG))); 
    
   public static final Pose2d REEFFACE_19_POSE = 
        new Pose2d(new Translation2d(REEFFACE_6_XPOS, REEFFACE_6_YPOS), new Rotation2d(Math.toRadians(REEFFACE_6_DEG))); 

    public static final Pose2d REEFFACE_22_POSE = 
        new Pose2d(new Translation2d(REEFFACE_3_XPOS, REEFFACE_3_YPOS), new Rotation2d(Math.toRadians(REEFFACE_3_DEG))); 

    public static final Pose2d REEFFACE_21_POSE = 
        new Pose2d(new Translation2d(REEFFACE_4_XPOS, REEFFACE_4_YPOS), new Rotation2d(Math.toRadians(REEFFACE_4_DEG))); 

    public static final Pose2d REEFFACE_20_POSE = 
        new Pose2d(new Translation2d(REEFFACE_5_XPOS, REEFFACE_5_YPOS), new Rotation2d(Math.toRadians(REEFFACE_5_DEG))); 

}    