#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

#define VERTICAL_ODOM 13

#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

#define LEFT_FRONT_DRIVE -10
#define LEFT_MIDDLE_DRIVE -7
#define LEFT_BACK_DRIVE -12

#define RIGHT_FRONT_DRIVE 13
#define RIGHT_MIDDLE_DRIVE 14
#define RIGHT_BACK_DRIVE 15

#define VERTICAL_ODOM 20
#define IMU 20

#define IntakeBottomRoller 9
#define IntakeTopRoller 1
#define IntakeLastWheel -6

#define LIFT 'H'
#define SCRAPE 'A' //change this
#define TOUNGUE 'G'

inline pros::MotorGroup left_mg({LEFT_FRONT_DRIVE, LEFT_MIDDLE_DRIVE, LEFT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_DRIVE, RIGHT_MIDDLE_DRIVE, RIGHT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup Intake_mg({IntakeBottomRoller, IntakeTopRoller, IntakeLastWheel});
inline pros::MotorGroup Intake_eject_mg({IntakeBottomRoller, IntakeTopRoller, -IntakeLastWheel});
inline pros::MotorGroup Intake_index_mg({IntakeBottomRoller, IntakeTopRoller});
inline pros::MotorGroup LastWheel({IntakeLastWheel});

inline pros::Rotation vertical_odom(VERTICAL_ODOM);

//pneumatics

inline pros::adi::Pneumatics lift(LIFT, false);
inline pros::adi::Pneumatics scrape(SCRAPE, false);
inline pros::adi::Pneumatics Toungue(TOUNGUE, false);