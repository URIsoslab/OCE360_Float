/*
Author Mingxi Zhou
OCE360 underwater float template program
*/
#include "mbed.h"
#include "LSM9DS1.h"  //IMU library
#include "MS5837.h"     //pressure sensor library
#include "SCI_SENSOR.h"     //science sensor
#include "SDFileSystem.h"   // SD card

DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);    //initial serial
Serial BLE(p13,p14);        //Bluetooth
LSM9DS1 IMU(p28, p27, 0xD6, 0x3C);  //initial IMU
LM19 temp(p19);
PhotoCell light(p20);
MS5837 p_sensor(p9, p10, ms5837_addr_no_CS);  //pressure sensor
PwmOut thruster(p21);  //set PWM pin    //max 1.3ms min 1.1ms
PwmOut thruster2(p22); //set PWM pin
SDFileSystem sd(p5, p6, p7, p8, "sd"); // the pinout on the mbed Cool Components workshop board

//global ticker
Ticker log_ticker;
Ticker imu_ticker;
// global timer
Timer t;
//MS5837 p_sensor(p9, p10, ms5837_addr_no_CS);
///File
FILE *fp;
char fname[100];
float PI = 3.14159265358979323846f;

//float operation parameters
float target_depth=0;   //global target depth default 0
int yo_num=0;           //global yo_num default 0
float thrust_on_time=0; //global thrust_on time default 0
float accel[3], mag[3], gyro[3], euler[3];  //global IMU data

//functions
void welcome();
void log_data();
//IMU related
void IMU_update(); //update IMU related varibles. we use imu_ticker to call this function
void mag_correction(float mx, float my, float mz, float mag_c[3]); //raw mag -> mag[3], mag_c[3] calibrated
void pose_estimate(float euler[3], float accel[3], float gyro[3], float mag[3]);  //pose estimation function
//Control related functions
void thrust_on(float pw, float on_time);  //input is pulse width

//-------------Main functions-----------------------------------------------------------------------------------------
int main()
{
//-----Initialization realted code-------//
   //inital set the thruster esc to 1ms duty cycle
    thruster.period(0.002);      // 2 ms period
    thruster.pulsewidth(1.0/1000.000);    /////IMU initial and begin
    thruster2.period(0.002);      // 2 ms period
    thruster2.pulsewidth(1.0/1000.000);    /////IMU initial and begin
    IMU.begin();
    IMU.calibrate(true);
    myled=1;
    //initialize pressure sensor
    pc.printf("setting the pressure sensor\r\n");
    p_sensor.MS5837Reset();
    p_sensor.MS5837Init();
    pc.printf("settting the tickers\r\n");
    t.start();
    myled=0;
    welcome();
    //-----setup ticker-------//
    //setup ticker to separate log and IMU data update.
    //so we could have all our control code in the while loop
    //   //log at 2 Hz
    imu_ticker.attach(&IMU_update,0.1);  //10Hz
    log_ticker.attach(&log_data,0.5);
    wait(1);
    while(1)
    {
      // put your main control code here
    }

}

//-------------Customized functions---------------------------------------------//----------------------------------------
///-----------Welcome menu---------------------///
void welcome()
{
    char buffer[100]={0};
    int flag=1;
    //Flush the port
    while(BLE.readable())
    {
    BLE.getc();
    }
    while(flag)
    {
        BLE.printf("### I am alive\r\n");
        BLE.printf("### Please enter the log file name you want\r\n");
        if(BLE.readable())
        {
            BLE.scanf("%s",buffer);
            sprintf(fname,"/sd/mydir/%s.txt",buffer); //make file name

            flag = 0; //set the flag to 0 to break the while
        }
        myled=!myled;
        wait(1);
    }
    //print name
    BLE.printf("### name received\r\n");
    BLE.printf("### file name and directory is: \r\n %s\r\n",fname); //file name and location
    //open file test
    mkdir("/sd/mydir",0777); //keep 0777, this is magic #
    fp = fopen(fname, "a");
    if(fp == NULL){
        BLE.printf("Could not open file for write\n");
    }
    else
    {
        BLE.printf("##file open good \n"); //open file and tell if open
        fprintf(fp, "Hello\r\n");
        fclose(fp);
    }

    BLE.printf("### The main program will start in 10 seconds\r\n");
    wait(5);
}

///-----------log functions---------------------///
void log_data()
{
    //log system time t.read()
    // log imu data, log sciene data
    // log pulse width
    // log pressure sensor data.
    //science sensor: temp.temp(), light.light()
    //IMU sensor

}

///-----------IMU related functions---------------------///
void IMU_update()
{
    IMU.readMag();
    IMU.readGyro();
    IMU.readAccel();
    accel[0] = IMU.calcAccel(IMU.ax);
    accel[1] = IMU.calcAccel(IMU.ay);
    accel[2] = -IMU.calcAccel(IMU.az);
    gyro[0]  = IMU.calcGyro(IMU.gx);
    gyro[1]  = IMU.calcGyro(IMU.gy);
    gyro[2]  = -IMU.calcGyro(IMU.gz);
    mag_correction(IMU.calcMag(IMU.mx), IMU.calcMag(IMU.my), IMU.calcMag(IMU.mz), mag);  //mag correction
    mag[2]   = - mag[2];
    pose_estimate(euler, accel, gyro, mag);  //pose update
}

void mag_correction(float mx, float my, float mz, float mag_c[3])
{
    float bias[3] = {0.0793,0.0357,0.2333};
    float scale[3][3] = {{1.0070, 0.0705, 0.0368},
                         {0.0705, 1.0807, 0.0265},
                         {0.0368, 0.0265, 0.9250}};
    //mag_c = (mag-bias)*scale

    mag_c[0] = (mx - bias[0]) *scale[0][0] + (my - bias[1]) *scale[1][0] + (mz - bias[2]) *scale[2][0];
    mag_c[1] = (mx - bias[0]) *scale[0][1] + (my - bias[1]) *scale[1][1] + (mz - bias[2]) *scale[2][1];
    mag_c[2] = (mx - bias[0]) *scale[0][2] + (my - bias[1]) *scale[1][2] + (mz - bias[2]) *scale[2][2];
}

void pose_estimate(float euler[3], float accel[3], float gyro[3], float mag[3])  //pose estimation function
{
        euler[0] =  atan2 (accel[1] , accel[2]/abs(accel[2])*(sqrt ((accel[0] * accel[0]) + (accel[2] * accel[2]))));
        euler[1] = - atan2( -accel[0] ,( sqrt((accel[1] * accel[1]) + (accel[2] * accel[2]))));
        float Yh = (mag[1] * cos(euler[0])) - (mag[2] * sin(euler[0]));
        float Xh = (mag[0] * cos(euler[1]))+(mag[1] * sin(euler[0])*sin(euler[1]))
                    + (mag[2] * cos(euler[0]) * sin(euler[1]));
        euler[2] = atan2(Yh, Xh);
        //convert into degrees
        euler[0] *= 180.0f / PI;
        euler[1] *= 180.0f / PI;
        euler[2] *= 180.0f / PI;
        //wrap the values to be within 0 to 360.
        for (int i=0;i<3;i++)
        {
            if(euler[i]<=0)
            {
                euler[i]=euler[i]+360;
            }
            if(euler[i]>360)
            {
                euler[i]=euler[i]-360;
            }
        }

}

///-----------Control related functions---------------------///
////Thruster on control, pw->pulse width in milli-second//
////                        pw range between 1 to 1.5//
////                       on_time-> thruster on time.
void thrust_on(float pw, float on_time)  //input is pulse width
{
    float pw_max=2.0;
    if(pw>pw_max)
    {
        pw=pw_max; //hard limitation
    }
    Timer tt;
    tt.reset();
    tt.start();
    // lets set the pulse width
    //thruster.period(20.0/1000.00);      // 20 ms period
    thruster.pulsewidth(pw/1000.00);
    thruster2.pulsewidth(pw/1000.00);
    //PWM will be kept until time out
    while(tt.read()<=on_time)
    {
    }
    //stop the timer
    tt.stop();
    //turn off the thruster
    thruster.pulsewidth(1.0/1000.00);
    thruster2.pulsewidth(1.0/1000.00);

}
