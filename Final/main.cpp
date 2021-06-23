#include "mbed.h"
#include "bbcar.h"
#include "bbcar_rpc.h"
#include <cmath>
//#include <mutex>
//#include <iostream>
//#include <thread>
using namespace std;
//std::mutex mtx;

DigitalOut led1(LED2);
Ticker servo_ticker;
PwmOut pin5(D5), pin6(D6);
BufferedSerial xbee(D1, D0);

BBCar car(pin5, pin6, servo_ticker);

//ping
BufferedSerial pc(USBTX, USBRX);
DigitalInOut ping(D10);
Thread t_ping, t1, t2;
void ping_function();
Timer t;

int is_stop = 1; 
int is_stop2 = 0;
int bool_value = 1;
int number_tag = 0;

parallax_ping  ping1(ping);

void line_detect(Arguments *in, Reply *out);
RPCFunction line(&line_detect, "line");

void get_value(Arguments *in, Reply *out);
RPCFunction getvalue(&get_value, "value");
float Tx;
float Tz;
float Ry;

void calibration(Arguments *in, Reply *out);
RPCFunction cal(&calibration, "calibration");

void circling();
void Apriltag();

int main() {
   char buf[256], outbuf[256];
   FILE *devin = fdopen(&xbee, "r");
   FILE *devout = fdopen(&xbee, "w");

   pc.set_baud(9600);
   xbee.set_baud(9600);
   t_ping.start(ping_function);
   while (1) {
      memset(buf, 0, 256);
      for( int i = 0; ; i++ ) {
         char recv = fgetc(devin);
         if(recv == '\n') {
            printf("\r\n");
            break;
         }
         buf[i] = fputc(recv, devout);
      }
   RPC::call(buf, outbuf);
   }
}

void Apriltag()
{
   char buf[256], outbuf[256];
   FILE *devin = fdopen(&xbee, "r");
   FILE *devout = fdopen(&xbee, "w");

   pc.set_baud(9600);
   xbee.set_baud(9600);

      memset(buf, 0, 256);
      for( int i = 0; ; i++ ) {
         char recv = fgetc(devin);
         if(recv == '\n') {
            printf("\r\n");
            break;
         }
         buf[i] = fputc(recv, devout);
      }
      RPC::call(buf, outbuf);
}

void ping_function()
{
   //parallax_ping  ping1(ping);
   while(1) {
      //std::lock_guard<std::mutex> mtx_locker(mtx);
      //printf("%f\r\n",(float)ping1);
      if((float)ping1>20) led1 = 1;
      else {
         led1 = 0;
         if(is_stop2==1) is_stop = 0;
         //car.stop();
         //break;
      }
      ThisThread::sleep_for(10ms);
   }
}

void line_detect(Arguments *in, Reply *out)
{
   float xx1 = in->getArg<float>();
   float yy1 = in->getArg<float>();
   float xx2 = in->getArg<float>();
   float yy2 = in->getArg<float>();
   
   printf("%f, %f, %f, %f\r\n",xx1,yy1,xx2,yy2);
   float angle;
   int spin_time;
   angle = atan(abs((yy1-yy2)/(xx1-xx2)))*180/M_PI;
   printf("%f degree\r\n",angle);
   angle = 90-angle;
   spin_time = int(2500*angle/180);
   
   char line_buffer[40] = {0};
   sprintf(line_buffer, "start line detection\r\n");
   xbee.write(line_buffer, sizeof(line_buffer));
   //t_ping.start(ping_function);
   //car.goStraight(-100);
   //ThisThread::sleep_for(std::chrono::milliseconds(200));
   if(((xx1-xx2)*(yy1-yy2)) < 0)
   {
      car.turn(-100, 0.15);
      ThisThread::sleep_for(std::chrono::milliseconds(spin_time));
      car.stop();
      ThisThread::sleep_for(std::chrono::milliseconds(500));
      //is_stop2 = 1;
      /*while(is_stop)
      {
        car.goStraight(-100);
        ThisThread::sleep_for(std::chrono::milliseconds(200));
      }
      car.stop();*/
   }
   else if(((xx1-xx2)*(yy1-yy2)) > 0)
   {
      car.turn(-100, -0.15);
      ThisThread::sleep_for(std::chrono::milliseconds(spin_time));
      car.stop();
      ThisThread::sleep_for(std::chrono::milliseconds(500));
      //is_stop2 = 1;
      /*while(is_stop)
      {
        car.goStraight(-100);
        ThisThread::sleep_for(std::chrono::milliseconds(200));
      }
      car.stop();*/
   }
   char line_buffer2[40] = {0};
   sprintf(line_buffer2, "end line detection\r\n");
   xbee.write(line_buffer2, sizeof(line_buffer2));
   circling();
}

void circling()
{
   char circle_buffer[40] = {0};
   sprintf(circle_buffer, "start circing\r\n");
   xbee.write(circle_buffer, sizeof(circle_buffer));
   float ping2;
   ping2 = float(ping1);
   int number1 = 0;
/*   while((ping2 <= 40)&&(number1<=6))
   {
       //std::lock_guard<std::mutex> mtx_locker(mtx);
       car.goStraight(100);
       ThisThread::sleep_for(std::chrono::milliseconds(1200)); //reverse 20cm
       car.stop();
       car.turn(-100, 0.15);
       ThisThread::sleep_for(std::chrono::milliseconds(200)); //turn right 20 degree
       car.stop();
       ThisThread::sleep_for(std::chrono::milliseconds(500));
       car.goStraight(-100);
       ThisThread::sleep_for(std::chrono::milliseconds(1400)); //forward 20cm
       ping2 = float(ping1);
       ThisThread::sleep_for(std::chrono::milliseconds(500));
       number1++;
   }


   car.goStraight(-100);
   ThisThread::sleep_for(std::chrono::milliseconds(1200)); //forward 20cm
   car.stop();
   car.turn(-100, -0.15);
   ThisThread::sleep_for(std::chrono::milliseconds(1240)); //turn left 90 degree
   car.stop();
   ThisThread::sleep_for(std::chrono::milliseconds(500));

   /*while(float(ping1) > 40)
   {
   car.goStraight(-100);
   ThisThread::sleep_for(std::chrono::milliseconds(1200)); //forward 20cm
   car.stop();
   }
   ping2 = float(ping1);
   if(ping2 > 40)
   {
        //std::lock_guard<std::mutex> mtx_locker(mtx);
        car.turn(-100, -0.15);
        ThisThread::sleep_for(std::chrono::milliseconds(620)); //turn left 45 degree
        car.stop();
        ThisThread::sleep_for(std::chrono::milliseconds(500));
   }

   ping2 = float(ping1);
   while(ping2 <= 40)
   {
       //std::lock_guard<std::mutex> mtx_locker(mtx);
       car.goStraight(100);
       ThisThread::sleep_for(std::chrono::milliseconds(1200)); //reverse 20cm
       car.stop();
       car.turn(-100, 0.15);
       ThisThread::sleep_for(std::chrono::milliseconds(200)); //turn right 15 degree
       car.stop();
       ThisThread::sleep_for(std::chrono::milliseconds(500));
       car.goStraight(-100);
       ThisThread::sleep_for(std::chrono::milliseconds(1400)); //forward 20cm
       ping2 = float(ping1);
   }

   /*car.goStraight(-100);
   ThisThread::sleep_for(std::chrono::milliseconds(1200)); //forward 20cm
   car.stop();
   car.turn(-100, -0.15);
   ThisThread::sleep_for(std::chrono::milliseconds(1240)); //turn left 90 degree
   car.stop();
   ThisThread::sleep_for(std::chrono::milliseconds(500));*/
   
   //ping2 = float(ping1);
   /*while(ping2 > 40)
   {
       //std::lock_guard<std::mutex> mtx_locker(mtx);
   car.goStraight(-100);
   ThisThread::sleep_for(std::chrono::milliseconds(600)); //forward 10cm
   car.stop();
   car.turn(-100, -0.15);
   ThisThread::sleep_for(std::chrono::milliseconds(620)); //turn left 45 degree
   car.stop();
   ping2 = float(ping1);
   if(ping2 <= 40) break;
   else 
   {
        car.turn(100, -0.15);
        ThisThread::sleep_for(std::chrono::milliseconds(620)); //reverse turn left 45 degree
        car.stop();
   }
   }
   car.goStraight(-100);
   ThisThread::sleep_for(std::chrono::milliseconds(1200)); //forward 10cm
   car.stop();
   car.turn(-100, -0.15);
   ThisThread::sleep_for(std::chrono::milliseconds(1240)); //turn left 45 degree
   car.stop();

   ping2 = float(ping1);
   while(ping2 <= 40)
   {
       //std::lock_guard<std::mutex> mtx_locker(mtx);
       car.goStraight(100);
       ThisThread::sleep_for(std::chrono::milliseconds(1200)); //reverse 20cm
       car.stop();
       car.turn(-100, 0.15);
       ThisThread::sleep_for(std::chrono::milliseconds(200)); //turn right 15 degree
       car.stop();
       ThisThread::sleep_for(std::chrono::milliseconds(500));
       car.goStraight(-100);
       ThisThread::sleep_for(std::chrono::milliseconds(1400)); //forward 20cm
       ping2 = float(ping1);
   }
   car.goStraight(-100);
   ThisThread::sleep_for(std::chrono::milliseconds(1200)); //forward 20cm
   car.stop();
   car.turn(-100, -0.15);
   ThisThread::sleep_for(std::chrono::milliseconds(1240)); //turn left 90 degree
   car.stop();
   ThisThread::sleep_for(std::chrono::milliseconds(500));
 */
   int sleep_time;
   if(ping2>30) sleep_time = (ping2-30)*60;
   else sleep_time = 1200;
   car.goStraight(-100);
   ThisThread::sleep_for(std::chrono::milliseconds(sleep_time));
   car.stop();
   car.turn(-100, 0.15);
   ThisThread::sleep_for(std::chrono::milliseconds(1350));
   car.stop();
   ThisThread::sleep_for(std::chrono::milliseconds(1000));

   ping2 = float(ping1);
   car.turn(-100, -0.45);
   ThisThread::sleep_for(std::chrono::milliseconds(1000));
   ping2 = 30;
   while(led1 == 1)
   {
   ThisThread::sleep_for(std::chrono::milliseconds(300));
   ping2 = float(ping1); 
   }
   car.stop();


   char circle_buffer2[40] = {0};
   sprintf(circle_buffer2, "end circing\r\n");
   xbee.write(circle_buffer2, sizeof(circle_buffer2));
   ThisThread::sleep_for(std::chrono::milliseconds(500));

   char circle_buffer3[40] = {0};
   char outbuf3[256];
   while(number_tag<=3) 
   {
      Apriltag();
      ThisThread::sleep_for(std::chrono::milliseconds(500));
   }
   sprintf(circle_buffer3, "/calibration/run %f %f %f\r\n",Tx,Tz,Ry);
   RPC::call(circle_buffer3, outbuf3);
}

void get_value(Arguments *in, Reply *out)
{
    Tx = in->getArg<float>();
    Tz = in->getArg<float>();
    Ry = in->getArg<float>();
    if((int(Tz)<0)) 
    {
       number_tag++;
    }
}

void calibration(Arguments *in, Reply *out)
{
        Tz = abs(Tz)*6.2;
        Tx = Tx*6.2;
        char cal_buffer[40] = {0};
        sprintf(cal_buffer, "Tx = %f, Tz = %f, Ry = %f\r\n",Tx,Tz,Ry);
        xbee.write(cal_buffer, sizeof(cal_buffer));
        ThisThread::sleep_for(std::chrono::milliseconds(500));

        char cal_buffer2[40] = {0};
        sprintf(cal_buffer2, "start calibration\r\n");
        xbee.write(cal_buffer2, sizeof(cal_buffer2));
        ThisThread::sleep_for(std::chrono::milliseconds(500));
        
        float angle;
        int reverse_time;
        int forward_time;
        if(Ry>0 && Ry<90) reverse_time = int(abs(Tx)*60/cos(Ry/180*M_PI));
        else if(Ry<360 && Ry>270) reverse_time = int(abs(Tx)*60/cos((360-Ry)/180*M_PI));
        //steer the car

        if((Ry<=1)||(Ry>=359))
        {
            printf("ping: %f\r\n",float(ping1));
        }
        else
        {
        //if(((Tz-30) <= abs(Tx)) || float(ping1)<30)
        if(Tz>100)
        {
            /*if(Ry>0 && Ry<90)
            {
                car.turn(100,-0.15);
                ThisThread::sleep_for(std::chrono::milliseconds(1240));
                car.stop();
                ThisThread::sleep_for(std::chrono::milliseconds(500));
                car.goStraight(100);
                ThisThread::sleep_for(std::chrono::milliseconds(reverse_time));
                car.stop();
            }
            else if(Ry<360 && Ry>270)
            {
                car.turn(100,0.15);
                ThisThread::sleep_for(std::chrono::milliseconds(1240));
                car.stop();
                ThisThread::sleep_for(std::chrono::milliseconds(500));
                car.goStraight(100);
                ThisThread::sleep_for(std::chrono::milliseconds(reverse_time));
                car.stop();
            }*/
        }
        else //forwrad
        {
            if(Ry>0 && Ry<90) angle = Ry/2*1.3;
            else if(Ry<360 && Ry>270) angle = (360-Ry)/2*1.3;
            printf("rotation angle: %f degree\r\n",angle);
            forward_time = int(2500*angle/90); //? /180
            int time_res = forward_time + 500;
            int time_res2 = 1700 - time_res;
            int forward_time2;
            int forward_time3;
            if(Ry>0 && Ry<90)
            {
                //judge1 = judge2;
                //judge2 = 0;
                printf("left\r\n");
                if(Ry<5) forward_time2 = 6*60*tan(Ry/180*M_PI)*Tz;
                else if(Ry<9) forward_time2 = 4*60*tan(Ry/180*M_PI)*Tz;
                else forward_time2 = 2*60*tan(Ry/180*M_PI)*Tz;
                if(Ry<9) forward_time3 = 3*forward_time;
                else forward_time3 = 2*forward_time;
                //printf("%d %d\r\n",is_complete,is_complete2);
                //if(Ry < 5)
                //if(judge1 !=judge2)
                //{
                car.turn(-100,0.15);
                ThisThread::sleep_for(std::chrono::milliseconds(forward_time));
                car.stop();
                car.goStraight(-100);
                ThisThread::sleep_for(std::chrono::milliseconds(forward_time2));
                car.stop();
                ThisThread::sleep_for(std::chrono::milliseconds(500));
                car.turn(-100,-0.15);
                ThisThread::sleep_for(std::chrono::milliseconds(forward_time3));
                car.stop();
                //}
                //else 
                //{
                    //int time_res = forward_time + 500;
                    //ThisThread::sleep_for(std::chrono::milliseconds(time_res));
                //}
                //car.goStraight(-100);
                //ThisThread::sleep_for(std::chrono::milliseconds(300));
                //car.stop();
                //ThisThread::sleep_for(std::chrono::milliseconds(time_res2));
            }
            else if(Ry<360 && Ry>270)
            {
                //judge1 = judge2;
                //judge2 = 1;
                printf("right\r\n");
                if(Ry>355) forward_time2 = 6*60*tan((360-Ry)/180*M_PI)*Tz;
                else if(Ry>351) forward_time2 = 4*60*tan((360-Ry)/180*M_PI)*Tz;
                else forward_time2 = 2*60*tan((360-Ry)/180*M_PI)*Tz;
                if(Ry>351) forward_time3 = 3*forward_time;
                else forward_time3 = 2*forward_time; 
                //printf("%d %d\r\n",is_complete,is_complete2);
                //if(judge1 !=judge2)
                //if( Ry>355)
                //{
                //car.turn(-100,0.15);
                //ThisThread::sleep_for(std::chrono::milliseconds(forward_time));
                //car.stop();
                //ThisThread::sleep_for(std::chrono::milliseconds(500));
                //}
                car.turn(-100,-0.15);
                ThisThread::sleep_for(std::chrono::milliseconds(forward_time));
                car.stop();
                car.goStraight(-100);
                ThisThread::sleep_for(std::chrono::milliseconds(forward_time2));
                car.stop();
                ThisThread::sleep_for(std::chrono::milliseconds(500));
                car.turn(-100,0.15);
                ThisThread::sleep_for(std::chrono::milliseconds(forward_time3));
                car.stop();
                //else
                //{
                //    ThisThread::sleep_for(std::chrono::milliseconds(time_res));
                //}
                //car.goStraight(-100);
                //ThisThread::sleep_for(std::chrono::milliseconds(300));
                //car.stop();
                //ThisThread::sleep_for(std::chrono::milliseconds(time_res2));
            } 
        }
        }
        //printf("ping: %f\r\n",float(ping1));
        char cal_buffer3[40] = {0};
        sprintf(cal_buffer3, "end calibration\r\n");
        xbee.write(cal_buffer3, sizeof(cal_buffer3));
        ThisThread::sleep_for(std::chrono::milliseconds(500));

        char cal_buffer4[40] = {0};
        sprintf(cal_buffer4, "start going destnation\r\n");
        xbee.write(cal_buffer4, sizeof(cal_buffer4));
        float ping3 = float(ping1)-40;
        int ping_time = int(60*ping3);
        car.goStraight(-100);
        ThisThread::sleep_for(std::chrono::milliseconds(ping_time));
        car.stop();
        ThisThread::sleep_for(std::chrono::milliseconds(500));

        char cal_buffer5[40] = {0};
        sprintf(cal_buffer5, "end going destnation\r\n");
        xbee.write(cal_buffer5, sizeof(cal_buffer5));
}