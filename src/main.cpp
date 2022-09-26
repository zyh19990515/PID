#include <Servo.h> 
#include <SSD1306.h>
#include <DATASCOPE.h>      //这是PC端上位机的库文件
#include <MsTimer2.h>        //定时中断
#include <PS2X_lib.h>  
#define ZERO_X 133  //X轴零点
#define ZERO_Y 100  //Y轴零点
//////////PS2引脚//////////////////
#define PS2_DAT        7  //14    
#define PS2_CMD        6  //15
#define PS2_SEL        5  //16
#define PS2_CLK        4  //17
//////////触摸屏控制引脚//////////////////
#define YL        2    
#define YH        15  
#define XL        3  
#define XH        14  
////////OLED显示屏引脚///////////
#define OLED_DC 8   //DC
#define OLED_CLK 13  //D0
#define OLED_MOSI 12  //D1
#define OLED_RESET 11  //RES
#define KEY 19  //按键引脚
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0); //OLED初始化代码
DATASCOPE data;//实例化一个 上位机 对象，对象名称为 data
Servo myservo1,myservo2;  //创建2个舵机控制对象
PS2X ps2x; // 创建1个PS2控制对象
int Flag_Stop=1,Flag_Show,Flag_Move;  //相关标志位
float Zero_X=133,Zero_Y=100,Target_X,Target_Y;  //X Y方向的目标值和控制量
int Position_X,Position_Y; //X Y方向的测量值
float PS2_KEY;   //PS2按键变量
float Balance_Kp=53,Balance_Kd=58;//PID参数
int PS2_LX=128,PS2_LY=128,PS2_RX=128,PS2_RY=128;     //PS2遥控相关
int error = 0;   //PS2使用的一个变量 识别PS2是否插入
void (* resetFunc) (void) = 0;// Reset func 
/**************************************************************************
函数功能：特定轨迹运动
入口参数：无
返回  值：无
**************************************************************************/
void Setting_Move(void)
{
     static float count;  //计数变量
     count++;  //自加
    if(Flag_Move==1)  //控制小球沿着三角形的轨迹运动
    {
             if(count<40) Zero_Y++;   
        else if(count<80) Zero_Y-=2,Zero_X-=1.5;
        else if(count<120)Zero_X+=3;
        else if(count<160)Zero_Y+=1,Zero_X-=1.5,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
    else if(Flag_Move==2)   //控制小球沿着球的轨迹运动
    {        
             if(count<40) Zero_Y++;
        else if(count<40+PI*40)Zero_Y=ZERO_Y+40*cos((count-40)/20),Zero_X=ZERO_X+40*sin((count-40)/20);
        else if(count<210)Zero_Y--,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
    else  if(Flag_Move==4)   //控制小球沿着叉(X)的轨迹运动
    {
             if(count<40) Zero_Y++,Zero_X--;
        else if(count<120)Zero_Y--,Zero_X++;
        else if(count<160)Zero_Y++,Zero_X--;
        else if(count<200)Zero_Y++,Zero_X++;
        else if(count<280)Zero_Y--,Zero_X--;
        else if(count<320)Zero_Y++,Zero_X++,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
    else  if(Flag_Move==8) //控制小球沿着正方形的轨迹运动
    {
             if(count<40) Zero_Y++;
        else if(count<80) Zero_X++;
        else if(count<160)Zero_Y--;
        else if(count<240)Zero_X--;
        else if(count<320)Zero_Y++;
        else if(count<360)Zero_X++;
        else if(count<400)Zero_Y--,Flag_Move=0,count=0;//最后一步，回到原点 计数器清零
    }
}
/**************************************************************************
函数功能：PID参数调节
入口参数：无
返回  值：无
**************************************************************************/
void Adjust(void)
{
    int X_temp,Y_temp,Threshold=100;//阈值
    X_temp=PS2_RX-128;   //X方向偏差临时变量更新
    Y_temp=PS2_RY-128;   //X方向偏差临时变量更新
    if(PS2_KEY==16||PS2_KEY==32||PS2_KEY==64||PS2_KEY==128)//左边的任意4个按键按下才能调节PID参数，防止误触
    {
    if(X_temp>Threshold) Balance_Kp++;   //KP参数增加
    if(X_temp<-Threshold)Balance_Kp--;   //KP参数减小
    if(Y_temp>Threshold) Balance_Kd--;   //KD参数减小
    if(Y_temp<-Threshold)Balance_Kd++;   //KP参数增加
    }
}
/**************************************************************************
函数功能：获取遥控器的参数
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
    float zero_x,zero_y,Step=3;
    zero_x=ZERO_X-(PS2_LX-128)/3;  //摇杆X方向控制球X方向的移动
    zero_y=ZERO_Y+(PS2_LY-128)/4;  //摇杆Y方向控制球Y方向的移动
   if(Zero_X<zero_x) Zero_X+=Step; //X方向调节目标值，也就是球的位置
   if(Zero_X>zero_x) Zero_X-=Step;
   if(Zero_Y<zero_y) Zero_Y+=Step;//Y方向调节目标值，也就是球的位置
   if(Zero_Y>zero_y) Zero_Y-=Step;
}
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{
  int i,Send_Count;
  data.DataScope_Get_Channel_Data(Position_X, 1);//显示第1个数据 X方向的坐标
  data.DataScope_Get_Channel_Data(Position_Y, 2);//显示第2个数据 Y方向的坐标
//  data.DataScope_Get_Channel_Data(0, 3);//显示第3个数据
//  data.DataScope_Get_Channel_Data(0, 4);//显示第4个数据
//  data.DataScope_Get_Channel_Data(0, 5);//显示第5个数据
//  data.DataScope_Get_Channel_Data(0, 6);//显示第6个数据
  Send_Count = data.DataScope_Data_Generate(2);   
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
 delay(50);  //上位机必须严格控制发送时序
}

/**************************************************************************
函数功能：求次方的函数
入口参数：m,n
返回  值：m的n次幂
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
  uint32_t result=1;  
  while(n--)result*=m;    
  return result;
} 
/**************************************************************************
函数功能：显示变量
入口参数：x:x坐标   y:行  num：显示的变量 len ：变量的长度
返回  值：无
**************************************************************************/
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len)
{           
    uint8_t t,temp;  
    uint8_t enshow=0;               
    for(t=0;t<len;t++)  {
    temp=(num/oled_pow(10,len-t-1))%10;
    oled.drawchar(x+6*t,y,temp+'0');
  }  
} 
/**************************************************************************
函数功能：舵机控制程序
入口参数：舵机控制量
返回  值：无
**************************************************************************/
 void Control_servo(float velocity_x,float velocity_y)
{
    myservo1.write(90-velocity_x);        // 指定舵机转向的角度
    myservo2.write(90-velocity_y);        // 指定舵机转向的角度
}
/**************************************************************************
函数功能：PS2手柄接收控制函数
入口参数：无
返回  值：无
**************************************************************************/
void PS2X_RX(void)
{
      ps2x.read_gamepad(false, 0); //read controller and set large motor to spin at 'vibrate' speed
      PS2_KEY=ps2x.ButtonDataByte();  //更新按键变量
      PS2_LX=ps2x.Analog(PSS_LX );    //更新左摇杆X方向的数据
      PS2_LY=ps2x.Analog(PSS_LY );    //更新左摇杆Y方向的数据
      PS2_RX=ps2x.Analog(PSS_RX );    //更新右摇杆X方向的数据
      PS2_RY=ps2x.Analog(PSS_RY );   //更新右摇杆Y方向的数据
}
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 2：双击
**************************************************************************/
uint8_t click_N_Double (uint8_t time)
{
    static  unsigned char flag_key,count_key,double_key; 
    static  unsigned int  count_single,Forever_count;
     if(digitalRead(KEY)==0)  Forever_count++;   //长按标志位未置1
     else        Forever_count=0;
    if(digitalRead(KEY)==0&&0==flag_key)   flag_key=1;  //按键按下
    if(0==count_key)
    {
        if(flag_key==1) 
        {
          double_key++;
          count_key=1;  
        }
        if(double_key==2) 
        {
          double_key=0;
          count_single=0;
          return 2;//双击执行的指令
        }
    }
    if(1==digitalRead(KEY))      flag_key=0,count_key=0;
    
    if(1==double_key)
    {
      count_single++;
      if(count_single>time&&Forever_count<time)
      {
      double_key=0;
      count_single=0; 
      return 1;//单击执行的指令
      }
      if(Forever_count>time)
      {
      double_key=0;
      count_single=0; 
      }
    } 
    return 0;//无操作
}
/**************************************************************************
函数功能：X方向平衡PD控制
入口参数：角度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balanceX(float Angle )
{  
   float  Differential,Bias,Balance_Ki=0.06;//定义差分变量和偏差
   static float Last_Bias,Integration,Balance_Integration,Flag_Target;  //上一次的偏差值
   int balance;//平衡的返回值
   Bias=(Angle-Zero_X);  //===求出平衡的角度中值 和机械相关  
   Differential=Bias-Last_Bias;  //求得偏差的变化率  
  if(++Flag_Target>20) //错频处理积分控制
  {
   Flag_Target=0;
   if(Flag_Stop==0) Integration+=Bias;  // 检测到小球且舵机使能则积分
   else Integration=0;//否则清零
   if(Integration<-200) Integration=-200; //积分限幅
   if(Integration>200)  Integration=200;  
   Balance_Integration=Integration*Balance_Ki;  //积分控制
  }   
   balance=Balance_Kp*Bias/500+Balance_Kd*Differential/50+Balance_Integration;   //===计算平衡控制的舵机PWM  PD控制   kp是P系数 kd是D系数 
   Last_Bias=Bias;  //保存上一次的偏差
   return balance;  //返回值
}
/**************************************************************************
函数功能：Y方向平衡PD控制
入口参数：角度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balanceY(float Angle )
{  
 float  Differential,Bias,Balance_Ki=0.06;//定义差分变量和偏差
   static float Last_Bias,Integration,Balance_Integration,Flag_Target;  //上一次的偏差值
   int balance;//平衡的返回值
   Bias=(Angle-Zero_Y);  //===求出平衡的角度中值 和机械相关  
   Differential=Bias-Last_Bias;  //求得偏差的变化率  
  if(++Flag_Target>20) //错频处理积分控制
  {
   Flag_Target=0;
   if(Flag_Stop==0) Integration+=Bias;  // 检测到小球且舵机使能则积分
   else Integration=0;//否则清零
   if(Integration<-200) Integration=-200; //积分限幅
   if(Integration>200)  Integration=200;  
   Balance_Integration=Integration*Balance_Ki;  //积分控制
  }   
   balance=Balance_Kp*Bias/500+Balance_Kd*Differential/50+Balance_Integration;   //===计算平衡控制的舵机PWM  PD控制   kp是P系数 kd是D系数 
   Last_Bias=Bias;  //保存上一次的偏差
   return balance;  //返回值
}
/**************************************************************************
函数功能：5ms控制函数 核心代码 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control(){  
    char Key;     //按键变量
    static uint8_t Flag_Target,Max_Target=30;
    //sei();        //全局中断开启
    Key=click_N_Double(50);//扫描按键变化  
    if(Key==1)       Flag_Stop=!Flag_Stop; //单击控制舵机的状态
    else if(Key==2)  Flag_Show=!Flag_Show; //单击控制上位机的状态
    if(++Flag_Target>4)  Flag_Target=0;  //分频处理
      if(Flag_Target==1)    //第一个10ms 采集Y轴数据
    {    
       digitalWrite(YL, LOW);  //给X方向+3.3V电压
       digitalWrite(YH, HIGH);   
       digitalWrite(XL, HIGH);  
       digitalWrite(XH, LOW); 
       Position_Y=analogRead(3)/5; //测量Y方向的坐标          
       digitalWrite(YL, HIGH);  
       digitalWrite(YH, LOW); 
       digitalWrite(XL, LOW);  
       digitalWrite(XH, HIGH); 
    }
       else   if(Flag_Target==2) //第二个10ms 接收PS2遥控并调节PID参数
    {
        if(PS2_KEY==4096||PS2_KEY==8192||PS2_KEY==16384||PS2_KEY==32768)Flag_Move=PS2_KEY/4096;   //判断是否满足设定动作执行条件
        if(Flag_Move==0)Get_RC();      //PS2遥控方向
        else  Setting_Move();          //设定动作
        Adjust();  //PID参数调节
    }
     else   if(Flag_Target==3) //第三个10ms 采集X轴数据
    {
       digitalWrite(YL, HIGH);  //给Y方向+3.3V电压//D3
       digitalWrite(YH, LOW); //  A0
       digitalWrite(XL, LOW);  //  D2
       digitalWrite(XH, HIGH); //  A1
       Position_X= analogRead(2)*4/15; //测量X方向的           
       digitalWrite(YL, LOW);  //
       digitalWrite(YH, HIGH); //  A0
       digitalWrite(XL, HIGH);  //  D2
       digitalWrite(XH, LOW); //  A1
     }
      else  if(Flag_Target==4)   //第四个10ms PID控制
    {
      Target_X=-balanceX(Position_X);   //X方向的PID控制器
      Target_Y=-balanceY(Position_Y);   //Y方向的PID控制器
      if(Target_X<-Max_Target) Target_X=-Max_Target;  //X方向的舵机的控制最大角度
      if(Target_X>Max_Target)  Target_X=Max_Target;   //X方向的舵机的控制最大角度
      if(Target_Y<-Max_Target) Target_Y=-Max_Target;  //Y方向的舵机的控制最大角度
      if(Target_Y>Max_Target)  Target_Y=Max_Target;   //Y方向的舵机的控制最大角度
      if(Flag_Stop==0)Control_servo(Target_X,Target_Y); //不存在异常，控制舵机
    }             
 }
/**************************************************************************
函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void setup()   {                
   oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
   oled.clear();   // clears the screen and buffer
   Serial.begin(128000);           //开启串口
   delay(200);                    //延时等待初始化完成
    pinMode(XL, OUTPUT);          //电机控制引脚
    pinMode(XH, OUTPUT);          //电机控制引脚，
    pinMode(YL, OUTPUT);          //电机速度控制引脚
    pinMode(YH, OUTPUT);          //电机速度控制引脚
   myservo1.attach(10);           //初始化各个舵机
   myservo2.attach(9);            //初始化各个舵机
   MsTimer2::set(10, control);    //使用Timer2设置定时中断
   MsTimer2::start();             //中断使能
   error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);//初始化PS2手柄
}
/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop()   {        
  if(Flag_Show==0) 
  {          
    //----------------------0-------------------------------------
    if(Flag_Stop==0)  oled.drawstring(00, 0, "Servo O-N");
    else              oled.drawstring(00, 0, "Servo OFF");
    if(error==0)      oled.drawstring(70, 0, "PS2 O-N");
    else              oled.drawstring(70, 0, "PS2 OFF");
     //---------------------1--------------------------------   
      oled.drawstring(00, 1, "LY");     
      OLED_ShowNumber(15, 1, PS2_LY, 3);
      oled.drawstring(60, 1, "LX");     
      OLED_ShowNumber(75, 1, PS2_LX, 3);             
     //---------------------2-------------------------------- 
      oled.drawstring(00, 2, "RY");     
      OLED_ShowNumber(15, 2, PS2_RY, 3);
      oled.drawstring(60, 2, "RX");     
      OLED_ShowNumber(75, 2, PS2_RX, 3); 
     //---------------------3--------------------------------    
      oled.drawstring(00, 3, "KP");     
      OLED_ShowNumber(15, 3, Balance_Kp, 3);
      oled.drawstring(60, 3, "KD");     
      OLED_ShowNumber(75, 3, Balance_Kd, 3);
     //---------------------4-------------------------------- 
      oled.drawstring(00, 4, "X-");     
      if( Zero_X<0)    oled.drawstring(30, 4, "-"),  
                       OLED_ShowNumber(45, 4, -Zero_X, 3);
      else             oled.drawstring(30, 4, "+"),  
                       OLED_ShowNumber(45, 4, Zero_X, 3);
    
      if( Position_X<0)oled.drawstring(80, 4, "-"),  
                       OLED_ShowNumber(95, 4, -Position_X, 3);
      else             oled.drawstring(80, 4, "+"),  
                       OLED_ShowNumber(95, 4, Position_X, 3);     
    //---------------------5-------------------------------- 
      oled.drawstring(00, 5, "Y-");     
      if( Zero_Y<0)    oled.drawstring(30, 5, "-"),  
                       OLED_ShowNumber(45, 5, -Zero_Y, 3);
      else             oled.drawstring(30, 5, "+"),  
                       OLED_ShowNumber(45, 5, Zero_Y, 3);
      if( Position_Y<0)oled.drawstring(80, 5, "-"),  
                       OLED_ShowNumber(95, 5, -Position_Y, 3);
      else             oled.drawstring(80, 5, "+"),  
                       OLED_ShowNumber(95, 5, Position_Y, 3); 
  //---------------------6--------------------------------     
                       oled.drawstring(0, 6, "PS2_KEY"),  
                       OLED_ShowNumber(45, 6, PS2_KEY,5);                
    //////=============刷新=======================//
     oled.display();  
     if(error==0) PS2X_RX();//接收PS2信息
  }
 else
 DataScope();//通过上位机发送数据到PC
}

