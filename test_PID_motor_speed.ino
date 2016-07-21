#include <PID_v1.h>

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 280, 0.01, DIRECT);

unsigned long count = 0;
void myisr(void)
{
  count++;
}

void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(2), myisr, RISING );
  Setpoint = 5000;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);//PID 采样周期 10ms
}

unsigned long lastt1 = 0;
unsigned long lastt2 = 0;
unsigned long lastCount1 = 0;
unsigned long lastCount2 = 0;
int in = 0;
unsigned long rpm = 0;

void loop() {

  if (millis() - lastt1 >= 10)//10ms 获取一次Input,并更新Setpoint
  {
    lastt1 = millis();
    Input = count - lastCount1;
    lastCount1 = count;
    Serial.print("I=");
    Serial.print(Input);
    Serial.print("\tS=");
    in = analogRead(A0);//电位器输入 Setpoint
    in = map(in, 0, 1023, 0, 13);//10ms内，电机最大转速，码盘最多能产生13个脉冲
    Setpoint = in;
    Serial.print(Setpoint);
    Serial.print("\tO=");
    Serial.print(Output);
    Serial.print("\trpm=");
    Serial.println(rpm);
  }
  if (millis() - lastt2 >= 500)//500ms 做一次RPM计算
  {
    lastt2 = millis();
    rpm = (count - lastCount2) * 24;
    lastCount2 = count;
  }

  myPID.Compute();
  analogWrite(6, LOW);
  analogWrite(5, Output);
}
