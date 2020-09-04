//座位加速度传感器，使用M5FIRE。每分钟传输一次三轴加速度的总和给ambidata
//改进：先累计数据，每5~10分钟发送一次数据（省电）
/*
  被験者椅子3
  ID       readKey            writeKey
  7586 907fd6300ebed2c9  eb9a754dc2ff43f4  2018/11/6
  被験者椅子2
  7585  a5516f480324ea72  5d8fe00c6395f818  2018/11/6
  被験者椅子1
  7515  07b036bf7ebdff1d  3e5c89fe184693de  2018/11/2
*/
#include <M5Stack.h>
#include "utility/MPU9250.h"
#include <Wire.h>
#include "Ambient.h"

//Ambient和wifi的设置
//研究室wifi aterm-ac2876-g
//pass       3295a7101338b
//杉本さんMY ASUS   2d2bc72444a8
WiFiClient client;
Ambient ambient;
const char* ssid = "MY ASUS";
const char* password = "2d2bc72444a8";
#define SEAT_ID 1       //传感器编号1~3
unsigned int channelId = 0; // AmbientのチャネルID
const char* writeKey = ""; // ライトキー
float abs_g_sum = 0;
float max_g = 0;
int pt = 0;
#define PERIOD 60


MPU9250 IMU;//加速度传感器

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Wire.begin();
  M5.Lcd.println("wifi connecting...");
  WiFi.begin(ssid, password);  //  Wi-Fi APに接続
  while (WiFi.status() != WL_CONNECTED) {  //  Wi-Fi AP接続待ち
    delay(100);
  }

  M5.Lcd.println("connected");

  M5.Lcd.println("wifi connected");
  delay(1000);

  switch (SEAT_ID) {
    case 1:
      channelId = 7515; // AmbientのチャネルID
      writeKey = "3e5c89fe184693de"; // ライトキー
      break;
    case 2:
      channelId = 7585; // AmbientのチャネルID
      writeKey = "5d8fe00c6395f818"; // ライトキー
      break;
    case 3:
      channelId = 7586; // AmbientのチャネルID
      writeKey = "eb9a754dc2ff43f4"; // ライトキー
      break;

  }


  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);
  dacWrite(25, 0);
  ambient.begin(channelId, writeKey, &client); // チャネルIDとライトキーを指定してAmbientの初期化
  M5.Lcd.println("ambient set");
  delay(1000);
  M5.Lcd.setBrightness(0);
  M5.Lcd.sleep();
  M5.Speaker.end();
}

void loop() {
  int abs_g = 0;
  dacWrite(25, 0);


  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    /* //获取角度
      IMU.readAccelData(IMU.accelCount);
      IMU.getAres();

      IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
      IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
      IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];
    */
    //获取加速度
    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();


    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;
    /*
      //获取磁场
      IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
      IMU.getMres();
      // User environmental x-axis correction in milliGauss, should be
      // automatically calculated
      //IMU.magbias[0] = +470.;
      // User environmental x-axis correction in milliGauss TODO axis??
      //IMU.magbias[1] = +120.;
      // User environmental x-axis correction in milliGauss
      //IMU.magbias[2] = +125.;

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
      IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
      IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];

      IMU.tempCount = IMU.readTempData();  // Read the adc values
      // Temperature in degrees Centigrade
      IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;

      int x = 64 + 10;
      int y = 128 + 20;
      int z = 192 + 30;
    */

    float abs_g = abs(IMU.gx) + abs(IMU.gy) + abs(IMU.gz);
    max_g = (max_g > IMU.gx) ? max_g : IMU.gx;
    max_g = (max_g > IMU.gy) ? max_g : IMU.gy;
    max_g = (max_g > IMU.gz) ? max_g : IMU.gz;


    /*

            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setTextColor(GREEN , BLACK);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(0, 0); M5.Lcd.print("MPU9250/AK8963");
        M5.Lcd.setCursor(0, 32); M5.Lcd.print("x");
        M5.Lcd.setCursor(x, 32); M5.Lcd.print("y");
        M5.Lcd.setCursor(y, 32); M5.Lcd.print("z");

        M5.Lcd.setTextColor(YELLOW , BLACK);
        M5.Lcd.setCursor(0, 48 * 2); M5.Lcd.print((int)(1000 * IMU.ax));
        M5.Lcd.setCursor(x, 48 * 2); M5.Lcd.print((int)(1000 * IMU.ay));
        M5.Lcd.setCursor(y, 48 * 2); M5.Lcd.print((int)(1000 * IMU.az));
        M5.Lcd.setCursor(z, 48 * 2); M5.Lcd.print("mg");

        M5.Lcd.setCursor(0, 64 * 2); M5.Lcd.print((int)(IMU.gx));
        M5.Lcd.setCursor(x, 64 * 2); M5.Lcd.print((int)(IMU.gy));
        M5.Lcd.setCursor(y, 64 * 2); M5.Lcd.print((int)(IMU.gz));
        //M5.Lcd.setCursor(x, 64 * 2); M5.Lcd.print(abs(IMU.gy));
        //M5.Lcd.setCursor(y, 64 * 2); M5.Lcd.print(abs_g);
        //M5.Lcd.setCursor(z, 64 * 2); M5.Lcd.print(abs_g_sum);
          M5.Lcd.setCursor(z, 64 * 2); M5.Lcd.print("o/s"); //这个是加速度

            M5.Lcd.setCursor(0, 80 * 2); M5.Lcd.print((int)(IMU.mx));
            M5.Lcd.setCursor(x, 80 * 2); M5.Lcd.print((int)(IMU.my));
            M5.Lcd.setCursor(y, 80 * 2); M5.Lcd.print((int)(IMU.mz));
            M5.Lcd.setCursor(z, 80 * 2); M5.Lcd.print("mG");

            M5.Lcd.setCursor(0,  96 * 2); M5.Lcd.print("Gyro Temperature ");
            M5.Lcd.setCursor(z,  96 * 2); M5.Lcd.print(IMU.temperature, 1);
            M5.Lcd.print(" C");
            delay(100);
    */
    delay(500);
    abs_g_sum = abs_g_sum + abs_g;
    int t = millis();
    if ( (t - pt) > (30 * 1000) ) { //累积1分钟时上报数据
      pt = millis() - ((t - pt) - 30 * 1000);
      ambient.set(1, abs_g_sum);
      ambient.set(2, max_g);

      ambient.send();
      //M5.Lcd.print("data send");
      //M5.Lcd.println(abs_g_sum);
      //M5.Lcd.println(IMU.gy);
      max_g = 0;
      abs_g_sum = 0;
    }
  }


}
