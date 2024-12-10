#include <SPI.h>
//#include "ADI_pedometer_algorithm.h"
#include "ADXL367.h"
ADXL367 XL367;

//-------------------------------------- MAIN VARIABLES -------------------------------------------------------//
float X_gs, Y_gs, Z_gs;
int16_t X, Y, Z;
const uint8_t Sample_Sets = 8;  //Sample Set = X,Y,Z 16bit data
uint8_t FIFO_buffer[Sample_Sets * 6];
uint8_t range = 0;
uint8_t reg_val = 0;
uint16_t aux = 0;
const uint8_t INT1_pin = 9;  //INT1 to pin D9
uint16_t num_stepsTotal = 0;
num_stepsTotal //サイコロの値
int sai = 0; //サイコロの最終的な値、出力する値
int32_t loopsCount = 0;
int v_count = 0;  // 連続同じ値のカウント
int last_v = -1;  // 最後のvの値（初期値は無効な値）
void setup() {

  Serial.begin(115200);
  while (!Serial);

  //加速度センサーの初期化
  pinMode(INT1_pin, INPUT);  //INT1ピンを入力モードに設定する
  XL367.begin(5, 4000000);  //CP = pin 10, CLK freq = 4MHz
                             // SPIプロトコルの設定, デバイスのソフトリセットを発行(内部の状態を初期化)
  range = XL367.SPI_set_range(ADXL367_RANGE_2G);

  XL367.SPI_set_output_rate(ADXL367_ODR_50_HZ);

  XL367.SPI_get_register_value(&reg_val, ADXL367_REG_FILTER_CTL, 1);

  XL367.SPI_fifo_setup(ADXL367_FIFO_STREAM, Sample_Sets * 3, false);

  XL367.SPI_get_register_value(&reg_val, ADXL367_REG_FIFO_CTL, 1);

  XL367.SPI_get_register_value(&reg_val, ADXL367_REG_FIFO_SAMPLES, 1);

  XL367.SPI_set_register_value(ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1,
                               ADXL367_REG_INTMAP1_LOWER, 1);

  XL367.SPI_get_register_value(&reg_val, ADXL367_REG_INTMAP1_LOWER, 1);

  XL367.SPI_set_power_mode(1);  // ADXL367を測定モードに切り替える

  delay(300);

  //InitAlgorithmParameters()  アルゴリズムの設定値を初期化;
  Serial.print("loops");
  Serial.print("\t");
  Serial.print("X_gs");
  Serial.print("\t");
  Serial.print("Y_gs");
  Serial.print("\t");
  Serial.print("Z_gs");
  Serial.print("\t");
  Serial.print("steps");
  Serial.print("\n");
}
void loop() {

  // 加速度センサーからデータを取得する
  //while(digitalRead(INT1_pin) == 0){}

  XL367.SPI_get_fifo_data(&FIFO_buffer[0], Sample_Sets * 6);
  for (int i = 0; i < Sample_Sets * 6; i = i + 6) {

    uint16_t aux = 0;

    aux = ((FIFO_buffer[i] << 8) | FIFO_buffer[i + 1]) & 0x3FFF;
    X = XL367.twos_complement(aux, 14);
    aux = ((FIFO_buffer[i + 2] << 8) | FIFO_buffer[i + 3]) & 0x3FFF;
    Y = XL367.twos_complement(aux, 14);
    aux = ((FIFO_buffer[i + 4] << 8) | FIFO_buffer[i + 5]) & 0x3FFF;
    Z = XL367.twos_complement(aux, 14);

    //num_stepsTotal = StepAlgorithm(X, Y, Z, num_stepsTotal);

    X_gs = (float)X / (1000 * 8 / range);
    Y_gs = (float)Y / (1000 * 8 / range);
    Z_gs = (float)Z / (1000 * 8 / range);
    //サイコロの値を決めてる、基本的に+-0.05くらい
    if(X_gs >= -0.03 && X_gs <= 0.07 && Y_gs >= -0.05 && Y_gs <= 0.08 && Z_gs >= 1.00 && Z_gs <= 1.10){
        v=1;
    }
    else if(X_gs >= 0.95 && X_gs <= 1.05 && Y_gs >= 0.03 && Y_gs <= 0.13 && Z_gs >= -0.08 && Z_gs <= 0.02){
        v=2;
    }
    else if(X_gs >= -0.04 && X_gs <= 0.06 && Y_gs >= 0.93 && Y_gs <= 1.03 && Z_gs >= -0.03 && Z_gs <= 0.07){
        v=3;
    }
    else if(X_gs >= -0.03 && X_gs <= 0.07 && Y_gs >= -1.00 && Y_gs <= -0.89 && Z_gs >= -0.02 && Z_gs <= 0.10){
        v=4;
    } 
    else if(X_gs >= -1.00 && X_gs <= -0.90 && Y_gs >= -0.05 && Y_gs <= 0.08 && Z_gs >= -0.03 && Z_gs <= 0.10){
        v=5;
    }
    else if(X_gs >= -0.03 && X_gs <= 0.05 && Y_gs >= -0.20 && Y_gs <= 0.10 && Z_gs >= -1.00 && Z_gs <= 0.06){
        v=6;
    }else{
      v=0;
    }
    // vが0の場合はカウントしない
    if (v == 0) {
      v_count = 0;  // vが0の場合はカウントリセット
      last_v = v;  // 前回のvも更新
      continue;  // 次のループに進む
    }

    // vが前回と同じ場合の処理
    if (v == last_v) {
      v_count++;  // 同じ値が続いた場合はカウントアップ
    } else {
      v_count = 0;  // vが変化した場合はカウントリセット
    }

    // vが40回連続で同じ数字の場合に出力
    if (v_count >= 40) {
      sai=v;
      Serial.print("設定");
      Serial.println(sai);
      v_count = 0;  // 出力したらカウントリセット
    }

    // 最後のvを更新
    last_v = v;

    //シリアル通信のデータや状態を更新する
    //       Serial.print(loopsCount);
    //       Serial.print("\t");
    Serial.print(X_gs);
    Serial.print("\t");
    Serial.print(Y_gs);
    Serial.print("\t");
    Serial.print(Z_gs);
    Serial.print("\t");
    Serial.print(v);
    //       Serial.print("\t");
    //       Serial.print(num_stepsTotal);
    Serial.print("\n");
    loopsCount++;
    delay(100);
   
  }
  
}
