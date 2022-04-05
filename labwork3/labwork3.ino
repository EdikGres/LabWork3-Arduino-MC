/*
 * in Arduino/hardware/avr/variants/standard:
 * #define PIN_A0   (14)
 * #define PIN_A1   (15)
 * #define PIN_A2   (16)
 * #define PIN_A3   (17)
 * #define PIN_A4   (18)
 * #define PIN_A5   (19)
 * differenceA 14-0=14
 * Variant: 9         A4>A2 | A2<A4+0.7В | A0>A2+A3 | A0<A3
 */
//#define DEBUG
#define DIFF_A 14
#define DIFF_LED 2
#define DELAY 1000
#define LED1 2
#define LED2 3
#define LED3 4
#define LED4 5


//Среднее арифметическое
const int NUM_READ = 10;  // количество усреднений для средних арифм. фильтров
//Калман
float _err_measure = 0.8;  // примерный шум измерений
float _q = 0.1;   // скорость изменения значений 0.001-1, варьировать самому
//Альфа-Бета фильтр
// период дискретизации (измерений), process variation, noise variation
float dt = 0.02;
float sigma_process = 3.0;
float sigma_noise = 0.7;
//Экспоненциальное сглаживание
float k = 0.1;  // коэффициент фильтрации, 0.0-1.0

unsigned int Apins[6] = {};
float Avolts[6] = {};
float Filtered[5] = {};
byte LEDs[4] = {};

void setup() {
  Serial.begin(9600);
  for(int i = 2; i < 6; i++ )
    pinMode(i, OUTPUT);
}

void loop() {
  for(int i = 0; i < sizeof(Apins)/sizeof(unsigned int); i++)
    Apins[i] = analogRead(i+DIFF_A);
    //a_print();
    toVolt();
    checkLED();
#ifdef DEBUG
    a_print();
    v_print();
    f_print();
#endif
    _print();
    delay(DELAY);
}
#ifdef DEBUG
void a_print(void) {
  Serial.print("VALUE\n");
  for(int i = 0; i < sizeof(Apins)/sizeof(unsigned int); i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print(" : ");
    Serial.print(Apins[i]);
    Serial.print("\n");
  }
  Serial.print("-----\n");
}

void v_print(void) {
  Serial.print("VOLTS\n");
  for(int i = 0; i < sizeof(Avolts)/sizeof(float); i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print(" : ");
    Serial.print(Avolts[i]);
    Serial.print("\n");
  }
  Serial.print("-----\n");
}

void f_print(void) {
  Serial.print("FILTERED\n");
  for(int i = 0; i < sizeof(Filtered)/sizeof(float); i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print(" : ");
    Serial.print(Filtered[i]);
    Serial.print("\n");
  }
  Serial.print("-----\n");
}
#endif

void _print(void) {
  Serial.print("VALUE\t");
  Serial.print("VOLTS\t");
  Serial.print("FILTERED\t");
  Serial.print("LED\t\n");
  for(int i = 0; i < 6; i++) {
    Serial.print(Apins[i]);
    Serial.print("\t");

    Serial.print(Avolts[i]);
    Serial.print("\t");

    if(i < 5){
      Serial.print(Filtered[i]);
      Serial.print("\t\t");
    }
    if(i < 4) {
      Serial.print(LEDs[i]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
}

void toVolt(void) {
  for (int i = 0; i < sizeof(Avolts)/sizeof(float); i++)
    Avolts[i] = map(Apins[i], 0, 1023, 0, 5000 ) / 1000.0;
}

void filter(void) {
  Filtered[0] = runMiddleArifm(Avolts[0]);
  Filtered[1] = median(Avolts[1]);
  Filtered[2] = simpleKalman(Avolts[2]);
  Filtered[3] = ABfilter(Avolts[3]);
  Filtered[4] = expRunningAverage(Avolts[4]);
}

void checkLED(void) {
  filter();
  //A4>A2
  LEDs[0] = (Filtered[4] > Filtered[2]) ? 1 : 0;
  //A2<A4+0.7В
  LEDs[1] = (Filtered[2] < (Filtered[4] + 0.7)) ? 1 : 0;
  //A0>A2+A3
  LEDs[2] = (Filtered[0] > (Filtered[2] + Filtered[3])) ? 1 : 0;
  //A0<A3
  LEDs[3] = (Filtered[0] < Filtered[3]) ? 1 : 0;
  for(int i = LED1; i <= LED4; i++)
    digitalWrite(i, LEDs[i-DIFF_LED]);
}

//бегущее среднее
float runMiddleArifm(float newVal) {  // принимает новое значение
  static byte idx = 0;                // индекс
  static float valArray[NUM_READ];    // массив
  valArray[idx] = newVal;             // пишем каждый раз в новую ячейку
  if (++idx >= NUM_READ) idx = 0;     // перезаписывая самое старое значение
  float average = 0;                  // обнуляем среднее
  for (int i = 0; i < NUM_READ; i++) {
    average += valArray[i];           // суммируем
  }
  return (float)average / NUM_READ;   // возвращаем
}
//медианный
float median(float newVal) {
  static float buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
}
//Фильтр Каллмана
float simpleKalman(float newVal) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}
//альфа-бета фильтр
float ABfilter(float newVal) {
  static float xk_1, vk_1, a, b;
  static float xk, vk, rk;
  static float xm;
  float lambda = (float)sigma_process * dt * dt / sigma_noise;
  float r = (4 + lambda - (float)sqrt(8 * lambda + lambda * lambda)) / 4;
  a = (float)1 - r * r;
  b = (float)2 * (2 - a) - 4 * (float)sqrt(1 - a);
  xm = newVal;
  xk = xk_1 + ((float) vk_1 * dt );
  vk = vk_1;
  rk = xm - xk;
  xk += (float)a * rk;
  vk += (float)( b * rk ) / dt;
  xk_1 = xk;
  vk_1 = vk;
  return xk_1;
}
// экспоненциальное сглаживание
float expRunningAverage(float newVal) {
  static float filVal = 0;
  filVal += (newVal - filVal) * k;
  return filVal;
}
