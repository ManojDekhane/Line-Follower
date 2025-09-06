#include <QTRSensors.h>

#define PWM1 9
#define AIN1 4
#define AIN2 3
#define PWM2 10
#define BIN1 6
#define BIN2 7
#define STDBY 5

QTRSensors qtr;

int P, D, I, previousError, PIDvalue, error, lsp, rsp, i;
int speed = 200;

// 0.0848
// 0.08435

// kd = 0.039

// float Kp = 0.131;
// float Kd = 0.04;
// float Ki = 0.0075;

float Kp = 0.131;
float Kd = 0.06;
float Ki = 0.000;

int position;
const uint8_t SensorCount = 8;
int threshold;

uint16_t sensorValues[SensorCount];

void setup()
{
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(11, INPUT_PULLUP);

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    qtr.setEmitterPin(2);

    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("calibrate");

    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
        // digitalWrite(AIN1, 1);
        // digitalWrite(AIN2, 0);
        // digitalWrite(BIN1, 0);
        // digitalWrite(BIN2, 1);
        // analogWrite(PWM1, 100);
        // analogWrite(PWM2, 100);
    }

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        threshold += (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
    }
    threshold /= SensorCount;

    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    digitalWrite(STDBY, 1);
    digitalWrite(LED_BUILTIN, LOW);
    while (digitalRead(11))
    {
    }
    delay(10);
}

void loop()
{

    // for(i=0;i<=SensorCount;i++)
    // {
    //   sensorValues[i]=analogRead(val[i]);
    // }
    position = qtr.readLineBlack(sensorValues);

    // bool straightLineAhead = (sensorValues[3] >= threshold && sensorValues[4] >= threshold);

    // 90degree right turn 0123,012,01,12,123,01234,0,012345,0123456
    if ((sensorValues[0] >= threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
        (sensorValues[0] >= threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] >= threshold && sensorValues[5] >= threshold && sensorValues[6] >= threshold && sensorValues[7] < threshold) ||
        (sensorValues[0] >= threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] >= threshold && sensorValues[5] >= threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||

        (sensorValues[0] >= threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
        (sensorValues[0] >= threshold && sensorValues[1] >= threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
        (sensorValues[0] >= threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] >= threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
        (sensorValues[0] < threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
        (sensorValues[0] < threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
        (sensorValues[0] >= threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold)

    )
    {
        // if(straightLineAhead){
        //   linefollow();
        // } else {
        digitalWrite(AIN1, 1);
        digitalWrite(AIN2, 0);
        digitalWrite(BIN1, 0);
        digitalWrite(BIN2, 1);
        analogWrite(PWM1, 200);
        analogWrite(PWM2, 200);
        // }
    }
    // 90degree left turn 7654,765,76,65,654,76543,7,765432,7654321
    else if ((sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] >= threshold && sensorValues[3] >= threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] >= threshold && sensorValues[3] >= threshold && sensorValues[2] >= threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] >= threshold && sensorValues[3] >= threshold && sensorValues[2] >= threshold && sensorValues[1] >= threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] >= threshold && sensorValues[3] >= threshold && sensorValues[2] >= threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] >= threshold && sensorValues[3] < threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] < threshold && sensorValues[3] < threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] < threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] >= threshold && sensorValues[3] < threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] < threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] < threshold && sensorValues[3] < threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] < threshold && sensorValues[4] < threshold && sensorValues[3] < threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] < threshold && sensorValues[5] < threshold && sensorValues[4] < threshold && sensorValues[3] < threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold) ||
             (sensorValues[7] >= threshold && sensorValues[6] >= threshold && sensorValues[5] >= threshold && sensorValues[4] >= threshold && sensorValues[3] < threshold && sensorValues[2] < threshold && sensorValues[1] < threshold && sensorValues[0] < threshold)

    )
    {
        // if(straightLineAhead) {
        //   linefollow();
        // } else {
        digitalWrite(AIN1, 0);
        digitalWrite(AIN2, 1);
        digitalWrite(BIN1, 1);
        digitalWrite(BIN2, 0);
        analogWrite(PWM1, 200);
        analogWrite(PWM2, 200);
        // }
    }
    // diamond (preffered left turn) 2345, 123456, 1256, 12456, 25, 12356
    //  else if((sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] >= threshold && sensorValues[5] >= threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
    //         (sensorValues[0] < threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] >= threshold && sensorValues[5] >= threshold && sensorValues[6] >= threshold && sensorValues[7] < threshold) ||
    //         (sensorValues[0] < threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] >= threshold && sensorValues[6] >= threshold && sensorValues[7] < threshold) ||
    //         (sensorValues[0] < threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] < threshold && sensorValues[4] >= threshold && sensorValues[5] >= threshold && sensorValues[6] >= threshold && sensorValues[7] < threshold) ||
    //         (sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] >= threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] >= threshold && sensorValues[6] < threshold && sensorValues[7] < threshold) ||
    //         (sensorValues[0] < threshold && sensorValues[1] >= threshold && sensorValues[2] >= threshold && sensorValues[3] >= threshold && sensorValues[4] < threshold && sensorValues[5] >= threshold && sensorValues[6] >= threshold && sensorValues[7] < threshold)||
    //         (sensorValues[0] < threshold && sensorValues[1] >= threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] >= threshold && sensorValues[7] < threshold)

    // )
    // {
    //   digitalWrite(AIN1, 1);
    //   digitalWrite(AIN2, 0);
    //   digitalWrite(BIN1, 0);
    //   digitalWrite(BIN2, 1);
    //   analogWrite(PWM1, 200);
    //   analogWrite(PWM2, 200);

    // }
    // else if(sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold)
    //   {
    //     digitalWrite(AIN1, 1);
    //   digitalWrite(AIN2, 0);
    //   digitalWrite(BIN1, 0);
    //   digitalWrite(BIN2, 1);
    //   analogWrite(PWM1, 100);
    //   analogWrite(PWM2, 100);
    // }
    else
    {
        linefollow();
    }
}

void linefollow()
{
    int error = position - 3500;
    // Serial.println(error);

    P = error;
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    rsp = speed - PIDvalue;
    lsp = speed + PIDvalue;

    if (lsp > 255)
    {
        lsp = 255;
    }
    if (lsp < 0)
    {
        lsp = 0;
    }
    if (rsp > 255)
    {
        rsp = 255;
    }
    if (rsp < 0)
    {
        rsp = 0;
    }

    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);

    analogWrite(PWM1, rsp);
    Serial.print(rsp);
    analogWrite(PWM2, lsp);
    Serial.print("\t");
    Serial.println(lsp);
}