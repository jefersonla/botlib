#include <TimerOne.h>
#include <PID_v1.h>

#define INTERROMPE_DIREITA 3
#define INTERROMPE_ESQUERDA 2

#define ACELERADOR_DIREITA 10
#define ACELERADOR_ESQUERDA 9

#define LUZ_ACELERADOR_DIREITA 12
#define LUZ_ACELERADOR_ESQUERDA 13

#define DIRECAO_DIREITA_1 7
#define DIRECAO_DIREITA_2 8

#define DIRECAO_ESQUERDA_1 5
#define DIRECAO_ESQUERDA_2 6

#define IR_PARA_FRENTE_DIREITA() do { digitalWrite(DIRECAO_DIREITA_1, HIGH); digitalWrite(DIRECAO_DIREITA_2, LOW); } while(false)
#define IR_PARA_FRENTE_ESQUERDA() do { digitalWrite(DIRECAO_ESQUERDA_1, HIGH); digitalWrite(DIRECAO_ESQUERDA_2, LOW); } while(false)
#define IR_PARA_FRENTE() do { IR_PARA_FRENTE_DIREITA(); IR_PARA_FRENTE_ESQUERDA(); } while(false)

#define IR_PARA_TRAS_DIREITA() do { digitalWrite(DIRECAO_DIREITA_1, LOW); digitalWrite(DIRECAO_DIREITA_2, HIGH); } while(false)
#define IR_PARA_TRAS_ESQUERDA() do { digitalWrite(DIRECAO_ESQUERDA_1, LOW); digitalWrite(DIRECAO_ESQUERDA_2, HIGH); } while(false)
#define IR_PARA_TRAS() do { IR_PARA_TRAS_DIREITA(); IR_PARA_TRAS_ESQUERDA(); } while(false)

#define ACELERA_DIREITA(VELOCIDADE) do { pwmDireita = VELOCIDADE; Timer1.pwm(ACELERADOR_DIREITA, VELOCIDADE); } while(false)
#define ACELERA_ESQUERDA(VELOCIDADE) do { pwmEsquerda = VELOCIDADE; Timer1.pwm(ACELERADOR_ESQUERDA, VELOCIDADE); } while(false)
#define ACELERA(VELOCIDADE) do { ACELERA_DIREITA(VELOCIDADE); ACELERA_ESQUERDA(VELOCIDADE); } while(false)

#define FREIO_DIREITA() { ACELERA_DIREITA(0); digitalWrite(DIRECAO_DIREITA_1, LOW); digitalWrite(DIRECAO_DIREITA_2, LOW); } while(false)
#define FREIO_ESQUERDA() { ACELERA_ESQUERDA(0); digitalWrite(DIRECAO_ESQUERDA_1, LOW); digitalWrite(DIRECAO_ESQUERDA_2, LOW); } while(false)
#define FREIO() do { FREIO_DIREITA(); FREIO_ESQUERDA(); } while(false)

volatile int contaDireita = 0;
volatile int contaEsquerda = 0;

double kpEsquerda = 4.85;
double kpDireita = 5.7;

double kiEsquerda = 1.3;
double kiDireita = 2;

double kdEsquerda = 1.1;
double kdDireita = 1.1;

//volatile int velocidadeDireita = 120;
//volatile int velocidadeEsquerda = 130;
//volatile int velocidadeDireita = 800;
//volatile int velocidadeEsquerda = 870;
volatile int velocidadeDireita = 540;
volatile int velocidadeEsquerda = 600;

double pwmDireita = 0;
double pwmEsquerda = 0;

volatile bool motoresAtivados = false;

volatile bool pidAtivado = true;

volatile int contador = 0;

volatile int anteriorDireita = 0;
volatile int anteriorEsquerda = 0;

volatile int girosDesejados = 150;

//Define Variables we'll be connecting to
double entradaEsquerda, entradaDireita;
volatile double objetivo = 100;

#define USE_PID

#ifdef USE_PID
//Specify the links and initial tuning parameters
PID motorEsquerdo(&entradaEsquerda, &pwmEsquerda, &objetivo, kpEsquerda, kiEsquerda, kdEsquerda, DIRECT);

//Specify the links and initial tuning parameters
PID motorDireito(&entradaDireita, &pwmDireita, &objetivo, kpDireita, kiDireita, kdDireita, DIRECT);
#endif

#define MUTEX_PRINT 14
#define DIVISOES 5
#define META (80 / DIVISOES)


void setup() {
  Serial.begin(115200);

  pinMode(ACELERADOR_DIREITA, OUTPUT);
  pinMode(ACELERADOR_ESQUERDA, OUTPUT);
  pinMode(LUZ_ACELERADOR_DIREITA, OUTPUT);
  pinMode(LUZ_ACELERADOR_ESQUERDA, OUTPUT);
  pinMode(DIRECAO_DIREITA_1, OUTPUT);
  pinMode(DIRECAO_DIREITA_2, OUTPUT);
  pinMode(DIRECAO_ESQUERDA_1, OUTPUT);
  pinMode(DIRECAO_ESQUERDA_2, OUTPUT);
  pinMode(MUTEX_PRINT, OUTPUT);

  pinMode(INTERROMPE_DIREITA, INPUT_PULLUP);
  pinMode(INTERROMPE_ESQUERDA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INTERROMPE_DIREITA), contadorDireita, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INTERROMPE_ESQUERDA), contadorEsquerda, CHANGE);

#ifdef USE_PID
  motorDireito.SetOutputLimits(0, 1023);
  motorEsquerdo.SetOutputLimits(0, 1023);
  motorDireito.SetSampleTime(1000 / DIVISOES);
  motorEsquerdo.SetSampleTime(1000 / DIVISOES);
#endif

  Timer1.initialize(1000000 / DIVISOES);
  Timer1.attachInterrupt(ajustaMotor);
}

void loop() {
  if (Serial.available() > 0) {
    delay(5);
    char serialLido[20];
    strncpy(serialLido, Serial.readStringUntil('\n').c_str(), 20);
    float valor_float;
    int valor_int;
    switch (serialLido[0]) {
      case 'I':
        motoresAtivados = true;
#ifdef USE_PID
        motorDireito.SetMode(AUTOMATIC);
        motorEsquerdo.SetMode(AUTOMATIC);
#endif
        contaDireita = 0;
        contaEsquerda = 0;
        anteriorDireita = 0;
        anteriorEsquerda = 0;
        ACELERA_DIREITA(velocidadeDireita);
        ACELERA_ESQUERDA(velocidadeEsquerda);
        IR_PARA_FRENTE();
        break;
      case 'S':
        motoresAtivados = false;
#ifdef USE_PID
        motorDireito.SetMode(MANUAL);
        motorEsquerdo.SetMode(MANUAL);
#endif
        FREIO();
        break;
      case 'L':
        switch (serialLido[1]) {
          case 'P':
            valor_float = atof(&serialLido[3]);
            kpEsquerda = valor_float;
#ifdef USE_PID
            motorEsquerdo.SetTunings(kpEsquerda, kiEsquerda, kdEsquerda);
#endif
            break;
          case 'I':
            valor_float = atof(&serialLido[3]);
            kiEsquerda = valor_float;
#ifdef USE_PID
            motorEsquerdo.SetTunings(kpEsquerda, kiEsquerda, kdEsquerda);
#endif
            break;
          case 'D':
            valor_float = atof(&serialLido[3]);
            kdEsquerda = valor_float;
#ifdef USE_PID
            motorEsquerdo.SetTunings(kpEsquerda, kiEsquerda, kdEsquerda);
#endif
            break;
          case 'S':
            valor_int = atoi(&serialLido[3]);
            velocidadeEsquerda = valor_int;
            ACELERA_ESQUERDA(velocidadeEsquerda);
            break;
          default:
            Serial.print("CODE UNRECOGNIZED 2 ");
            Serial.println(serialLido[1]);
        }
        break;
      case 'R':
        switch (serialLido[1]) {
          case 'P':
            valor_float = atof(&serialLido[3]);
            kpDireita = valor_float;
#ifdef USE_PID
            motorDireito.SetTunings(kpDireita, kiDireita, kdDireita);
#endif
            break;
          case 'I':
            valor_float = atof(&serialLido[3]);
            kiDireita = valor_float;
#ifdef USE_PID
            motorDireito.SetTunings(kpDireita, kiDireita, kdDireita);
#endif
            break;
          case 'D':
            valor_float = atof(&serialLido[3]);
            kdDireita = valor_float;
#ifdef USE_PID
            motorDireito.SetTunings(kpDireita, kiDireita, kdDireita);
#endif
            break;
          case 'S':
            valor_int = atoi(&serialLido[3]);
            velocidadeDireita = valor_int;
            ACELERA_DIREITA(velocidadeDireita);
            break;
          default:
            Serial.print("CODE UNRECOGNIZED 2 ");
            Serial.println(serialLido[1]);
        }
        break;
      case 'P':
        switch (serialLido[1]) {
          case 'E':
            pidAtivado = true;
            break;
          case 'D':
            pidAtivado = false;
            break;
          default:
            Serial.print("CODE UNRECOGNIZED 2 ");
            Serial.println(serialLido[1]);
        }
        break;
      case 'G':
        digitalWrite(MUTEX_PRINT, HIGH);
        Serial.print("P,A,X,X,");
        Serial.print(kpEsquerda);
        Serial.print(",");
        Serial.print(kiEsquerda);
        Serial.print(",");
        Serial.print(kdEsquerda);
        Serial.print(",");
        Serial.print(kpDireita);
        Serial.print(",");
        Serial.print(kiDireita);
        Serial.print(",");
        Serial.print(kdDireita);
        Serial.print(",");
        Serial.print(pwmEsquerda);
        Serial.print(",");
        Serial.println(pwmDireita);
        digitalWrite(MUTEX_PRINT, LOW);
        break;
      default:
        Serial.print("CODE UNRECOGNIZED 1 ");
        Serial.println(serialLido[0]);
    }
  }
}

void ajustaMotor() {
  if (motoresAtivados && pidAtivado) {
    entradaEsquerda = contaEsquerda - anteriorEsquerda;
#ifdef USE_PID
    motorEsquerdo.Compute();
#else
    pwmEsquerda += kpEsquerda * abs(entradaEsquerda - META) * ((entradaEsquerda > META) ? -1 : 1);
    pwmEsquerda = (pwmEsquerda < 0 ? 0 : ((pwmEsquerda > 1023) ? 1023 : pwmDireita);
#endif
    ACELERA_ESQUERDA(pwmEsquerda);

    entradaDireita = contaDireita - anteriorDireita;
#ifdef USE_PID
    motorDireito.Compute();
#else
    pwmDireita += kpDireita * abs(entradaDireita - META) * ((entradaDireita > META) ? -1 : 1);
    pwmDireita = (pwmDireita < 0 ? 0 : ((pwmDireita > 1023) ? 1023 : pwmDireita);
#endif
    ACELERA_DIREITA(pwmDireita);

    if (entradaEsquerda > entradaDireita) {
      contaEsquerda -= (entradaEsquerda - entradaDireita);
      contaDireita += (entradaEsquerda - entradaDireita);
    }
    else if (entradaDireita > entradaEsquerda) {
      contaEsquerda += (entradaDireita - entradaEsquerda);
      contaDireita -= (entradaDireita - entradaEsquerda);
    }

    anteriorDireita = contaDireita;
    anteriorEsquerda = contaEsquerda;
  }
  if (contador == DIVISOES && !digitalRead(MUTEX_PRINT)) {
    Serial.print(contaEsquerda);
    Serial.print(",");
    Serial.print(contaDireita);
    Serial.print(",");
    Serial.print(pwmEsquerda);
    Serial.print(",");
    Serial.println(pwmDireita);
    contador = 0;
    anteriorDireita = 0;
    anteriorEsquerda = 0;
    contaDireita = 0;
    contaEsquerda = 0;
  }
  contador++;
}

void contadorDireita() {
  contaDireita++;
}

void contadorEsquerda() {
  contaEsquerda++;
}
