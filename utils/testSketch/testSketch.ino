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

volatile double kpDireita = 0;
volatile double kpEsquerda = 0;

//volatile int velocidadeDireita = 120;
//volatile int velocidadeEsquerda = 130;
volatile int velocidadeDireita = 800;
volatile int velocidadeEsquerda = 870;

volatile double pwmDireita = 0;
volatile double pwmEsquerda = 0;

volatile bool motoresAtivados = false;

volatile int contador = 0;

volatile int girosDesejados = 150;

//Define Variables we'll be connecting to
double entradaEsquerda, entradaDireita;
double objetivo = 80;

//Specify the links and initial tuning parameters
PID motorEsquerdo(&entradaEsquerda, &pwmEsquerda, &objetivo,kpEsquerda,0,0, DIRECT);

//Specify the links and initial tuning parameters
PID motorDireito(&entradaDireita, &pwmDireita, &objetivo,kpDireita,0,0, DIRECT);

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

  pinMode(INTERROMPE_DIREITA, INPUT_PULLUP);
  pinMode(INTERROMPE_ESQUERDA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INTERROMPE_DIREITA), contadorDireita, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INTERROMPE_ESQUERDA), contadorEsquerda, CHANGE);

  Timer1.initialize(1000000 / 10);
  Timer1.attachInterrupt(ajustaMotor);

  motorEsquerdo.SetMode(AUTOMATIC);
  motorDireito.SetMode(AUTOMATIC);
  motorEsquerdo.SetOutputLimits(0, 1023); 
  motorDireito.SetSampleTime(1000/10); 
  motorEsquerdo.SetSampleTime(1000/10); 
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
        contaDireita = 0;
        contaEsquerda = 0;
        ACELERA_DIREITA(velocidadeDireita);
        ACELERA_ESQUERDA(velocidadeEsquerda);
        IR_PARA_FRENTE();
        break;
      case 'S':
        motoresAtivados = false;
        FREIO();
        break;
      case 'L':
        switch (serialLido[1]) {
          case 'P':
            valor_float = atof(&serialLido[3]);
            kpEsquerda = valor_float;
            motorEsquerdo.SetTunings(kpEsquerda, 0, 0);
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
            motorDireito.SetTunings(kpDireita, 0, 0);
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
      default:
        Serial.print("CODE UNRECOGNIZED 1 ");
        Serial.println(serialLido[0]);
    }
  }
}

void ajustaMotor() {
  if (motoresAtivados) {
    //pwmEsquerda += kpEsquerda * (contaEsquerda - (girosDesejados / 10));
    entradaEsquerda = contaEsquerda;
    motorEsquerdo.Compute();
    ACELERA_ESQUERDA(pwmEsquerda);
    //pwmDireita += kpDireita * (contaDireita - (girosDesejados / 10));
    entradaDireita = contaDireita;
    motorDireito.Compute();
    ACELERA_DIREITA(pwmDireita);
  }
  if (contador == 10) {
    contador = 0;
    Serial.print(contaEsquerda);
    Serial.print(",");
    Serial.println(contaDireita);
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

