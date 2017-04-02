#include <TimerOne.h>

#define INTERROMPE_DIREITA 3
#define INTERROMPE_ESQUERDA 2

#define ACELERADOR_DIREITA 6
#define ACELERADOR_ESQUERDA 5

#define LUZ_ACELERADOR_DIREITA 12
#define LUZ_ACELERADOR_ESQUERDA 13

#define DIRECAO_DIREITA_1 7
#define DIRECAO_DIREITA_2 8

#define DIRECAO_ESQUERDA_1 10
#define DIRECAO_ESQUERDA_2 9

#define IR_PARA_FRENTE_DIREITA() do { digitalWrite(DIRECAO_DIREITA_1, HIGH); digitalWrite(DIRECAO_DIREITA_2, LOW); } while(false)
#define IR_PARA_FRENTE_ESQUERDA() do { digitalWrite(DIRECAO_ESQUERDA_1, HIGH); digitalWrite(DIRECAO_ESQUERDA_2, LOW); } while(false)
#define IR_PARA_FRENTE() do { IR_PARA_FRENTE_DIREITA(); IR_PARA_FRENTE_ESQUERDA(); } while(false)

#define IR_PARA_TRAS_DIREITA() do { digitalWrite(DIRECAO_DIREITA_1, LOW); digitalWrite(DIRECAO_DIREITA_2, HIGH); } while(false)
#define IR_PARA_TRAS_ESQUERDA() do { digitalWrite(DIRECAO_ESQUERDA_1, LOW); digitalWrite(DIRECAO_ESQUERDA_2, HIGH); } while(false)
#define IR_PARA_TRAS() do { IR_PARA_TRAS_DIREITA(); IR_PARA_TRAS_ESQUERDA(); } while(false)

#define ACELERA_DIREITA(VELOCIDADE) do { pwmDireita = VELOCIDADE; analogWrite(ACELERADOR_DIREITA, VELOCIDADE); } while(false)
#define ACELERA_ESQUERDA(VELOCIDADE) do { pwmEsquerda = VELOCIDADE; analogWrite(ACELERADOR_ESQUERDA, VELOCIDADE); } while(false)
#define ACELERA(VELOCIDADE) do { ACELERA_DIREITA(VELOCIDADE); ACELERA_ESQUERDA(VELOCIDADE); } while(false)

#define FREIO_DIREITA() { ACELERA_DIREITA(0); digitalWrite(DIRECAO_DIREITA_1, LOW); digitalWrite(DIRECAO_DIREITA_2, LOW); } while(false)
#define FREIO_ESQUERDA() { ACELERA_ESQUERDA(0); digitalWrite(DIRECAO_ESQUERDA_1, LOW); digitalWrite(DIRECAO_ESQUERDA_2, LOW); } while(false)
#define FREIO() do { FREIO_DIREITA(); FREIO_ESQUERDA(); } while(false)

volatile int contaDireita = 0;
volatile int contaEsquerda = 0;

volatile double kpDireita = 1.2;
volatile double kpEsquerda = 1.2;

volatile int velocidadeDireita = 65;
volatile int velocidadeEsquerda = 66;

volatile int pwmDireita = 0;
volatile int pwmEsquerda = 0;

volatile bool motoresAtivados = false;

void setup() {
  Serial.begin(9600);
  
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
}

void loop() {
  contaDireita = 0;
  contaEsquerda = 0;
  ACELERA_DIREITA(velocidadeDireita);
  ACELERA_ESQUERDA(velocidadeEsquerda);
  IR_PARA_FRENTE();
  motoresAtivados = true;
  delay(1000);
  motoresAtivados = false;
  Serial.print(contaDireita);
  Serial.print(",");
  Serial.println(contaEsquerda);
  FREIO();
  delay(1500);
}

void ajustaMotor(){
  if(motoresAtivados){
    if(contaDireita > contaEsquerda){
      pwmEsquerda += kpEsquerda * (contaDireita - contaEsquerda);
    }
    else if(contaEsquerda > contaDireita){
      pwmDireita += kpDireita * (contaEsquerda - contaDireita);
    }
    
    //ACELERA_DIREITA(pwmDireita);
    //ACELERA_ESQUERDA(pwmEsquerda);
    velocidadeDireita += pwmDireita;
    velocidadeEsquerda += pwmEsquerda;
  }
}

void contadorDireita(){
  contaDireita++;
}

void contadorEsquerda(){
  contaEsquerda++;
}

