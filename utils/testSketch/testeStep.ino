/* Bibliotecas de Controle de Motores de passo */
/* #include <Stepper.h> */
/* #include <AccelStepper.h> */
#include <CustomStepper.h> /* http://playground.arduino.cc/uploads/Main/CustomStepper.zip */

/* Define pinos */
#define DIRECAO_DIREITA_1 8
#define DIRECAO_DIREITA_2 9

#define DIRECAO_ESQUERDA_1 10
#define DIRECAO_ESQUERDA_2 11

/* Define váriaveis do motor */
#define NUMERO_PASSOS 4075.7728395
#define VELOCIDADE_RPM 12
#define ANGULO_GIRO 90

/* Define os parametros iniciais de ligacao do motor de passo */
CustomStepper stepper(DIRECAO_DIREITA_1, DIRECAO_DIREITA_2, DIRECAO_ESQUERDA_1, DIRECAO_ESQUERDA_2, (byte[]) {
  8, B1000, B1100, B0100,
  B0110, B0010, B0011, B0001, B1001
}, NUMERO_PASSOS, VELOCIDADE_RPM, CW);

/* Funcoes */

#define IR_PARA_FRENTE() do { stepper.setDirection(CW); stepper.setRPM(VELOCIDADE_RPM); } while(false)
#define IR_PARA_TRAS() do { stepper.setDirection(CCW); stepper.setRPM(VELOCIDADE_RPM); } while(false)
#define GIRAR_ESQUERDA() do { stepper.setDirection(CCW);  stepper.rotateDegrees(ANGULO_GIRO); } while(false)
#define GIRAR_DIREITA() do { stepper.setDirection(CW);  stepper.rotateDegrees(ANGULO_GIRO); } while(false)
#define FREIO() do { stepper.setDirection(STOP); } while(false)

void setup() {
  serial.begin(115200);

  pinMode(DIRECAO_DIREITA_1, OUTPUT);
  pinMode(DIRECAO_DIREITA_2, OUTPUT);
  pinMode(DIRECAO_ESQUERDA_1, OUTPUT);
  pinMode(DIRECAO_ESQUERDA_2, OUTPUT);

}

void loop() {
  GIRAR_DIREITA()
  stepper.run();
}

