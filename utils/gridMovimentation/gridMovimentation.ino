#include <AccelStepper.h>

/* Pinos de controle do motor da esquerda */
#define MOTOR_ESQUERDA_IN1 4
#define MOTOR_ESQUERDA_IN2 5
#define MOTOR_ESQUERDA_IN3 6
#define MOTOR_ESQUERDA_IN4 7

/* Pinos de controle do motor da direita */
#define MOTOR_DIREITA_IN1 14
#define MOTOR_DIREITA_IN2 15
#define MOTOR_DIREITA_IN3 16
#define MOTOR_DIREITA_IN4 17

/* Quantidade de celulas do grid */
#define NUM_LINHAS  4
#define NUM_COLUNAS 5
const int num_celulas = (NUM_LINHAS * NUM_COLUNAS);

/* Define tamanho de cada celula quadrada do GRID */
#define TAMANHO_GRID_MM 240

/* Constantes do robo */
#define LARGURA_ROBO_MM     120
#define COMPRIMENTO_ROBO_MM 120
#define ALTURA_ROBO_MM      100

/* Eixo das rodas */
#define CENTRO_EIXO_RODAS_X     90
#define CENTRO_EIXO_RODAS_Y     85
#define COMPRIMENTO_EIXO_RODAS  180

/* Tamanho das rodas */
#define DIAMETRO_RODAS_MM 70
#define LARGURA_RODAS_MM  25
const int raio_rodas = DIAMETRO_RODAS_MM / 2;
const int raio_eixos_motor = (COMPRIMENTO_EIXO_RODAS - 22) / 2;

/* Comprimento do movimento */
#define comprimentoArco(ANGULO, RAIO) ((ANGULO > 180) ? ((ANGULO * PI * 2 * RAIO) / 360) : ((ANGULO * PI * RAIO) / 180))
#define comprimentoArcoDecimal(ANGULO, RAIO) ((ANGULO > 180) ? ((ANGULO * PI * 2.0 * RAIO) / 360.0) : ((ANGULO * PI * RAIO) / 180.0))

/* Constantes de quantidade de movimento do motor */
#define NUMERO_PASSOS_FULL_STEP 2048
#define NUMERO_PASSOS_HALF_STEP 4096

/* Velocidade do motor em passos por segundo */
#define VELOCIDADE_MOTOR_PASSOS_SEGUNDO 1024

/* Tamanho de movimentos base */
const double step_angle = 360.0 / NUMERO_PASSOS_HALF_STEP;
const double step_length = comprimentoArco(step_angle, raio_rodas) ;

/* Movimentos aceitos */
#define FULL_STEP 4
#define HALF_STEP 8

/* Controles das rodas do robo */
AccelStepper motor_esquerda(HALF_STEP, MOTOR_ESQUERDA_IN1, MOTOR_ESQUERDA_IN3, MOTOR_ESQUERDA_IN2, MOTOR_ESQUERDA_IN4);
AccelStepper motor_direita(HALF_STEP, MOTOR_DIREITA_IN1, MOTOR_DIREITA_IN3, MOTOR_DIREITA_IN2, MOTOR_DIREITA_IN4);

/* Constante de ajuste */
#define ERROR_REPAIR 10

/* Move o robo em mm se positivo para frente negativo para trás */
void moverRobo(int distancia_mm_esquerda, int distancia_mm_direita) {
  /* Configura o movimento */
  motor_esquerda.move(ceil(distancia_mm_esquerda) / step_length);
  motor_direita.move(ceil(distancia_mm_direita) / step_length);
  Serial.println(ceil((distancia_mm_direita) / step_length));

  /* Configura a velocidade */
  motor_esquerda.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);
  motor_direita.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);

  /* Enquanto não alcançar a distância desejada */
  while (motor_esquerda.distanceToGo() != 0 && motor_direita.distanceToGo() != 0) {
    motor_esquerda.runSpeedToPosition();
    motor_direita.runSpeedToPosition();
  }
}

#define PASSOS_ROTACAO_EXATA_90   2400

/* Rotaciona robo em uma quantidade especifica de graus */
void rotacionaRobo(int graus) {
  /* Distancia a ser percorrida */
  int distancia_passos = (graus / 90) * PASSOS_ROTACAO_EXATA_90;

  /* Se maior que 0 movimento horario caso contrário anti horario */
  if (graus > 0) {
    motor_esquerda.move(distancia_passos);
    motor_direita.move(-distancia_passos);
  }
  else {
    /* Configura o movimento */
    motor_esquerda.move(-distancia_passos);
    motor_direita.move(distancia_passos);
  }

  /* Configura a velocidade */
  motor_esquerda.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);
  motor_direita.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);

  /* Enquanto não alcançar a distância desejada */
  while (motor_esquerda.distanceToGo() != 0 && motor_direita.distanceToGo() != 0) {
    motor_esquerda.runSpeedToPosition();
    motor_direita.runSpeedToPosition();
  }
}

void setup() {
  /* Inicializa a velocidade da comunicação serial */
  Serial.begin(115200);
  Serial.println("[$] Sketch teste movimentacao em grid");

  /* Configura as velocidades máximas e a velocidade desejada dos motores */
  motor_esquerda.setMaxSpeed(2000.0);
  motor_direita.setMaxSpeed(2000.0);
  motor_esquerda.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);
  motor_direita.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);

  /* Por alguma razão isso é necessário */
  motor_esquerda.move(-1);
  motor_direita.move(+1);
}

void loop() {

  delay(5000);
  rotacionaRobo(90);
  delay(5000);
  rotacionaRobo(90);
  delay(5000);
  rotacionaRobo(90);
  delay(5000);
  rotacionaRobo(90);
  delay(2000);
}
