#include <AccelStepper.h>
#include <Ultrasonic.h>
#include <QList.h>
#include <QList.cpp>
#include <limits.h>

/* Tamanho do grid */
#define NUM_LINHAS        4
#define NUM_COLUNAS       5
const int total_vertices = (NUM_LINHAS * NUM_COLUNAS);

/* Vertice Atual */
#define verticeAtual(I, J)              ((I * NUM_COLUNAS) + J)

/* Helpers para movimentação horizontal */
#define verticeHorizontal(I, J, POS)    (verticeAtual(I, J) + (1 * POS))
#define verticeHorizontalEsquerda(I, J) (verticeHorizontal(I, J, -1)
#define verticeHorizontalDireita(I, J)  (verticeHorizontal(I, J, 1)

/* Helpers para movimentação vertical */
#define verticeVertical(I, J, POS)      (verticeAtual(I, J) + (NUM_COLUNAS * POS)
#define verticeVerticalAcima(I, J)      (verticeHorizontal(I, J, -1)
#define verticeVerticalAbaixo(I, J)     (verticeHorizontal(I, J, 1)

/* Vertice de inicio ou origem */
#define INICIO_LINHA    3
#define INICIO_COLUNA   3
#define VERTICE_INICIO  (verticeAtual(INICIO_LINHA, INICIO_COLUNA))

/* Vertice de fim ou objetivo */
#define FINAL_LINHA     1
#define FINAL_COLUNA    0
#define VERTICE_FINAL   (verticeAtual(FINAL_LINHA, FINAL_COLUNA))

/* Status de visitado */
#define NAO_VISITADO    0
#define VISITANDO       1
#define VISITADO        2

/* Possiveis direções */
#define APONTADO_NORTE  0
#define APONTADO_LESTE  1
#define APONTADO_SUL    2
#define APONTADO_OESTE  3

/* Posições do mapa */
#define DES 0
#define OBJ 1
#define LIV 2
#define BAR 3
#define ORI 4

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

/* Pinos sensor ultrassonico */
#define ULTRASSONICO_TRIGGER 18
#define ULTRASSONICO_ECHO    19

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
const int raio_eixos_motor = (COMPRIMENTO_EIXO_RODAS - 5) / 2;

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

/* Estruturas de dados */

/* Vertices */
typedef struct VerticeStruct {
  int i;
  int j;
  int distancia;
  int visitado;
} Vertice;

/* Pares */
typedef struct TuplaStruct {
  int primeiro;
  int segundo;

  TuplaStruct(int _primeiro, int _segundo) {
    primeiro = _primeiro;
    segundo = _segundo;
  }
} Tupla;

/* Grafo e caminhos */
Vertice vertices[total_vertices];
int anterior[total_vertices];
QList<int> grafo[total_vertices];
QList<int> caminho;

/* Controles das rodas do robo */
AccelStepper motor_esquerda(HALF_STEP, MOTOR_ESQUERDA_IN1, MOTOR_ESQUERDA_IN3, MOTOR_ESQUERDA_IN2, MOTOR_ESQUERDA_IN4);
AccelStepper motor_direita(HALF_STEP, MOTOR_DIREITA_IN1, MOTOR_DIREITA_IN3, MOTOR_DIREITA_IN2, MOTOR_DIREITA_IN4);

/* Orientacao do robo */
int orientacao_atual;

/* Sensor ultrassonico */
Ultrasonic ultrasonico(ULTRASSONICO_TRIGGER, ULTRASSONICO_ECHO);

/* Mapa para testes */
int mapa_test[][5] = {
  { LIV, LIV, LIV, LIV, LIV },
  { OBJ, BAR, BAR, BAR, LIV },
  { BAR, LIV, LIV, LIV, LIV },
  { LIV, LIV, LIV, ORI, LIV }
};

/* Mapa Default */
int mapa_global[][5] = {
  { OBJ, DES, DES, DES, DES },
  { DES, DES, DES, DES, DES },
  { DES, DES, DES, DES, DES },
  { DES, DES, DES, ORI, DES }
};

/* Move o robo em mm se positivo para frente negativo para trás */
void moverRobo(int distancia_mm_esquerda, int distancia_mm_direita) {
  /* Configura o movimento */
  motor_esquerda.move(ceil(distancia_mm_esquerda) / step_length);
  motor_direita.move(ceil(distancia_mm_direita) / step_length);

  /* Configura a velocidade */
  motor_esquerda.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);
  motor_direita.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);

  /* Enquanto não alcançar a distância desejada */
  while (motor_esquerda.distanceToGo() != 0 && motor_direita.distanceToGo() != 0) {
    motor_esquerda.runSpeedToPosition();
    motor_direita.runSpeedToPosition();
  }
}

/* Rotaciona robo em uma quantidade especifica de graus */
void rotacionaRobo(int graus) {
  /* Distancia a ser percorrida */
  int distancia_mm = ceil(comprimentoArco(raio_eixos_motor, abs(graus)));

  /* Se maior que 0 movimento horario caso contrário anti horario */
  if (graus > 0) {
    moverRobo(distancia_mm, -distancia_mm);
  }
  else {
    moverRobo(-distancia_mm, distancia_mm);
  }
}

/* Mover uma casa para frente */
void rotacionarRobo(int graus) {
  Serial.print(F("[&] Rotacionando robo - "));
  Serial.print(graus);
  Serial.println(F("º"));
  rotacionaRobo(graus);
}

/* Mover uma casa para frente */
void moverParaFrente() {
  Serial.println(F("[-] Movendo uma casa para frente"));
  moverRobo(TAMANHO_GRID_MM, TAMANHO_GRID_MM);
}

/* Rotaciona para a direcao desejada */
void rotacionarPara(int direcao) {
  Serial.print(F("[$] Orientacao atual - Apontado "));
  switch (orientacao_atual) {
    case APONTADO_NORTE:
      Serial.println(F("Norte"));
      break;
    case APONTADO_SUL:
      Serial.println(F("Sul"));
      break;
    case APONTADO_LESTE:
      Serial.println(F("Leste"));
      break;
    case APONTADO_OESTE:
      Serial.println(F("Oeste"));
      break;
  }

  /* Se robô já está devidamente posicionado */
  if (orientacao_atual == direcao) {
    Serial.println(F("[!] Robô ja esta na orientacao desejada"));
    return;
  }

  Serial.print(F("[0] Rotacionando "));

  /* Armazena a quantidade de passos de 90 graus para a nova direção desejada */
  int qtd_rotacao = 0;
  switch (orientacao_atual) {
    case APONTADO_NORTE:
      switch (direcao) {
        case APONTADO_SUL:
          qtd_rotacao++;
        case APONTADO_LESTE:
          qtd_rotacao++;
          break;
        case APONTADO_OESTE:
          qtd_rotacao--;
      }
      break;
    case APONTADO_LESTE:
      switch (direcao) {
        case APONTADO_OESTE:
          qtd_rotacao++;
        case APONTADO_SUL:
          qtd_rotacao++;
          break;
        case APONTADO_NORTE:
          qtd_rotacao--;
      }
      break;
    case APONTADO_SUL:
      switch (direcao) {
        case APONTADO_NORTE:
          qtd_rotacao++;
        case APONTADO_OESTE:
          qtd_rotacao++;
          break;
        case APONTADO_LESTE:
          qtd_rotacao--;
      }
      break;
    case APONTADO_OESTE:
      switch (direcao) {
        case APONTADO_LESTE:
          qtd_rotacao++;
        case APONTADO_NORTE:
          qtd_rotacao++;
          break;
        case APONTADO_SUL:
          qtd_rotacao--;
      }
  }

  /* O tamanho da rotação será dado na quantidade de passos de 90 no menor sentido */
  int tamanho_rotacao = (qtd_rotacao * 90);

  /* Fim da mensagem de rotação */
  Serial.print(tamanho_rotacao);
  Serial.print(F("º para "));
  Serial.println(((tamanho_rotacao > 0) ? F("direita") : F("esquerda")));

  /* Nova orientacao atual */
  orientacao_atual = direcao;

  /* Realiza a operação mecânica de rotacionar o robo */
  rotacionarRobo(tamanho_rotacao);

  /* Mostra a nova orientação */
  Serial.print(F("[$] Nova orientacao - Apontado "));
  switch (orientacao_atual) {
    case APONTADO_NORTE:
      Serial.println(F("Norte"));
      break;
    case APONTADO_SUL:
      Serial.println(F("Sul"));
      break;
    case APONTADO_LESTE:
      Serial.println(F("Leste"));
      break;
    case APONTADO_OESTE:
      Serial.println(F("Oeste"));
      break;
  }
}

/* Move o robo uma casa para o norte */
void moverNorte() {
  /* Rotaciona o robo para a direção norte */
  Serial.println(F("[^] Movendo para o norte"));
  rotacionarPara(APONTADO_NORTE);

  /* Move uma casa */
  moverParaFrente();
}

/* Move o robo uma casa para o sul */
void moverSul() {
  /* Rotaciona o robo para a direção sul */
  Serial.println(F("[v] Movendo para o sul"));
  rotacionarPara(APONTADO_SUL);

  /* Move uma casa */
  moverParaFrente();
}

/* Move o robo uma casa para o leste */
void moverLeste() {
  /* Rotaciona o robo para a direção norte */
  Serial.println(F("[>] Movendo para o leste"));
  rotacionarPara(APONTADO_LESTE);

  /* Move uma casa */
  moverParaFrente();
}

/* Move o robo uma casa para o oeste */
void moverOeste() {
  /* Rotaciona o robo para a direção norte */
  Serial.println(F("[<] Movendo para o oeste"));
  rotacionarPara(APONTADO_OESTE);

  /* Move uma casa */
  moverParaFrente();
}

/* Move para um nó adjacente */
void moverAdjacente(int dir_i, int dir_j, bool mover) {
  /* Como não são aceitas diagonais só podemos nos mover em 4 direções N/S/L/O */
  /* Se I é diferente de 0 então estamos nos movendo na vertical */
  if (dir_i != 0) {
    /* Se o nosso I é menor que o anterior então iremos nos mover para o norte */
    if (dir_i > 0) {
      if (mover) {
        moverNorte();
      }
      else {
        rotacionarPara(APONTADO_NORTE);
      }
    }
    /* Caso o contrário iremos nos mover para o sul*/
    else {
      if (mover) {
        moverSul();
      }
      else {
        rotacionarPara(APONTADO_SUL);
      }
    }
  }
  /* Se J é diferente de 0 então estamos nos movendo na horizontal */
  else if (dir_j != 0) {
    /* Se o nosso J é maior que o anterior então iremos nos mover para o leste */
    if (dir_j < 0) {
      if (mover) {
        moverLeste();
      }
      else
        rotacionarPara(APONTADO_LESTE);
    }
    /* Caso contrário iremos nos mover para o oeste */
    else {
      if (mover) {
        moverOeste();
      }
      else {
        rotacionarPara(APONTADO_OESTE);
      }
    }
  }
}

/* Usa os sensores para detectar um obstaculo */
bool checaObstaculo() {
  Serial.println(F("[%] Checando obstaculo"));
  float cmMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  //return (ultrasonico.distanceRead() < TAMANHO_GRID_MM);
  if(cmMsec < 34){
    return true;
  }
  else{
    return false;
  }
}

/* Verifica obstaculos */
bool verificaObstaculos(int u, int v, int mapa[][NUM_COLUNAS]) {
  /* O Objetivo é verificar se um nó adjacente ao nó atual possui obstaculos
     para isso utilizaremos o mapa, que é uma matriz com o estado de cada posição
     do grid, se a posição não estiver inicializada ou seu valor for desconhecido
     inicializaremos ela com o status do sensor de verificação de obstaculo */

  /* Primeiro verificamos se a posição já está inicializada no mapa */
  if (mapa[vertices[v].i][vertices[v].j] != DES) {
    return (mapa[vertices[v].i][vertices[v].j] == BAR);
  }

  /* Se a posição é desconhecida rotacionamos o robo para a posição e usamos o sensor */
  int novo_i = vertices[u].i - vertices[v].i;
  int novo_j = vertices[u].j - vertices[v].j;

  /* Apenas rotaciona para o vertice adjacente */
  moverAdjacente(novo_i, novo_j, false);

  /* Após termos rotacionado o robo iremos usar o sensor para verificar a posição */
  bool temObstaculo = checaObstaculo();
  mapa[vertices[v].i][vertices[v].j] = ((checaObstaculo()) ? BAR : LIV );
  return temObstaculo;
}

/* Algoritmo de Dijkstra menor caminho do no atual para todos os outros nós */
void dijkstra(int mapa[][NUM_COLUNAS]) {
  QList<Tupla*> Q;

  /* Adicionamos o inicio */
  Q.push_front(new Tupla(0, VERTICE_INICIO));

  /* Posição anterior */
  int u_anterior = -1;

  /* Enquanto nossa fila de prioridade */
  while (Q.size() != 0) {
    /* Extraimos o vertice de menor distancia da nossa lista */
    int u = Q.front()->segundo;
    Q.pop_front();

    /* Imprime a posição anterior */
    if (u_anterior != -1) {
      Serial.print(F("[*] Posicao anterior | i = "));
      Serial.print(vertices[u_anterior].i);
      Serial.print(F(", j = "));
      Serial.println(vertices[u_anterior].j);
    }

    /* Imprime a posição atual */
    Serial.print(F("[@] Posicao Atual | i = "));
    Serial.print(vertices[u].i);
    Serial.print(F(", j = "));
    Serial.println(vertices[u].j);

    /* Verificamos se já acessamos alguma posição */
    if (u_anterior != -1) {

      /* Se estavamos em alguma posição na iteração anterior precisamos mover para
         a nova posição a ser atingida */
      int novo_i = vertices[u_anterior].i - vertices[u].i;
      int novo_j = vertices[u_anterior].j - vertices[u].j;

      /* Se a posição anterior não é adjacente a nova posição precisamos passar pela
         origem para atingir o no, o que vai necessitar de uma serie de passos */
      if (((novo_i != 0) && (novo_j != 0)) || ((novo_i > 1 || novo_i < -1) || (novo_j > 1 || novo_j < -1))) {
        /* Vertices não adjacentes */
        Serial.println(F("[x] Vertices nao sao adjacentes"));

        /* O vertice atual ao proximo u */
        int proximo_u = anterior[u_anterior];

        /* O proximo vertice a qual devemos alcançar */
        int atual_u = u_anterior;

        /* Percorreremos até a origem */
        while (atual_u != VERTICE_INICIO) {
          /* Armazena a nova posicao entre os dois vertices */
          novo_i = vertices[atual_u].i - vertices[proximo_u].i;
          novo_j = vertices[atual_u].j - vertices[proximo_u].j;

          /* Move para a plataforma adjacente */
          moverAdjacente(novo_i, novo_j, true);

          /* Atualiza a posicao atual como a anterior */
          atual_u = proximo_u;

          /* E a próxima posição como sendo o acesso da posição anterior a posição próxima */
          proximo_u = anterior[proximo_u];
        }

        /* Após atingido o inicio podemos agora sair do objetivo para o nosso destino */
        QList<int> caminho_obj_temp;

        /* Reconstroi o caminho do nosso objetivo atual para o inicio */
        atual_u = u;
        while (atual_u != VERTICE_INICIO) {
          caminho_obj_temp.push_front(atual_u);
          atual_u = anterior[atual_u];
        }

        /* Como estamos no inicio basta percorrer nosso objetivo temporário e teremos chegado em u */
        for (int i = 0; i < caminho_obj_temp.size(); i++) {
          /* A próxima posição é igual a posição do vetor atual */
          proximo_u = caminho_obj_temp[i];

          /* Armazena a nova posicao entre os dois vertices */
          novo_i = vertices[atual_u].i - vertices[proximo_u].i;
          novo_j = vertices[atual_u].j - vertices[proximo_u].j;

          /* Move para a plataforma adjacente */
          moverAdjacente(novo_i, novo_j, true);

          /* Atualiza a posicao atual como a anterior */
          atual_u = proximo_u;
        }
      }
      /* Se forem adjacentes basta caminhar para a próxima posição */
      else {
        /* Vertices não adjacentes */
        Serial.println(F("[y] Vertices sao adjacentes"));
        moverAdjacente(novo_i, novo_j, true);
      }
    }

    /* Para cada vizinho v de u */
    for (int vi = 0; vi < grafo[u].size(); vi++) {
      /* Vertice adjacente */
      int v = grafo[u][vi];

      /* A primeira coisa que iremos verificar é se o nosso vizinho pode ser acessado
        se ele não puder removeremos as arestas associadas a ele a partir do vertice */
      if (verificaObstaculos(u, v, mapa)) {
        /* Remove as arestas do vertice escolhido para nos e de nos para este vertice */
        Serial.print(F("[#] Removendo vertice V obstruido - i = "));
        Serial.print(vertices[v].i);
        Serial.print(F(", j = "));
        Serial.println(vertices[v].j);
        grafo[u].clear(vi);
        grafo[v].clear(grafo[v].indexOf(u));

        /* Como removemos um item de u e v, decrementamos a posição */
        vi--;

        /* Vertice não pode ser acessado próxima posição */
        continue;
      }

      /* Soma-se a distância de u com a distancia de u a v como ela é fixa temos 1 */
      int nova_distancia = vertices[u].distancia + 1;

      /* Se o novo caminho for o melhor caminho armazenamos o mesmo */
      if (nova_distancia < vertices[v].distancia) {
        vertices[v].distancia = nova_distancia;
        anterior[v] = u;

        /* Como nossa lista deve agir como uma fila de prioridades precisamos
           decidir qual a melhor posicao para a nova distancia obtida para isso
           procuraremos por um lugar o qual a nova_distancia seja <= as distancias
           presentes se for a primeira posição push_front, se for a última push_back
           se estiver no meio push_front, desloca todas as posições até a posição
           desejada e então adiciona a nova distância no local adequado */
        int pos;
        for (pos = 0; pos < Q.size() && (nova_distancia > Q[pos]->primeiro); pos++);

        /* Percorido a lista verificamos a posição */
        /* A nova distância é a menor de todas */
        if (pos == 0) {
          Q.push_front(new Tupla(nova_distancia, v));
        }
        /* A nova distância é a maior de todas */
        else if (pos == Q.size()) {
          Q.push_back(new Tupla(nova_distancia, v));
        }
        /* A nova distância possui um valor intermediario */
        else {
          /* Desloca a primeira posição para uma nova primeira posição */
          Q.push_front(Q[1]);

          /* Desloca as demais posições */
          for (int j = 1; j < pos; j++) {
            Q[j] = Q[j + 1];
          }

          /* Armazena o novo item na posição adequada dele */
          Q[pos] = new Tupla(nova_distancia, v);
        }
      }
    }

    /* Armazena a última posição acessada do grid no caso o valor de u atual */
    u_anterior = u;
  }
}

void setup() {
  /* Inicia a comunicação */
  Serial.begin(115200);

  /* Inicializando Dijkstra */
  Serial.println(F("Arduino Dijkstra V1.0\n"));

  /* Configura as velocidades máximas e a velocidade desejada dos motores */
  motor_esquerda.setMaxSpeed(2000.0);
  motor_direita.setMaxSpeed(2000.0);
  motor_esquerda.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);
  motor_direita.setSpeed(VELOCIDADE_MOTOR_PASSOS_SEGUNDO);

  /* Por alguma razão isso é necessário */
  motor_esquerda.move(-1);
  motor_direita.move(+1);

  /* Armazena o tempo de execução */
  long tempo_execucao = millis();

  /* Inicializa os vertices e o grafo do mapa */
  for (int i = 0; i < NUM_LINHAS; i++) {
    for (int j = 0; j < NUM_COLUNAS; j++) {
      int vertice_atual = verticeAtual(i, j);

      /* Inicializa o vertice */
      vertices[vertice_atual].i = i;
      vertices[vertice_atual].j = j;
      vertices[vertice_atual].distancia = INT_MAX;
      vertices[vertice_atual].visitado = NAO_VISITADO;

      /* Inicializa as arestas do vertice */
      if ((i + 1) != NUM_LINHAS) {
        grafo[vertice_atual].push_back(vertice_atual + NUM_COLUNAS);
        grafo[vertice_atual + NUM_COLUNAS].push_back(vertice_atual);
      }
      if ((j + 1) != NUM_COLUNAS) {
        grafo[vertice_atual].push_back(vertice_atual + 1);
        grafo[vertice_atual + 1].push_back(vertice_atual);
      }
    }
  }

  /* Orientacao_atual */
  orientacao_atual = APONTADO_NORTE;

  /* Inicializa a distância do primeiro vértice como 0 */
  vertices[VERTICE_INICIO].distancia = 0;

  /* Procura o menor caminho partindo da origem até todos os vértices */
  //dijkstra(mapa_test);
  dijkstra(mapa_global);

  /* Com o menor caminho traçado reconstruimos o melhor caminho até o objetivo */
  int vertice_atual = VERTICE_FINAL;
  while (vertice_atual != VERTICE_INICIO) {
    caminho.push_front(vertice_atual);
    vertice_atual = anterior[vertice_atual];
  }

  /* Adiciona o vertice inicial */
  caminho.push_front(vertice_atual);

  /* Imprime o caminho do melhor caminho */
  Serial.println(F("\nCaminho Final:\n"));
  for (int i = 0; i < caminho.size(); i++) {
    Serial.print(F("Vertice #"));
    Serial.print(caminho[i]);
    Serial.print(F(" Cord. i = "));
    Serial.print(vertices[caminho[i]].i);
    Serial.print(F(" j = "));
    Serial.println(vertices[caminho[i]].j);
  }

  /* Tempo de execucao final */
  tempo_execucao = millis() - tempo_execucao;

  /* Fim da obtenção do menor caminho */
  Serial.println(F("\n[e] Fim da obtenção do menor caminho"));
  Serial.print(F("[i] Tamanho do menor caminho = "));
  Serial.println(caminho.size() - 1);
  Serial.print(F("[t] Menor caminho encontrado em "));
  Serial.print(tempo_execucao / 1000.0);
  Serial.println(F("s"));
  delay(30000);
}

/* Percorre o menor caminho enquanto estiver ligado */
void loop() {

  /* Mensagem de inicio do laço */
  Serial.println(F("\n[s] Percorrendo o menor caminho!\n"));

  /* Nosso primeiro vertice é sempre o ínicio */
  int atual_u = VERTICE_INICIO;

  /* Percorremos o menor caminho */
  for (int i = 1; i < caminho.size(); i++) {
    /* A próxima posição é igual a posição do vetor atual */
    int proximo_u = caminho[i];

    /* Imprime a posicao atual */
    Serial.print(F("[*] Posicao atual | i = "));
    Serial.print(vertices[atual_u].i);
    Serial.print(F(", j = "));
    Serial.println(vertices[atual_u].j);

    /* Imprime a proxima posição */
    Serial.print(F("[@] Proxima posicao | i = "));
    Serial.print(vertices[proximo_u].i);
    Serial.print(F(", j = "));
    Serial.println(vertices[proximo_u].j);

    /* Armazena a nova posicao entre os dois vertices */
    int novo_i = vertices[atual_u].i - vertices[proximo_u].i;
    int novo_j = vertices[atual_u].j - vertices[proximo_u].j;

    /* Move para a plataforma adjacente */
    moverAdjacente(novo_i, novo_j, true);

    /* Atualiza a posicao atual como a anterior */
    atual_u = proximo_u;
  }

  /* Mensagem de fim */
  Serial.println(F("\n[f] Fim da execucao do menor caminho\n"));

  /* Espera 20 segundos e percorre o caminho novamente */
  delay(20000);
}
