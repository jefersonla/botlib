
/*Bilbiotecas*/
#include <Ultrasonic.h>
#include <limits.h>

/* Tamanho do grid */
#define NUM_LINHAS        3
#define NUM_COLUNAS       3

/* Tamanho da lista que guardará o caminho percorrido */
#define a   (NUM_LINHAS * 15)

/* Tamanho do grid para auxiliar o A* */
#define m   (NUM_LINHAS + 2)

int grid[m][m];
int heuristic[NUM_LINHAS][NUM_COLUNAS];


/* Váriaveis para sinalizar fim do programa */
int fim = 0;
int counter = 0;

/* Define objetivo*/
int GoalRow = 1;
int GoalCol = 3;

/* Define posição atual */
int ActualRow;
int ActualCol;

/* Define próxima posição */
int NextRow;
int NextCol;

/* Define posição inicial */
int StartRow = 3;
int StartCol = 1;

/* Váriaveis utilizadas na função de custo */

/* Configura custo em andar pelas ortogonais */
int ort = 10;
/* Configura custo em andar pelas diagonais */
int obstacle = 1000;

/* Variáveis auxiliares */
int i;
int j;
int z;
int p;

/* Guarda os indices das linhas percorridas no grid */
int pathRow[a];
/* Guarda os indices das colunas percorridas no grid */
int pathCol[a];

/* Mesma função, mas só armazena o caminho final, auxilia o robô na parte de reexeutar o caminho */
int finalRow[m];
int finalCol[m];

/* Função usada no teste, para finalizar o programa após uma execução */
void intervencao() {
  for (i = 0; i < m; i++) {
    for (j = 0; j < m; j++) {
      Serial.println(grid[i][j]);
      fim = 1;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Estou no Setup!");

  /* Inicializa minha matriz com as distâncias de Manhattan, função heurística */
  heuristicFunction();
  ActualRow = StartRow;
  ActualCol = StartCol;
  /* O caminho obviamente inicia pelo ponto inicial */
  pathRow[0] = StartRow;
  pathCol[0] = StartCol;
  z = 1;

  /* Setando todo o grid para 1000, isso irá auxiliar no momento da execução do A* */
  for (i = 0; i < m; i++) {
    for (j = 0; j < m; j++) {
      grid[i][j] = 1000;
    }
  }

  /* Local das funções de ajustes de rodas */
  //calibratemotors()
}

void loop() {
  Serial.println("Inicio do loop!");
  delay(1000);
  if (!fim) {
    /* Giro o robo, enquanto calculo o A* para cada quadrado observado */
    a_star();
    delay(1000);
    /* Encontra o menor valor no Grid para definir próxima posição */
    findPath();
    delay(1000);
    /* Vai para a próxima posição */
    goAhead();
    delay(1000);
    /* Imprime grid */
    printgrid();
    /* Imprime matriz heurística */
    printheuristic();
    delay(1000);
    /* Caso seja o fim, finalize e imprima o resultado da computaçao */
    if (isFim()) {
      /* Imprime caminho final */
      defineWay();
      delay(10000);
    }
    delay(2000);
    /* Descomente a linha a seguir para parar o código após uma interação */
    /* intervencao(); */
  }

  else {
    /* Robo reexecuta o movimento correto ao apertar o botão */
    /* TODO */
  }
  Serial.println("Fim do loop!");
}

/* Printa o caminho percorrido*/
void printpath() {
  Serial.println("Este eh ao caminho percorrido ate aqui:");
  for (i = 0; i < z; i++) {
    Serial.print(pathRow[i]);
    Serial.print(" ");
    Serial.println(pathCol[i]);
  }
}

/* Printa a matriz Heuristica*/
void printheuristic() {
  Serial.println("Este eh a heuristica do labirinto, resolvido por Manhanttan:");
  for (i = 0; i < NUM_LINHAS; i++) {
    for (j = 0; j < NUM_LINHAS; j++) {
      Serial.print(heuristic[i][j]);
      Serial.print("  ");
    }
    Serial.print("\n");
  }
}

/* Printa o grid */
void printgrid() {
  Serial.println("Este eh o grid do labirinto, resolvido por A*:");
  for (i = 0; i < m; i++) {
    for (j = 0; j < m; j++) {
      Serial.print(grid[i][j]);
      Serial.print("  ");
    }
    Serial.print("\n");
  }
}

/* Verifica se é o fim */
boolean isFim() {
  Serial.println("Verificando se eh o fim...");
  //This funcion will see if we reach the goal
  //if ActualPoint is equal GoalPoint
  if (ActualRow == GoalRow && ActualCol == GoalCol) {
    fim = 1;
    return true;
  }
  else {
    return false;
  }
}


/* Detecta obstaculo */

boolean isObstacle(int x, int y) {
  Serial.println("Verificando obstaculos...");
  /* Barreiras virtualmente colocadas */
  if (x == 0 or y == 0 or x == 4 or y == 4) {
    return true;
  }
  else {
    return false;
  }
}

/* Movimentos do robo */

/* Move para a frente */
void goForward() {
  Serial.println("Movendo para frente...");
  /* TODO */
}

/* Move para trás */
void goBackward() {
  Serial.println("Movendo para tras...");
  /* TODO */
}

/* Move para esquerda */
void turnLeft() {
  Serial.println("Movendo para esquerda...");
  /* TODO */
}

/* Move para direita */
void turnRight() {
  Serial.println("Movendo para direita...");
  /* TODO */
}

/* Gira o robô para a direita */
void turnAround() {
  Serial.println("Girando...");
  /* TODO */
}


/* Funções do A Estrela */

int aux1;
int aux2;

/* Funcao Heuristica calculada com a distância de Manhanttan */
void heuristicFunction() {
  Serial.println("Calculando matriz heuristica...");
  for (i = 0; i < NUM_LINHAS; i++) {
    for (j = 0; j < NUM_LINHAS; j++) {
      aux1 = abs(i - (GoalRow - 1));
      aux2 = abs(j - (GoalCol - 1));
      aux1 = aux1 + aux2;
      heuristic[i][j] = aux1;
      /*
        Serial.print("heuristic[");
        Serial.print(i);
        Serial.print("][");
        Serial.print(j);
        Serial.print("] = ");
        Serial.println(heuristic[i][j]);
      */
    }
  }
}

/* Funcao A Estrela (Junta funcao Heuristica com as variáveis de Custo) */
void a_star() {
  Serial.println("Aplicando A* em meus vizinhos...");

  /* Para evitar recalculos, seta o quadrado atual como 2000*/
  grid[ActualRow][ActualCol] = 2000;

  /* Calculo o A Estrela do quadrado da frente */
  setGrid(ActualRow - 1, ActualCol);
  /* Depois giro para a direita */
  turnAround();
  /* Calculo o A Estrela do quadrado da direita */
  setGrid(ActualRow, ActualCol + 1);
  /* Depois giro para a direita */
  turnAround();
  /* Calculo o A Estrela do quadrado de trás */
  setGrid(ActualRow + 1, ActualCol);
  /* Depois giro para a direita */
  turnAround();
  /* Calculo o A Estrela do quadrado da esquerda */
  setGrid(ActualRow, ActualCol - 1);
  /* Depois giro para a direita, para voltar a orientação inicial */
  turnAround();
}

/* Percorro minha matriz Grid em busca do menor valor, o primeiro menor valor será minha próxima posição */
void findPath() {
  Serial.println("Definindo rota...");
  p = z - 1;
  int padrao = 1000;
  for (i = 1; i < m; i++) {
    for (j = 1; j < m; j++) {
      if (grid[i][j] < padrao) {
        NextRow = i;
        NextCol = j;
        /*Serial.print(NextRow);
          Serial.println(NextCol);*/
        padrao = grid[i][j];
      }
    }
  }
}

/* Define o novo valor do quadrado observado (até então, desconhecido) */
void setGrid(int x, int y) {
  Serial.println("Calculando heuristica mais custo...");
  int q = x;
  int r = y;
  /* Se for obstaculo, valor 1000 */
  if (isObstacle(q, r)) {
    if (q >= 0 && r >= 0) {
      grid[x][y] = obstacle;
      return;
    }
  }
  /* Se já foi calculado, ignora */
  if (grid[x][y] == 2000) { //(grid[x][y] == 2000)
    return;
  }
  /* Calculo custo do caminho até chegar mais a heuritica */
  else {
    grid[x][y] = 0;
    grid[x][y] = 10 + heuristic[x - 1][y - 1];
    Serial.print("Este eh meu indice Z: ");
    Serial.println(z);
    p = z - 1;
    int lock2 = 1;
    printpath();
    while (lock2) {
      if (pathRow[p] == StartRow && pathCol[p] == StartCol) {
        lock2 = 0;
      }
      else {
        /*Serial.print(pathRow[p]);
          Serial.print("  ");
          Serial.println(pathCol[p]);*/
        grid[x][y] += heuristic[pathRow[p] - 1][pathCol[p] - 1];
        p -= 1;
      }
    }
  }
}

/* Encontra um caminho até a próxima posição */
void goAhead() {
  Serial.println("Indo para proxima posicao...");
  Serial.print("Linha: ");
  Serial.print(NextRow);
  Serial.print(" Coluna: ");
  Serial.println(NextCol);
  //Verifico se próxima posição é um vizinho
  /* Se for,  eles estão na mesma linha, devo ir para direita ou esquerda */
  if (ActualRow == NextRow) {
    while (ActualCol != NextCol) {
      if (ActualCol < NextCol) {
        turnRight();
        ActualRow = NextRow;
        ActualCol += 1;
      }
      else {
        turnLeft();
        ActualRow = NextRow;
        ActualCol -= 1;
      }
      pathRow[z] = ActualRow;
      pathCol[z] = ActualCol;
      z += 1;
    }
    return;
  }
  /* Se for,  eles estão na mesma linha, devo ir para frente ou trás */
  if (ActualCol == NextCol) {
    while (ActualRow != NextRow) {
      if (ActualRow < NextRow) {
        goBackward();
        ActualRow += 1;
        ActualCol = NextCol;
      }
      else {
        goForward();
        ActualRow -= 1;
        ActualCol = NextCol;
      }
      pathRow[z] = ActualRow;
      pathCol[z] = ActualCol;
      z += 1;
    }
    return;
  }
  /* Caso não seja vizinho, devo regredir no grid, retrocendendo na lista percorrida até alcançar um vizinho */
  Serial.println("Nao e vizinho.............................");
  p -= 1;
  /* Se for,  eles estão na mesma linha, devo ir para direita ou esquerda */
  if (ActualRow == pathRow[p]) {
    if (ActualCol < pathCol[p]) {
      turnRight();
    }
    else {
      turnLeft();
    }
    ActualRow = pathRow[p];
    ActualCol = pathCol[p];
    pathRow[z] = ActualRow;
    pathCol[z] = ActualCol;
    z += 1;
    goAhead();
  }
  /* Se for,  eles estão na mesma linha, devo ir para frente ou trás */
  if (ActualCol == pathCol[p]) {
    if (ActualRow < pathRow[p]) {
      goBackward();
    }
    else {
      goForward();
    }
    ActualRow = pathRow[p];
    ActualCol = pathCol[p];
    pathRow[z] = ActualRow;
    pathCol[z] = ActualCol;
    z += 1;
    goAhead();
  }
  goAhead();
}


void defineWay() {
  p = z;
  int lock = 1;
  Serial.println("Imprimindo caminho final...");
  /* Retrocede o indice da lista até encontrar a posição inicial */
  while (lock) {
    if (pathRow[p] == StartRow && pathCol[p] == StartCol) {
      lock = 0;
    }
    else {
      counter++;
      p--;
    }
  }
  /* Imprime caminho final */
  for (i = 0; i < counter; i++) {
    finalRow[i] = pathRow[p];
    finalCol[i] = pathCol[p];
    Serial.print("Passo ");
    Serial.print(i + 1);
    Serial.print(" ");
    Serial.print(pathRow[p]);
    Serial.print(" ");
    Serial.println(pathCol[p]);
    p++;
  }
}
