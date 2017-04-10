/*Libraries*/
//#include <Ultrasonic.h>

/* Define Matrix Grid */

#define n 3
#define a   (n * 15)
#define m   (n + 2)

int grid[m][m];
int heuristic[n][n];

/*Inserir uma interface gráfica para visualização do A-Star */
int processing[n][n];
int numerador = 0;
int iD;

/* MAKE IT STOP */
int fim = 0;           
int counter = 0;       

/* Define Goal */
int GoalRow = 1;
int GoalCol = 3;

/* Define Actual Point */
int ActualRow;
int ActualCol;

/* Define Next Point */
int NextRow;
int NextCol;


/* Define StartPoint */
int StartRow = 3;
int StartCol = 1;

/* Function g(n) - Cost */
int ort = 10;      // Define ortogonal's(up, down, left, right) cost as 10
int obstacle = 1000; //Define obstacle

/* Variables */
int i;
int j;
int z;//Indice utilizado nos vetores path
int p;
int pathRow[a];
int pathCol[a];
int finalRow[m];
int finalCol[m];

void intervencao(){
  for(i = 0; i < m; i++){
      for(j = 0; j < m; j++){
         //serial.println(grid[i][j]);
         fim = 1;
      }
   }
}

void setup() {
   Serial.begin(9600);
   //serial.println("Estou no Setup!");
   heuristicFunction();
   ActualRow = StartRow;
   ActualCol = StartCol;
   
   //O caminho obviamente inicia pelo ponto inicial
   pathRow[0] = StartRow;
   pathCol[0] = StartCol;
   z = 1;
   // Setando todo o grid para 1000, isso irá auxiliar no momento da execução do A*
   for(i = 0; i < m; i++){
      for(j = 0; j < m; j++){
         grid[i][j] = 1000;
      }
   }
   grid[1][2] = 16;
   
   for(i = 0; i < n; i++){
     for(j = 0; j < n; j++){
      processing[i][j] = numerador;
      numerador ++;
   }  
  }

  iD = processing[StartRow - 1][StartCol - 1];
  Serial.write(iD);
  
  //calibratemotors()
}

void loop() {
  //serial.println("Inicio do loop!");
  delay(1000);
  if(!fim){
    a_star();
    delay(1000);
    findPath();
    delay(1000);
    goAhead();
    delay(1000);
    printgrid();
    printheuristic();
    delay(1000);
    if(isFim()){
       defineWay();
       delay(10000);
    }
    delay(2000);
    //intervencao();
  }
  
  else{
    //Don't do nothing...
  }
  //serial.println("Fim do loop!");
}
void printpath(){
  //serial.println("Este eh ao caminho percorrido ate aqui:");
  for(i = 0; i < z; i++){
    //serial.print(pathRow[i]);
    //serial.print(" ");
    //serial.println(pathCol[i]);
  }
}

void printheuristic(){
  //serial.println("Este eh a heuristica do labirinto, resolvido por Manhanttan:");
  for(i = 0; i < n; i++){
    for(j = 0; j < n; j++){
      //serial.print(heuristic[i][j]);
      //serial.print("  ");
    }
    //serial.print("\n");
  }  
}

void printgrid(){
  //serial.println("Este eh o grid do labirinto, resolvido por A*:");
  for(i = 0; i < m; i++){
    for(j = 0; j < m; j++){
      //serial.print(grid[i][j]);
      //serial.print("  ");
    }
    //serial.print("\n");
  }  
}

boolean isFim(){
  //serial.println("Verificando se eh o fim...");
  //This funcion will see if we reach the goal
  //if ActualPoint is equal GoalPoint
  if(ActualRow == GoalRow && ActualCol == GoalCol){
    fim = 1;
    return true; 
  }
  else{
    return false;
  }
}

/* PD Functions - Make the Robot drive straight*/

/* Obstacles Detector Functions */

boolean isObstacle(int x, int y){
  //serial.println("Verificando obstaculos...");
  //This funcion will see if there is a object on a point in the matrix that is a obstacles
  if (x == 0 or y == 0 or x == 4 or y == 4){
    return true;
  }
  else{
    return false;
  }
}

/* Robot Moviments Functions */

//This follows moviments has as function, besides move the robot aroud the grid, make the robo orientation be always to the North, this can make it easier to move the robot 
void goForward(){
  //serial.println("Movendo para frente...");
  //Move robot forward
}

void goBackward(){
  //serial.println("Movendo para tras...");
  //Move robot backward
}

void turnLeft(){
  //serial.println("Movendo para esquerda...");
  //Turn robot left *doesn't mean change matriz position, move robot 90º on his atual position
  //goForward()     *go to other position on the grid
  //Turn robot right
}

void turnRight(){
  //serial.println("Movendo para direita...");
  //Turn robot right 
  //goForward()     
  //Turn robot left
}

//Just change the robot orientation
void turnAround(){
  //serial.println("Girando...");
  //Turn robot right
}


/* A Star Functions */

int aux1;
int aux2;

/* Function h(n) - Define Heuristic using Manhattan Distance*/
void heuristicFunction(){
  //serial.println("Calculando matriz heuristica...");
  for(i = 0; i < n; i++){
    for(j = 0; j < n; j++){
      aux1 = abs(i - (GoalRow-1));
      aux2 = abs(j - (GoalCol-1));
      aux1 = aux1 + aux2;
      heuristic[i][j] = aux1;
      
      //serial.print("heuristic[");
      //serial.print(i);
      //serial.print("][");
      //serial.print(j);
      //serial.print("] = ");
      //serial.println(heuristic[i][j]);
      
    }
  }  
}

/* Function f(n) = g(n) + h(n) */
void a_star(){
  //serial.println("Aplicando A* em meus vizinhos...");
  /*
  int StartRow = 3;
  int StartCol = 1;
  I need o check all around, but skip previous calculate...
  */
  //Seta o grid como 2000 para evitar loop infinito
  grid[ActualRow][ActualCol] = 2000;
   
  //Fist test forward (forward is -1 of the ActualPosition Row)
  setGrid(ActualRow-1,ActualCol);
  //Then test right (right is +1 of the ActualPosition Col)
  turnAround();
  setGrid(ActualRow,ActualCol+1);
  //Then test backward (backward is +1 of the ActualPosition Row)
  turnAround();
  setGrid(ActualRow+1,ActualCol);
  //Then test left (left is -1 of the ActualPosition Col),
  turnAround();
  setGrid(ActualRow,ActualCol-1);
  //Finally turn back to Origin orientation
  turnAround();
}

void findPath(){
  //serial.println("Definindo rota...");
  p = z - 1;
  int padrao = 1000;
  for(i = 1; i < m; i++){
    for(j = 1; j < m; j++){
       if(grid[i][j] < padrao){
        NextRow = i;
        NextCol = j;
        //serial.print(NextRow);
        //serial.println(NextCol);
        padrao = grid[i][j];
       }
    }
  }
}

void setGrid(int x, int y){
  //serial.println("Calculando heuristica mais custo...");
  int q = x;
  int r = y;
  if (isObstacle(q, r)){
    if (q >= 0 && r >= 0){
      grid[x][y] = obstacle;
      return;
    }
  } 
  if (grid[x][y] == 2000){ //(grid[x][y] == 2000)
      return;
  }
  if (grid[x][y] == 16){ //(grid[x][y] == 2000)
      return;
  }
  else{
      grid[x][y] = 0;
      grid[x][y] = 10 + heuristic[x-1][y-1];
      // NAO
      //int w = z;
      //while(pathRow[w] != StartRow && pathCol[w] != StartCol){
     //   grid[x][y] += heuristic[pathRow[w-1]][pathCol[w-1]];
      //  w -= 1;
     // }
     // NAO
     //serial.print("Este eh meu indice Z: ");
     //serial.println(z);
     p = z-1;
     int lock2 = 1;
     printpath();
     while(lock2){
      if(pathRow[p] == StartRow && pathCol[p] == StartCol){
        lock2 = 0;  
      }
      else{  
        //serial.print(pathRow[p]);
        //serial.print("  ");
        //serial.println(pathCol[p]);
        grid[x][y] += heuristic[pathRow[p]-1][pathCol[p]-1];
        p-=1;
      }  
    }
  }
}

void goAhead(){
  //serial.println("Indo para proxima posicao...");
  //serial.print("Linha: ");
  //serial.print(NextRow);
  //serial.print(" Coluna: ");
  //serial.println(NextCol);
  //É necessario se posicionar neste novo grid
  //Verifico se próxima posição é um vizinho
  if(ActualRow == NextRow){  //Se for,  eles estão na mesma linha, devo left ou right
    while(ActualCol != NextCol){   
      if(ActualCol < NextCol){
        turnRight();
        ActualRow = NextRow;
        ActualCol +=1;
      }
      else{
        turnLeft(); 
        ActualRow = NextRow;
        ActualCol -= 1;
      }
      iD = processing[ActualRow - 1][ActualCol - 1];
      Serial.write(iD);
      //serial.write("\n");
      pathRow[z] = ActualRow;
      pathCol[z] = ActualCol;
      z += 1;
    }  
    return;
  }
  if(ActualCol == NextCol){  //Se for, eles estão na mesma coluna, devo forward ou backward
     while(ActualRow != NextRow){ 
      if(ActualRow < NextRow){// turn right
        goBackward();
        ActualRow += 1;
        ActualCol = NextCol;
      }
      else{
        goForward();
        ActualRow -= 1;
        ActualCol = NextCol;
      }
      iD = processing[ActualRow - 1][ActualCol - 1];
      Serial.write(iD);
      //serial.write("\n");
      pathRow[z] = ActualRow;
      pathCol[z] = ActualCol;
      z += 1;
    } 
    return;
  }
  //Caso não seja vizinho, devo regredir no grid, retrocendendo na lista percorrida até alcançar um vizinho,
  //serial.println("Nao e vizinho.............................");
  p -= 1;
  if(ActualRow == pathRow[p]){ //Se for,  eles estão na mesma linha, devo left ou right
     if(ActualCol < pathCol[p]){
      turnRight();
     }
     else{
      turnLeft(); 
     }
     ActualRow = pathRow[p];
     ActualCol = pathCol[p];
     iD = processing[ActualRow - 1][ActualCol - 1];
     Serial.write(iD);
     //serial.write("\n");
     pathRow[z] = ActualRow;
     pathCol[z] = ActualCol;
     z += 1;
     goAhead();
  }
  if(ActualCol == pathCol[p]){  //Se for, eles estão na mesma coluna, devo forward ou backward
     if(ActualRow < pathRow[p]){// turn right
      goBackward();
     }
     else{
      goForward();
     }
     ActualRow = pathRow[p];
     ActualCol = pathCol[p];
     iD = processing[ActualRow - 1][ActualCol - 1];
     Serial.write(iD);
     //serial.write("\n");
     pathRow[z] = ActualRow;
     pathCol[z] = ActualCol;
     z += 1;
     goAhead();
  }
  goAhead(); //?
}

void defineWay(){
  p = z;
  int lock = 1;
  while(lock){
    if(pathRow[p] == StartRow && pathCol[p] == StartCol){
      lock = 0;  
    }
    else{  
      counter++;
      p--;
    }  
  }
  for(i = 0; i < counter; i++){
    finalRow[i] = pathRow[p];
    finalCol[i] = pathCol[p]; 
    p++;
  }
  numerador = n*n;
  Serial.write(numerador);
}
