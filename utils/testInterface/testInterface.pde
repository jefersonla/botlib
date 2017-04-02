/**
 * MIT License
 * 
 * Copyright (c) 2017 Jeferson Lima & Andressa Andrade
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
/**
 *
 * Interface for test and configuration of Robozino
 * Use processing 3.1.X or above with the libraries listed bellow:
 *   > ControlP5
 *
 */

import controlP5.ControlP5;
import controlP5.ControlEvent;
import controlP5.Textfield;
import controlP5.Toggle;
import controlP5.Button;
import controlP5.ScrollableList;
import processing.serial.Serial;

/* Color Constants */
final color COLOR_WHITE = color(255);
final color COLOR_ORANGE_LIGHT = color(131, 255, 20);
final color COLOR_GREEN_LIGHT = color(232, 158, 12);
final color COLOR_RED = color(255, 0, 0);
final color COLOR_PURPLE = color(62, 12, 232);
final color COLOR_BLUE_LIGHT = color(13, 255, 243);
final color COLOR_PINK = color(200, 46, 232);

/* Buttons */
final int START_MOTOR_BUTTON = 1;
final int STOP_MOTOR_BUTTON = 2;

/* Serial Speed */
final int SERIAL_SPEED = 115200;

/* Messages to send */
final String START_MOTOR_MSG = "I";
final String STOP_MOTOR_MSG = "S";

/* P5 Controller */
ControlP5 cp5;

/* Number of lines in LineGraph */
final int numberOfLines = 2;

/* Time Interval */
final int timeInterval = 60;

/* Enable use of mockup serial */
final boolean mockupSerial = false;

/* Graph Line colors */
color graphColors[];

/* Graph of rotations of each motor */
Graph LineGraph;

/* Display line check */
boolean displayLine[];

/* Content of LineGraph */
float[][] lineGraphValues;
float[] lineGraphSampleNumbers;

/* List of serial availables */
String serialList[];

/* Serial object and serial enable variable */
boolean serialEnabled;
Serial serialPort;

/* Serial Buffer */
final int MAX_SERIAL_BUFFER = 100;
byte[] serialBuffer; 

/* Logo Image */
PImage logo;

/* Setup Interface */
void setup() {
  /* Create Interface */
  size(800, 600);
  noStroke();
  surface.setTitle("Robozino - Debug Interface");

  /* Create a new controller for cp5 lib */
  cp5 = new ControlP5(this);

  /* Configure Line Graph */

  /* Construct LineGraph */
  LineGraph = new Graph(239, 225, 480, 290, color (20, 20, 200));

  /* Labels of LineGraph */
  LineGraph.xLabel =" Time ";
  LineGraph.yLabel =" Steps ";
  LineGraph.Title =" Rotations of Each Motor ";  

  /* Num. of div. X axis / Max of X axis / Min of Y axis */
  LineGraph.xDiv = 6;
  LineGraph.xMax = 0;
  LineGraph.xMin = -60;

  /* Num. of div. of Y axis / Max of Y axis / Min of Y axis */
  LineGraph.yDiv = 10;
  LineGraph.yMax = 250;
  LineGraph.yMin = 0;

  /* Configurations of plot of LineGraph */

  /* Colors of the LineGraph */
  graphColors = new color[numberOfLines];
  graphColors[0] = COLOR_ORANGE_LIGHT;
  graphColors[1] = COLOR_GREEN_LIGHT;

  /* Array of values of the LineGraph */
  lineGraphValues = new float[numberOfLines][timeInterval];

  /* I'm not sure what is that */
  lineGraphSampleNumbers = new float[timeInterval];

  /* Initialize matrix for line graph and samples */
  for (int i = 0; i < numberOfLines; i++) {
    for (int j = 0; j < timeInterval; j++) {
      /* Initialize lineGraphValues */
      lineGraphValues[i][j] = 0;

      /* Initialize sample number one time only */
      if (i == 0) {
        lineGraphSampleNumbers[j] = j;
      }
    }
  }

  /* Initialize visibility of LineGraph lines */
  displayLine = new boolean[numberOfLines];
  for (int i = 0; i < numberOfLines; i++) {
    displayLine[i] = true;
  }

  /* Resize Y axis limits input */
  int xAxisLineGraph = 190;
  int yAxisLineGraph = 208;
  int inputWidth = 40;
  cp5.addTextfield("lgMaxY")
    .setPosition(xAxisLineGraph, yAxisLineGraph)
    .setText("250")
    .setWidth(inputWidth)
    .setAutoClear(false);
  cp5.addTextfield("lgMinY")
    .setPosition(xAxisLineGraph, yAxisLineGraph + 290)
    .setText("0")
    .setWidth(inputWidth)
    .setAutoClear(false);

  /* Label for on/off button and change kp */
  int labelXPos = 10;
  int labelYPos = 160;
  cp5.addTextlabel("on/off")
    .setText("on/off")
    .setPosition(labelXPos + 25, labelYPos)
    .setColor(0);
  cp5.addTextlabel("pgain")
    .setText("P Gain")
    .setPosition(labelXPos + 75, labelYPos)
    .setColor(0);
  cp5.addTextlabel("lmark")
    .setText("L")
    .setPosition(labelXPos, labelYPos + 20)
    .setColor(0);
  cp5.addTextlabel("rmark")
    .setText("R")
    .setPosition(labelXPos, labelYPos + 49)
    .setColor(0);

  /* Enable graphic visualization of each steps counter */
  int gainXPos = 30;
  int gainYPos = 174;
  cp5.addToggle("lgStepsL")
    .setPosition(gainXPos, gainYPos)
    .setValue(true)
    .setMode(ControlP5.SWITCH)
    .setColorActive(graphColors[0]);
  cp5.addToggle("lgStepsR")
    .setPosition(gainXPos, gainYPos + 30)
    .setValue(true)
    .setMode(ControlP5.SWITCH)
    .setColorActive(graphColors[1]);

  /* Input for P Gain */
  cp5.addTextfield("pGainL")
    .setPosition(gainXPos + 52, gainYPos)
    .setText("1.0")
    .setWidth(inputWidth)
    .setAutoClear(false);
  /* Input for P Gain */
  cp5.addTextfield("pGainR")
    .setPosition(gainXPos + 52, gainYPos + 30)
    .setText("1.0")
    .setWidth(inputWidth)
    .setAutoClear(false);

  /* Display serial avaialable dropdown list and label*/
  int serialListX = 673;
  int serialListY = 115;
  serialList = Serial.list();
  cp5.addTextlabel("serialListLabel")
    .setText("Serial List")
    .setPosition(serialListX, serialListY)
    .setColor(0);
  cp5.addScrollableList("SerialList")
    .setPosition(serialListX, serialListY + 15)
    .setSize(100, 200)
    .setBarHeight(20)
    .setItemHeight(20)
    .addItems(serialList)
    .setOpen(false)
    .setType(ScrollableList.DROPDOWN);

  /* Toolbar control */
  int toolbarX = 400;
  int toolbarY = 130;

  /* Enable Serial and label */
  cp5.addTextlabel("serialEnableLabel")
    .setText("  Serial\n on/off")
    .setPosition(toolbarX + 222, toolbarY - 20)
    .setColor(0);
  cp5.addToggle("seEn")
    .setPosition(toolbarX + 222, toolbarY)
    .setValue(false)
    .setMode(ControlP5.SWITCH)
    .setColorActive(COLOR_BLUE_LIGHT);

  /* Start Motors */
  cp5.addButton("Start Motor")
    .setValue(START_MOTOR_BUTTON)
    .setPosition(toolbarX, toolbarY)
    .updateSize()
    .setSize(100, 20);

  /* Stop Motors */
  cp5.addButton("Stop Motor")
    .setValue(STOP_MOTOR_BUTTON)
    .setPosition(toolbarX + 110, toolbarY)
    .updateSize()
    .setSize(100, 20);

  /* Start with serial disabled */
  serialEnabled = false;
  serialPort = null;

  /* Initialize Serial Buffer */
  serialBuffer = new byte[MAX_SERIAL_BUFFER];

  /* Load logo image */
  logo = loadImage("images/logo.png");

  /* Set interface background */
  background(COLOR_WHITE);

  /* Put logo image */
  image(logo, 15, 15);
}

/* Draw interface */
void draw() {

  /* Read serial and update values if serial is enabled and available or debug serial is enabled */
  if ((serialEnabled && serialPort.available() > 0) || mockupSerial) {
    /* String with data received from serial */
    String dataString = ",";

    /* If mockup Serial was enabled just go ahead */
    if (mockupSerial) {
      dataString = mockupSerialFunction();
    }
    /* If debug serial was enabled and mockup serial wasn't being used */
    else if (serialEnabled) {
      /* Try read data from Serial Port */
      try {
        serialPort.readBytesUntil('\n', serialBuffer);
      }
      /* Catch exception if it's not possible read from serial */
      catch (Exception e) {
        errorMsg("ERROR WHILE READING FROM SERIAL PORT, MAYBE YOU DON'T HAVE ENOUGH\n" + 
          "PRIVILEGES, OR THIS SERIAL IS BEING USED BY ANOTHER PROCCESS");
        e.printStackTrace();
      }

      /* Convert byte buffer to ascii string */
      dataString = new String(serialBuffer);
    }

    /* Split string received  */
    String[] dataReceived = split(dataString, ',');

    /* Update lines with content received from Serial Port */
    for (int i = 0; i < numberOfLines; i++) {
      /* Shift content in one position */
      for (int j = 0; j < timeInterval - 1; j++) {
        lineGraphValues[i][j] = lineGraphValues[i][j + 1];
      }

      /* Add value received in each line */
      if (dataReceived.length >= numberOfLines) {
        lineGraphValues[i][timeInterval - 1] = float(dataReceived[i]);
      }
      /* Otherwise add a 0 */
      else {
        errorMsg("INVALID DATA RECEIVED");
        lineGraphValues[i][timeInterval - 1] = 0;
      }
    }
  }

  /* Check if Serial List has updated */
  if (serialList != Serial.list()) {
    /* Update Serial List */
    serialList = Serial.list();
    cp5.get(ScrollableList.class, "SerialList")
      .setItems(serialList);
  }

  /* Print the LineGraph */
  LineGraph.DrawAxis();
  for (int i = 0; i < numberOfLines; i++) {
    /* Check if the line is configured to be displayed */
    if (displayLine[i]) {
      /* Chose color of the actual line */
      LineGraph.GraphColor = graphColors[i];
      /* Update the graph with the time */
      LineGraph.LineGraph(lineGraphSampleNumbers, lineGraphValues[i]);
    }
  }
}

/* Check which event has occured */
void controlEvent(ControlEvent theEvent) {

  /* Get name of interface which perform the event */
  String parameter = theEvent.getName();

  /* If the event was triggered from a text field */
  if (theEvent.isAssignableFrom(Textfield.class)) {

    /* Get value of textfield */
    String value = theEvent.getStringValue();

    /* Now change parameters accordily */
    switch(parameter) {
    case "lgMaxY": 
      LineGraph.yMax = float(value);
      infoMsg("UPDATING MAX LIMIT OF LINE GRAPH");
      break;
    case "lgMinY":
      LineGraph.yMin = float(value);
      infoMsg("UPDATING MIN LIMIT OF LINE GRAPH");
      break;
    case "pGainL":
      infoMsg("SETTING A NEW P GAIN ON LEFT SIDE");
      if(serialEnabled){
        //TODO
      }
      else{
        warningMsg("SERIAL NOT CONECTED");
      }
      break;
    case "pGainR":
      infoMsg("SETTING A NEW P GAIN ON RIGHT SIDE");
      if(serialEnabled){
        //TODO
      }
      else{
        warningMsg("SERIAL NOT CONECTED");
      }
      break;
    default:
      errorMsg("UNKNOW TEXTFIELD EVENT - " + parameter + " - " + value);
    }
  }
  /* Was triggered from a switch button */
  else if (theEvent.isAssignableFrom(Toggle.class)) {

    /* Get value of the toggle switch button */
    boolean value = theEvent.getValue() > 0 ? true : false;

    /* Check which operations need to be perfomed */
    switch(parameter) {
    case "lgStepsL":
      infoMsg("CHANGING STATE OF VISIBILITY OF LINE STEPS MOTOR LEFT");
      displayLine[0] = value;
      break;
    case "lgStepsR":
      infoMsg("CHANGING STATE OF VISIBILITY OF LINE STEPS MOTOR RIGHT");
      displayLine[1] = value;
      break;
    case "seEn":
      /* If value is to enable serial */
      if (value) {
        /* And serial port is correct, turn enable serial flag true */
        if (serialPort != null) {
          infoMsg("SERIAL ACTIVATED SUCCESSFULLY");
          serialEnabled = true;
        }
        /* Otherwise, if we found errors keep button on off state */
        else {
          errorMsg("CANNOT ACTIVATE SERIAL. SERIAL PORT UNDEFINED");
          cp5.get(Toggle.class, "seEn")
            .setValue(false);
          serialEnabled = false;
        }
      }
      /* Otherwise just disable serial interface */
      else {
        infoMsg("SERIAL DISABLED");
        serialEnabled = false;
      }
      break;
    default:
      errorMsg("UNKNOW SWITCH EVENT - " + parameter + " - " + value);
    }
  } 
  /* Button event */
  else if (theEvent.isAssignableFrom(Button.class)) {

    /* Execut action of the clicked button */
    switch(parameter) {
    case "Start Motor":
      /* If serial is enabled send the start message */
      if (serialEnabled) {
        serialPort.write(START_MOTOR_MSG);
        infoMsg("SENDING START MESSAGE TO ROBOT");
      } else {
        warningMsg("SERIAL NOT CONECTED");
      }
      break;
    case "Stop Motor":
      /* If serial is enabled send the stop message */
      if (serialEnabled) {
        serialPort.write(STOP_MOTOR_MSG);
        infoMsg("SENDING STOP MESSAGE TO ROBOT");
      } else {
        warningMsg("SERIAL NOT CONECTED");
      }
      break;
    default:
      errorMsg("UNKNOW BUTTON EVENT - " + parameter);
    }
  }
  /* Dropdown list event */
  else if (theEvent.isAssignableFrom(ScrollableList.class)) {
    /* Get Value of scrollable list clicked */
    int value = int(theEvent.getValue());

    /* Configure the desired dropdown list */
    switch(parameter) {
    case "SerialList":
      infoMsg("SERIAL PORT - " + serialList[value] + " - SELECTED");
      /* Try to instantiate a new serial connection */
      try {
        serialPort = new Serial(this, serialList[value], SERIAL_SPEED);
      }
      catch(Exception e) {
        errorMsg("CANNOT CONNECT TO SERIAL PORT " + serialList[value] + ", \n" + 
          "PROBABLY YOU DON'T HAVE ENOUGH PRIVILEGES, " + 
          "OR THIS INTERFACE IS BEING USED BY ANOTHER PROGRAM");
        serialPort = null;
      }
      break;
    default:
      errorMsg("UNKNOW SCROLLABLE LIST EVENT - " + parameter);
    }
  }
  /* Unused Events */
  else {
    warningMsg("UNUSED EVENT DETECTED");
  }
}

/* Debug Messages */

/* Info Message */
void infoMsg(String msg) {
  System.out.println("[INFO] " + msg);
}

/* Warning Message */
void warningMsg(String msg) {
  System.err.println("[WARNING] " + msg);
}

/* Error Message */
void errorMsg(String msg) {
  System.err.println("[ERROR] " + msg);
}
