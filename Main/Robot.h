#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Leg.h"
#include "Workspace.h"


class Robot {
public:
  // ÍNDICES DE LAS PATAS
  enum LegID {
    RF = 0,  // Right Front
    RM = 1,  // Right Middle
    RR = 2,  // Right Rear

    LF = 3,  // Left Front
    LM = 4,  // Left Middle
    LR = 5   // Left Rear
  };

  // ESTRUCTURA DE POSICIÓN
  struct Position {
    float x;
    float y;
    float z;
  };

  // Constructor.
  Robot();

  // Inicializar PCA9685 y patas.
  void begin();

  // Establecer posición HOME usando el Workspace.
  bool goHome(Workspace* workspace, float homeHeight);

  // Establecer objetivo de una pata.
  bool moveLegTo(uint8_t legID, float x, float y, float z);

  // Actualizar los servos.
  void update();

  // Obtener posición objetivo de una pata.
  Position getLegPosition(uint8_t legID);


  // CINEMÁTICA DEL ROBOT COMPLETO
  // Reservada para movimientos futuros del cuerpo.
  // No se utiliza para la marcha actual.
  void setBodyPose(float x, float y, float z, float roll, float pitch, float yaw);


  /*=========Esto es temporal(?)==========*/
  // Permite usar las PCA ya inicializadas
  Adafruit_PWMServoDriver* getPcaA() {
    return &_pwmFront;
  }

  Adafruit_PWMServoDriver* getPcaB() {
    return &_pwmMiddle;
  }

  Adafruit_PWMServoDriver* getPcaC() {
    return &_pwmRear;
  }


private:
  // PCA9685
  Adafruit_PWMServoDriver _pwmFront;
  Adafruit_PWMServoDriver _pwmMiddle;
  Adafruit_PWMServoDriver _pwmRear;

  // PATAS
  Leg _legs[6];

  // POSICIÓN ACTUAL
  Position _legPosition[6];

  // GEOMETRÍA DE LAS PATAS
  static constexpr float L1 = 49.25f;
  static constexpr float L2 = 101.63f;
  static constexpr float L3 = 127.92f;
};