#pragma once

#include <Arduino.h>

#include "Robot.h"
#include "MotionPlanner.h"


class Gait {

public:
  // Constructor.
  Gait();

  // Inicializar referencias.
  void begin(Robot* robot, MotionPlanner* planner);

  // Iniciar o reiniciar la marcha.
  void start();

  // Actualizar marcha.
  void update(float speed);

  void stop();



private:
  // Referencias.
  Robot* _robot;
  MotionPlanner* _planner;

  // Fase general de marcha.
  float _phase;

  // Tiempo de última actualización.
  unsigned long _lastUpdate;

  // Indica si la marcha está activa.
  bool _running;

  // Actualizar una pata.
  void updateLeg(uint8_t legID, bool swing, float t);
};