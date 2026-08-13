#include "Gait.h"

// CONSTRUCTOR
Gait::Gait() {
  _robot = nullptr;
  _planner = nullptr;

  _phase = 0.0f;

  // Velocidad inicial de marcha.
  _speed = 0.4f;
  _lastUpdate = 0;
  _running = false;
}

// BEGIN
void Gait::begin(Robot* robot, MotionPlanner* planner) {
  _robot = robot;
  _planner = planner;

  _phase = 0.0f;
  _lastUpdate = millis();
  _running = false;
}

// START
void Gait::start() {
  // Reiniciar ciclo.
  _phase = 0.0f;

  _lastUpdate = millis();

  _running = true;
}

// VELOCIDAD
void Gait::setSpeed(float speed) {
  // Evitar valores negativos.
  if (speed < 0.0f) {
    speed = 0.0f;
  }

  _speed = speed;
}

// UPDATE
void Gait::update() {
  // Verificar módulos.
  if (_robot == nullptr || _planner == nullptr) {
    return;
  }

  // No mover si no está iniciada la marcha.
  if (!_running) {
    return;
  }

  // TIEMPO
  unsigned long now = millis();
  float dt = (now - _lastUpdate) / 1000.0f;
  _lastUpdate = now;


  // Limitar saltos grandes de tiempo.
  if (dt > 0.1f) {
    dt = 0.1f;
  }

  // Avanzar fase.
  _phase += _speed * dt;

  // Mantener fase entre 0 y 1.
  while (_phase >= 1.0f) {
    _phase -= 1.0f;
  }
  // PRIMERA MITAD

  if (_phase < 0.5f) {
    // Convertir 0.0 - 0.5 a 0.0 - 1.0.
    float t = _phase * 2.0f;

    // SWING
    updateLeg(Robot::RM, true, t);
    updateLeg(Robot::LF, true, t);
    updateLeg(Robot::LR, true, t);

    // STANCE
    updateLeg(Robot::RF, false, t);
    updateLeg(Robot::RR, false, t);
    updateLeg(Robot::LM, false, t);
  }

  // SEGUNDA MITAD
  else {
    // Convertir 0.5 - 1.0 a 0.0 - 1.0.
    float t = (_phase - 0.5f) * 2.0f;

    // SWING
    updateLeg(Robot::RF, true, t);
    updateLeg(Robot::RR, true, t);
    updateLeg(Robot::LM, true, t);

    // STANCE
    updateLeg(Robot::RM, false, t);
    updateLeg(Robot::LF, false, t);
    updateLeg(Robot::LR, false, t);
  }
}

// ACTUALIZAR PATA
void Gait::updateLeg(uint8_t legID, bool swing, float t) {
  MotionPlanner::Point3D point;

  if (swing) {
    point = _planner->getSwingPoint(legID, t);
  } else {
    point = _planner->getStancePoint(legID, t);
  }

  // Mandar objetivo a la pata.
  _robot->moveLegTo(legID, point.x, point.y, point.z);
}