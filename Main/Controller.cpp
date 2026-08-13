#include "Controller.h"


// CONSTRUCTOR
Controller::Controller() {
  // Valores iniciales.
  _direction = 0.0f;
  _height = -150.0f;

  // El primer comando debe procesarse.
  _changed = true;
}

// BEGIN
void Controller::begin() {
  // Valores predeterminados.
  _direction = 0.0f;
  _height = -150.0f;

  // Solicitar cálculo inicial.
  _changed = true;
}


// CAMBIAR DIRECCIÓN
void Controller::setDirection(float direction) {
  //_motionMode = WALK;

  // Normalizar dirección.
  direction = normalizeDirection(direction);

  // No hacer nada si no cambió.
  if (direction == _direction) {
    return;
  }

  // Guardar nueva dirección.
  _direction = direction;

  // Solicitar nueva trayectoria.
  _changed = true;
}


/*
//GIROS AXIALES
void Controller::turnLeft() {
  _motionMode = TURN_LEFT;

  // Todas las patas usan exactamente la misma dirección local
  for (int i = 0; i < 6; i++) {
    _localDirections[i] = -90.0f;
  }
}

void Controller::turnRight() {
  _motionMode = TURN_RIGHT;
  // Todas las patas usan exactamente la misma dirección local
  for (int i = 0; i < 6; i++) {
    _localDirections[i] = 90.0f;
  }
}
*/


// CAMBIAR ALTURA
void Controller::setHeight(float height) {
  // No hacer nada si no cambió.
  if (height == _height) {
    return;
  }

  // Guardar nueva altura.
  _height = height;

  // Solicitar nueva trayectoria.
  _changed = true;
}


// CAMBIAR COMANDO COMPLETO
void Controller::setCommand(float direction, float height) {
  // Normalizar dirección.
  direction = normalizeDirection(direction);

  // Revisar si algo cambió.
  if (direction == _direction && height == _height) {
    return;
  }

  // Guardar nuevo comando.
  _direction = direction;
  _height = height;

  // Solicitar nueva trayectoria.
  _changed = true;
}

// OBTENER DIRECCIÓN
float Controller::getDirection() const {
  return _direction;
}

// OBTENER ALTURA
float Controller::getHeight() const {
  return _height;
}

// COMPROBAR CAMBIO
bool Controller::hasChanged() const {
  return _changed;
}

// LIMPIAR CAMBIO
void Controller::clearChanged() {
  // El comando ya fue procesado.
  _changed = false;
}

// NORMALIZAR DIRECCIÓN
float Controller::normalizeDirection(float direction) {
  // Convertir a rango 0° - 360°.
  while (direction >= 360.0f) {
    direction -= 360.0f;
  }

  while (direction < 0.0f) {
    direction += 360.0f;
  }

  return direction;
}