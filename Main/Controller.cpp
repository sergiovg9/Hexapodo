#include "Controller.h"


// CONSTRUCTOR
Controller::Controller() {

  // Valores predeterminados.
  _direction = 0.0f;
  _height = -150.0f;
  _speed = 0.4f;

  _movementMode = MotionPlanner::WALK;
  _modoAnterior = MotionPlanner::WALK;

  // El primer comando debe calcular trayectoria.
  _changed = true;

  // Banderas de interrupción.
  _cambioModo = true;
  _cambioDireccion = false;
  _cambioAltura = false;
  _cambioVelocidad = false;

  // El robot comienza detenido en HOME.
  _movementEnabled = false;
  _movementStateChanged = false;
  _cambioMarcha = false;
  _gaitStartRequired = false;
}


// BEGIN
void Controller::begin() {

  // Valores predeterminados.
  _direction = 0.0f;
  _height = -150.0f;
  _speed = 0.4f;

  _movementMode = MotionPlanner::WALK;
  _modoAnterior = MotionPlanner::WALK;

  _changed = true;

  _cambioModo = true;
  _cambioDireccion = false;
  _cambioAltura = false;
  _cambioVelocidad = false;
  _movementEnabled = false;
  _movementStateChanged = false;
  _cambioMarcha = false;
  _gaitStartRequired = false;


  // CONFIGURAR PINES
  pinMode(PIN_GIRO_IZQUIERDA, INPUT_PULLDOWN);
  pinMode(PIN_GIRO_DERECHA, INPUT_PULLDOWN);

  pinMode(PIN_DIRECCION_50, INPUT_PULLDOWN);
  pinMode(PIN_ALTURA_110, INPUT_PULLDOWN);
  pinMode(PIN_VELOCIDAD, INPUT_PULLDOWN);

  pinMode(PIN_MARCHA, INPUT_PULLDOWN);

  // CONFIGURAR INTERRUPCIONES
  attachInterruptArg(digitalPinToInterrupt(PIN_GIRO_IZQUIERDA), interrupcionModo, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(PIN_GIRO_DERECHA), interrupcionModo, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(PIN_DIRECCION_50), interrupcionDireccion, this, RISING);
  attachInterruptArg(digitalPinToInterrupt(PIN_ALTURA_110), interrupcionAltura, this, RISING);
  attachInterruptArg(digitalPinToInterrupt(PIN_VELOCIDAD), interrupcionVelocidad, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(PIN_MARCHA), interrupcionMarcha, this, RISING);
}


// UPDATE
void Controller::update() {
  // Procesar solamente los controles que hayan generado una interrupción.
  actualizarModoMovimiento();
  actualizarDireccion();
  actualizarAltura();
  actualizarVelocidad();
  actualizarMarcha();
}


// INTERRUPCIONES
void IRAM_ATTR Controller::interrupcionModo(void* arg) {
  Controller* controller = static_cast<Controller*>(arg);
  controller->_cambioModo = true;
}

void IRAM_ATTR Controller::interrupcionDireccion(void* arg) {
  Controller* controller = static_cast<Controller*>(arg);
  controller->_cambioDireccion = true;
}

void IRAM_ATTR Controller::interrupcionAltura(void* arg) {
  Controller* controller = static_cast<Controller*>(arg);
  controller->_cambioAltura = true;
}

void IRAM_ATTR Controller::interrupcionVelocidad(void* arg) {
  Controller* controller = static_cast<Controller*>(arg);
  controller->_cambioVelocidad = true;
}

void IRAM_ATTR Controller::interrupcionMarcha(void* arg) {
  Controller* controller = static_cast<Controller*>(arg);
  controller->_cambioMarcha = true;
}


// ACTUALIZAR MODO
void Controller::actualizarModoMovimiento() {

  if (!_cambioModo) {
    return;
  }

  _cambioModo = false;

  bool izquierda = digitalRead(PIN_GIRO_IZQUIERDA);
  bool derecha = digitalRead(PIN_GIRO_DERECHA);

  MotionPlanner::MovementMode nuevoModo;

  if (izquierda && !derecha) {

    nuevoModo = MotionPlanner::TURN_LEFT;

  } else if (derecha && !izquierda) {

    nuevoModo = MotionPlanner::TURN_RIGHT;

  } else {

    nuevoModo = MotionPlanner::WALK;
  }

  if (nuevoModo != _movementMode) {


    _movementMode = nuevoModo;

    _changed = true;

    if (!_movementEnabled && (nuevoModo == MotionPlanner::TURN_LEFT || nuevoModo == MotionPlanner::TURN_RIGHT)) {
      _gaitStartRequired = true;
    }
  }
}

// ACTUALIZAR DIRECCIÓN
void Controller::actualizarDireccion() {

  if (!_cambioDireccion) {
    return;
  }

  _cambioDireccion = false;

  // Control temporal.
  setDirection(50.0f);
}


// ACTUALIZAR ALTURA
void Controller::actualizarAltura() {

  if (!_cambioAltura) {
    return;
  }

  _cambioAltura = false;

  // Control temporal.
  setHeight(-110.0f);
}


// ACTUALIZAR VELOCIDAD
void Controller::actualizarVelocidad() {

  if (!_cambioVelocidad) {
    return;
  }

  _cambioVelocidad = false;

  // Control temporal.
  setSpeed(0.8f);
}

void Controller::actualizarMarcha() {

  if (!_cambioMarcha) {
    return;
  }

  _cambioMarcha = false;

  // Alternar entre marcha y HOME.
  _movementEnabled = !_movementEnabled;

  // Avisar al Main.
  _movementStateChanged = true;

  // Si vamos a comenzar a caminar, calcular nuevamente las trayectorias.
  if (_movementEnabled) {
    _changed = true;
  }
}


// SET DIRECTION
void Controller::setDirection(float direction) {

  direction = normalizeDirection(direction);

  if (direction == _direction) {
    return;
  }

  _direction = direction;

  // La dirección modifica las trayectorias.
  _changed = true;
}


// SET HEIGHT
void Controller::setHeight(float height) {

  if (height == _height) {
    return;
  }

  _height = height;

  // La altura modifica las trayectorias.
  _changed = true;
}


// SET SPEED
void Controller::setSpeed(float speed) {

  if (speed < 0.0f) {
    speed = 0.0f;
  }

  _speed = speed;

  // IMPORTANTE: Cambiar velocidad NO requiere recalcular trayectorias.
}


// SET MOVEMENT MODE
void Controller::setMovementMode(MotionPlanner::MovementMode mode) {

  if (mode == _movementMode) {
    return;
  }

  _movementMode = mode;

  // El modo modifica las trayectorias.
  _changed = true;

  if (!_movementEnabled && (mode == MotionPlanner::TURN_LEFT || mode == MotionPlanner::TURN_RIGHT)) {
    _gaitStartRequired = true;
  }
}

// SET MOVEMENT ENABLED
void Controller::setMovementEnabled(bool enabled) {

  if (enabled == _movementEnabled) {
    return;
  }

  _movementEnabled = enabled;

  // Avisar al Main que cambió entre marcha y HOME.
  _movementStateChanged = true;

  // Si comenzamos a caminar, recalcular trayectoria.
  if (_movementEnabled) {
    _changed = true;
  }
}

// SET COMMAND
void Controller::setCommand(float direction, float height) {

  direction = normalizeDirection(direction);

  if (direction == _direction && height == _height) {
    return;
  }

  _direction = direction;
  _height = height;

  _changed = true;
}


// GETTERS

float Controller::getDirection() const {
  return _direction;
}

float Controller::getHeight() const {
  return _height;
}

float Controller::getSpeed() const {
  return _speed;
}

MotionPlanner::MovementMode Controller::getMovementMode() const {
  return _movementMode;
}

bool Controller::isMovementEnabled() const {
  return _movementEnabled;
}

bool Controller::hasMovementStateChanged() const {
  return _movementStateChanged;
}

void Controller::clearMovementStateChanged() {
  _movementStateChanged = false;
}


// CAMBIO DE TRAYECTORIA
bool Controller::hasChanged() const {
  return _changed;
}

void Controller::clearChanged() {

  _changed = false;
}

bool Controller::isTurning() const {

  return _movementMode == MotionPlanner::TURN_LEFT || _movementMode == MotionPlanner::TURN_RIGHT;
}

bool Controller::isGaitStartRequired() const {
  return _gaitStartRequired;
}

void Controller::clearGaitStartRequired() {
  _gaitStartRequired = false;
}

// NORMALIZAR DIRECCIÓN
float Controller::normalizeDirection(float direction) {

  while (direction >= 360.0f) {
    direction -= 360.0f;
  }

  while (direction < 0.0f) {
    direction += 360.0f;
  }

  return direction;
}