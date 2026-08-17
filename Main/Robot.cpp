#include "Robot.h"

// CONSTRUCTOR
Robot::Robot()
  : _pwmRight(0x40), _pwmLeft(0x41) {
  // Inicializar posiciones.
  for (int i = 0; i < 6; i++) {
    _legPosition[i] = { 0.0f, 0.0f, 0.0f };
  }
}

// BEGIN
void Robot::begin() {
  // INICIALIZAR PCA9685
  _pwmRight.begin();
  _pwmRight.setPWMFreq(50);

  _pwmLeft.begin();
  _pwmLeft.setPWMFreq(50);

  // PATAS DERECHAS
  // PCA 0x40
  // RF = canales 0,1,2
  // RM = canales 3,4,5
  // RR = canales 6,7,8

  _legs[RF].init(&_pwmRight, 0, 1, 2, L1, L2, L3);
  _legs[RM].init(&_pwmRight, 3, 4, 5, L1, L2, L3);
  _legs[RR].init(&_pwmRight, 6, 7, 8, L1, L2, L3);

  // PATAS IZQUIERDAS
  // PCA 0x41
  // LF = canales 0,1,2
  // LM = canales 3,4,5
  // LR = canales 6,7,8

  _legs[LF].init(&_pwmLeft, 0, 1, 2, L1, L2, L3);
  _legs[LM].init(&_pwmLeft, 3, 4, 5, L1, L2, L3);
  _legs[LR].init(&_pwmLeft, 6, 7, 8, L1, L2, L3);
}

// HOME
bool Robot::goHome(Workspace* workspace, float homeHeight) {

  // Verificar Workspace.
  if (workspace == nullptr) {
    return false;
  }

  // Buscar la altura disponible más cercana.
  float altura = workspace->getNearestHeight(homeHeight);

  // Consultar UNA SOLA VEZ la línea de trabajo a 0°.
  LineaTrabajo homeLine;

  bool lineOK = workspace->getLine(
    altura,
    0.0f,
    homeLine);

  if (!lineOK) {
    return false;
  }

  // Calcular punto medio de la línea.
  float homeX = (homeLine.xi + homeLine.xf) * 0.5f;
  float homeY = (homeLine.yi + homeLine.yf) * 0.5f;
  float homeZ = altura;

  // Todas las patas usan exactamente el mismo punto local.
  for (uint8_t legID = 0; legID < 6; legID++) {

    bool ok = moveLegTo(
      legID,
      homeX,
      homeY,
      homeZ);

    if (!ok) {
      return false;
    }
  }

  return true;
}

// MOVER UNA PATA
bool Robot::moveLegTo(uint8_t legID, float x, float y, float z) {
  // Verificar índice.
  if (legID >= 6) {
    return false;
  }

  // Calcular IK en Leg.
  bool alcanzable = _legs[legID].moveTo(x, y, z);

  // No actualizar posición si no es alcanzable.
  if (!alcanzable) {
    return false;
  }

  // Guardar posición objetivo.
  _legPosition[legID].x = x;
  _legPosition[legID].y = y;
  _legPosition[legID].z = z;

  return true;
}

// ACTUALIZAR SERVOS
void Robot::update() {
  // Actualizar las seis patas.
  for (int i = 0; i < 6; i++) {
    _legs[i].updateServos();
  }
}

// OBTENER POSICIÓN
Robot::Position Robot::getLegPosition(uint8_t legID) {
  // Protección contra índice incorrecto.
  if (legID >= 6) {
    return { 0.0f, 0.0f, 0.0f };
  }

  return _legPosition[legID];
}

// CINEMÁTICA DEL ROBOT COMPLETO
void Robot::setBodyPose(float x, float y, float z, float roll, float pitch, float yaw) {
  /*
      Reservado para implementar posteriormente la cinemática del cuerpo completo.

      Esta función NO participa en la marcha.

      Futuramente aquí se podrá:

      1. Definir posición del cuerpo.
      2. Definir roll, pitch y yaw.
      3. Transformar coordenadas globales.
      4. Convertirlas a coordenadas locales.
      5. Enviar cada objetivo con moveLegTo().
    */

  (void)x;
  (void)y;
  (void)z;
  (void)roll;
  (void)pitch;
  (void)yaw;
}