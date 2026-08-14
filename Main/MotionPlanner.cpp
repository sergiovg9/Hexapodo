#include "MotionPlanner.h"

// CONSTRUCTOR
MotionPlanner::MotionPlanner() {
  _robot = nullptr;
  _workspace = nullptr;

  _height = -150.0f;
  _globalDirection = -90.0f;

  _trajectoryReady = false;

  // Inicializar trayectorias.
  for (int i = 0; i < 6; i++) {
    _trajectory[i].start = { 0.0f, 0.0f, 0.0f };
    _trajectory[i].end = { 0.0f, 0.0f, 0.0f };
    _trajectory[i].flight = { 0.0f, 0.0f, 0.0f };

    _localDirection[i] = 0.0f;
  }
}

// BEGIN
void MotionPlanner::begin(Robot* robot, Workspace* workspace) {
  _robot = robot;
  _workspace = workspace;
}

// CALCULAR TRAYECTORIAS
bool MotionPlanner::calculateTrajectories(float globalDirection, float height, MovementMode mode) {
  // Verificar módulos.
  if (_robot == nullptr || _workspace == nullptr) {
    return false;
  }

  // Guardar comando actual.
  _globalDirection = normalizeAngle(globalDirection);
  _height = _workspace->getNearestHeight(height);

  // CALCULAR CADA PATA
  for (uint8_t legID = 0; legID < 6; legID++) {
    // Elegir dirección local dependiendo del modo.
    float localDirection;

    if (mode == TURN_LEFT) {
      // Giro axial hacia la izquierda.Todas las patas recorren dirección local 90°.
      localDirection = 90.0f;

    } else if (mode == TURN_RIGHT) {
      // Giro axial hacia la derecha.
      localDirection = 270.0f;

    } else {
      // Caminata normal.
      localDirection = calculateLocalDirection(legID, _globalDirection);
    }

    _localDirection[legID] = localDirection;


    // El Workspace contiene todas las direcciones de 0° a 358°.
    float workspaceDirection = normalizeAngle(localDirection);


    // BUSCAR LÍNEA DEL SUELO
    LineaTrabajo groundLine;

    // Buscar usando solamente 0° - 180°.
    bool groundOK = _workspace->getLine(_height, workspaceDirection, groundLine);

    if (!groundOK) {
      _trajectoryReady = false;
      return false;
    }

    // INICIO Y FINAL.
    // Cada dirección tiene su propia línea en el Workspace.
    _trajectory[legID].start.x = groundLine.xi;
    _trajectory[legID].start.y = groundLine.yi;

    _trajectory[legID].end.x = groundLine.xf;
    _trajectory[legID].end.y = groundLine.yf;

    _trajectory[legID].start.z = _height;
    _trajectory[legID].end.z = _height;

    // PUNTO DE VUELO
    // El workspace está separado cada 5 mm. 20 alturas equivalen a 100 mm.
    float requestedFlightHeight = _height + 100.0f;

    float flightHeight = _workspace->getNearestHeight(requestedFlightHeight);

    LineaTrabajo flightLine;

    // El punto de vuelo se consulta a 0°: Usamos el primer punto de la línea superior. Este será el punto de control de la curva Bézier del movimiento de vuelo.
    bool flightOK = _workspace->getLine(flightHeight, workspaceDirection, flightLine);

    if (!flightOK) {
      _trajectoryReady = false;
      return false;
    }

    // Centro de la línea alcanzable a la altura de vuelo.
    float flightCenterX = (flightLine.xi + flightLine.xf) * 0.5f;
    float flightCenterY = (flightLine.yi + flightLine.yf) * 0.5f;


    // Punto superior del paso.
    _trajectory[legID].flight.x = flightCenterX;
    _trajectory[legID].flight.y = flightCenterY;
    _trajectory[legID].flight.z = flightHeight;
  }

  // Trayectorias listas.
  _trajectoryReady = true;

  return true;
}

// DIRECCIÓN LOCAL
float MotionPlanner::calculateLocalDirection(
  uint8_t legID,
  float globalDirection) const {
  float offset = 0.0f;

  switch (legID) {
    case Robot::RF:
      offset = 30.0f;
      break;


    case Robot::RM:
      offset = 90.0f;
      break;


    case Robot::RR:
      offset = 150.0f;
      break;


    case Robot::LF:
      offset = -30.0f;
      break;


    case Robot::LM:
      offset = -90.0f;
      break;


    case Robot::LR:
      offset = -150.0f;
      break;


    default:
      offset = 0.0f;
      break;
  }


  return normalizeAngle(globalDirection + offset);
}


// STANCE
MotionPlanner::Point3D MotionPlanner::getStancePoint(uint8_t legID, float t) const {
  // Limitar tiempo.
  t = constrain(t, 0.0f, 1.0f);

  if (!_trajectoryReady || legID >= 6) {
    return { 0.0f, 0.0f, 0.0f };
  }

  //Durante stance la pata se mueve del final hacia el inicio. El pie permanece en el piso.
  return linearInterpolation(_trajectory[legID].end, _trajectory[legID].start, t);
}

// SWING
MotionPlanner::Point3D MotionPlanner::getSwingPoint(uint8_t legID, float t) const {
  // Limitar tiempo.
  t = constrain(t, 0.0f, 1.0f);


  if (!_trajectoryReady || legID >= 6) {
    return { 0.0f, 0.0f, 0.0f };
  }

  //Durante swing: inicio → punto de vuelo → final

  return quadraticBezier(_trajectory[legID].start, _trajectory[legID].flight, _trajectory[legID].end, t);
}

// OBTENER TRAYECTORIA
MotionPlanner::LegTrajectory MotionPlanner::getTrajectory(uint8_t legID) const {
  if (legID >= 6) {
    LegTrajectory empty;

    empty.start = { 0.0f, 0.0f, 0.0f };
    empty.end = { 0.0f, 0.0f, 0.0f };
    empty.flight = { 0.0f, 0.0f, 0.0f };

    return empty;
  }


  return _trajectory[legID];
}

// INTERPOLACIÓN LINEAL
MotionPlanner::Point3D MotionPlanner::linearInterpolation(const Point3D& p0, const Point3D& p1, float t) const {
  Point3D result;

  result.x = p0.x + (p1.x - p0.x) * t;

  result.y = p0.y + (p1.y - p0.y) * t;

  result.z = p0.z + (p1.z - p0.z) * t;

  return result;
}

// BÉZIER CUADRÁTICA
MotionPlanner::Point3D MotionPlanner::quadraticBezier(const Point3D& p0, const Point3D& p1, const Point3D& p2, float t) const {
  Point3D result;

  float u = 1.0f - t;

  // Bézier cuadrática.
  result.x = u * u * p0.x + 2.0f * u * t * p1.x + t * t * p2.x;
  result.y = u * u * p0.y + 2.0f * u * t * p1.y + t * t * p2.y;
  result.z = u * u * p0.z + 2.0f * u * t * p1.z + t * t * p2.z;

  return result;
}


// NORMALIZAR ÁNGULO
float MotionPlanner::normalizeAngle(float angle) const {
  while (angle >= 360.0f) {
    angle -= 360.0f;
  }

  while (angle < 0.0f) {
    angle += 360.0f;
  }

  return angle;
}