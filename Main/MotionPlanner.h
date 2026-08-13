#pragma once

#include <Arduino.h>

#include "Robot.h"
#include "Workspace.h"


class MotionPlanner {

public:

  // Punto 3D.
  struct Point3D {
    float x;
    float y;
    float z;
  };

  // Trayectoria completa de una pata.
  struct LegTrajectory {
    // Inicio del paso.
    Point3D start;

    // Final del paso.
    Point3D end;

    // Punto de control para el vuelo.
    Point3D flight;
  };

  // Tipo de movimiento.
  enum MovementMode {
    WALK,
    TURN_LEFT,
    TURN_RIGHT
  };

  // Constructor.
  MotionPlanner();

  // Inicializar referencias.
  void begin(Robot* robot, Workspace* workspace);

  // Calcular las trayectorias de las 6 patas.
  bool calculateTrajectories(float globalDirection, float height, MovementMode mode = WALK);

  // Obtener punto durante stance.
  Point3D getStancePoint(uint8_t legID, float t) const;

  // Obtener punto durante swing.
  Point3D getSwingPoint(uint8_t legID, float t) const;

  // Obtener trayectoria calculada.
  LegTrajectory getTrajectory(uint8_t legID) const;



private:

  // Referencias a otros módulos.
  Robot* _robot;
  Workspace* _workspace;

  // Trayectorias calculadas.
  LegTrajectory _trajectory[6];

  // Dirección local de cada pata.
  float _localDirection[6];

  // Altura seleccionada.
  float _height;

  // Dirección global seleccionada.
  float _globalDirection;

  // Indica si ya existen trayectorias.
  bool _trajectoryReady;

  // Convertir dirección global a local.
  float calculateLocalDirection(uint8_t legID, float globalDirection) const;

  // Normalizar ángulo a 0° - 360°.
  float normalizeAngle(float angle) const;

  // Interpolación lineal.
  Point3D linearInterpolation(const Point3D& p0, const Point3D& p1, float t) const;

  // Curva Bézier cuadrática.
  Point3D quadraticBezier(const Point3D& p0, const Point3D& p1, const Point3D& p2, float t) const;
};