#pragma once

#include <Arduino.h>

#include "WorkspaceData.h"

class Workspace {
public:
  // Constructor.
  Workspace();

  // Inicializar.
  void begin();

  // Buscar línea para una altura y ángulo.
  bool getLine(float height, float angle, LineaTrabajo& line) const;



private:
  // Buscar altura más cercana.
  int findHeightIndex(float height) const;

  // Buscar ángulo más cercano.
  int findAngleIndex(float angle) const;

  // Normalizar ángulo entre 0° y 360°.
  float normalizeAngle(float angle) const;
};