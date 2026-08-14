#include "Workspace.h"
#include "WorkspaceData.h"

#include <math.h>

// CONSTRUCTOR
Workspace::Workspace() {
}

// BEGIN
void Workspace::begin() {
}

// OBTENER LÍNEA
bool Workspace::getLine(float height, float angle, LineaTrabajo& line) const {
  // Verificar altura.
  if (height < alturas[0] || height > alturas[NUM_ALTURAS - 1]) {
    return false;
  }

  // Normalizar ángulo entre 0° y 360°.
  angle = normalizeAngle(angle);

  // Buscar índices.
  int heightIndex = findHeightIndex(height);
  int angleIndex = findAngleIndex(angle);

  // Verificar índices.
  if (heightIndex < 0 || angleIndex < 0) {
    return false;
  }

  // Copiar línea encontrada.
  line = workspace[heightIndex][angleIndex];

  return true;
}


// BUSCAR ALTURA
int Workspace::findHeightIndex(float height) const {
  int bestIndex = 0;

  float bestDifference = fabs(height - alturas[0]);

  for (int i = 1; i < NUM_ALTURAS; i++) {
    float difference = fabs(height - alturas[i]);

    if (difference < bestDifference) {
      bestDifference = difference;
      bestIndex = i;
    }
  }

  return bestIndex;
}


// BUSCAR ÁNGULO
int Workspace::findAngleIndex(float angle) const {
  // Los ángulos están separados exactamente cada 2°:
  // 0, 2, 4, ..., 358.

  int index = (int)round(angle / 2.0f);

  // Ejemplo:
  // 359° / 2 = 179.5 -> round() = 180.
  //
  // Pero el índice 180 no existe.
  // 360° equivale nuevamente a 0°.
  if (index >= NUM_ANGULOS) {
    index = 0;
  }

  return index;
}


// NORMALIZAR ÁNGULO
float Workspace::normalizeAngle(float angle) const {
  // Mantener entre 0° y 360°.
  while (angle >= 360.0f) {
    angle -= 360.0f;
  }

  while (angle < 0.0f) {
    angle += 360.0f;
  }

  return angle;
}