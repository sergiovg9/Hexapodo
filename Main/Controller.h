#pragma once

#include <Arduino.h>


class Controller {
public:
  // Constructor.
  Controller();

  // Inicializar el controlador.
  void begin();

  // Cambiar dirección global del robot. AVANCE NORMAL
  void setDirection(float direction);

  // Cambiar altura global del robot.
  void setHeight(float height);

  // Cambiar dirección y altura.
  void setCommand(float direction, float height);

  // Obtener dirección actual.
  float getDirection() const;

  // Obtener altura actual.
  float getHeight() const;

  // Indica si el comando cambió.
  bool hasChanged() const;

  // Indica que el nuevo comando ya fue procesado.
  void clearChanged();



private:
  
  // Dirección global del robot en grados.
  float _direction;

  // Altura global del robot en mm.
  float _height;

  // Indica que se necesita una nueva trayectoria.
  bool _changed;

  // Normaliza la dirección entre 0° y 360°.
  float normalizeDirection(float direction);
};