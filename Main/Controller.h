#pragma once

#include <Arduino.h>
#include "MotionPlanner.h"

class Controller {
public:

  // CONSTRUCTOR
  Controller();

  // INICIALIZAR
  void begin();

  // Revisar interrupciones y actualizar comandos.
  void update();

  // CAMBIAR COMANDOS
  void setDirection(float direction);
  void setHeight(float height);
  void setSpeed(float speed);

  void setCommand(float direction, float height);

  // OBTENER COMANDOS
  float getDirection() const;
  float getHeight() const;
  float getSpeed() const;

  MotionPlanner::MovementMode getMovementMode() const;

  // CAMBIO DE TRAYECTORIA
  bool hasChanged() const;
  void clearChanged();


private:

  // COMANDO ACTUAL

  float _direction;
  float _height;
  float _speed;

  MotionPlanner::MovementMode _movementMode;

  // Dirección, altura o modo cambiaron.
  bool _changed;


  // PINES TEMPORALES DE CONTROL

  static const uint8_t PIN_GIRO_IZQUIERDA = 27;
  static const uint8_t PIN_GIRO_DERECHA   = 26;

  static const uint8_t PIN_DIRECCION_50 = 14;
  static const uint8_t PIN_ALTURA_110   = 12;
  static const uint8_t PIN_VELOCIDAD    = 13;


  // BANDERAS DE INTERRUPCIÓN

  volatile bool _cambioModo;
  volatile bool _cambioDireccion;
  volatile bool _cambioAltura;
  volatile bool _cambioVelocidad;


  // INTERRUPCIONES

  static void IRAM_ATTR interrupcionModo(void* arg);
  static void IRAM_ATTR interrupcionDireccion(void* arg);
  static void IRAM_ATTR interrupcionAltura(void* arg);
  static void IRAM_ATTR interrupcionVelocidad(void* arg);


  // PROCESAR CAMBIOS

  void actualizarModoMovimiento();
  void actualizarDireccion();
  void actualizarAltura();
  void actualizarVelocidad();


  // Normalizar dirección entre 0° y 360°.
  float normalizeDirection(float direction);
};