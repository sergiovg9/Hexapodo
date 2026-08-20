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

  // ESTADO DE MARCHA
  bool isMovementEnabled() const;
  bool hasMovementStateChanged() const;
  void clearMovementStateChanged();


private:

  // COMANDO ACTUAL

  float _direction;
  float _height;
  float _speed;

  MotionPlanner::MovementMode _movementMode;

  // Dirección, altura o modo cambiaron.
  bool _changed;
  // true  -> robot caminando
  // false -> robot en HOME
  bool _movementEnabled;
  // Indica que se cambió entre marcha y HOME.
  bool _movementStateChanged;


  // PINES TEMPORALES DE CONTROL
  static const uint8_t PIN_GIRO_IZQUIERDA = 27;
  static const uint8_t PIN_GIRO_DERECHA = 26;

  static const uint8_t PIN_DIRECCION_50 = 14;
  static const uint8_t PIN_ALTURA_110 = 12;
  static const uint8_t PIN_VELOCIDAD = 13;

  static const uint8_t PIN_MARCHA = 33;

  // BANDERAS DE INTERRUPCIÓN
  volatile bool _cambioModo;
  volatile bool _cambioDireccion;
  volatile bool _cambioAltura;
  volatile bool _cambioVelocidad;
  volatile bool _cambioMarcha;

  // INTERRUPCIONES
  static void IRAM_ATTR interrupcionModo(void* arg);
  static void IRAM_ATTR interrupcionDireccion(void* arg);
  static void IRAM_ATTR interrupcionAltura(void* arg);
  static void IRAM_ATTR interrupcionVelocidad(void* arg);
  static void IRAM_ATTR interrupcionMarcha(void* arg);

  // PROCESAR CAMBIOS
  void actualizarModoMovimiento();
  void actualizarDireccion();
  void actualizarAltura();
  void actualizarVelocidad();
  void actualizarMarcha();

  // Normalizar dirección entre 0° y 360°.
  float normalizeDirection(float direction);
};