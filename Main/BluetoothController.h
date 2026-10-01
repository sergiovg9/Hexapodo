#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>

#include "Controller.h"


class BluetoothController {

public:

  // CONSTRUCTOR
  BluetoothController() {
    _controller = nullptr;
    _buffer = "";
    _giroActivo = false;
    _estadoAnteriorMovimiento = false;
  }


  // BEGIN
  void begin(Controller* controller) {

    _controller = controller;

    // Nombre que aparecerá al buscar el ESP32 por Bluetooth.
    _bluetooth.begin("Hexapod");
  }


  // UPDATE
  void update() {

    while (_bluetooth.available()) {

      char caracter = _bluetooth.read();

      // Fin de comando.
      if (caracter == '\n') {

        procesarComando(_buffer);

        _buffer = "";
      }

      // Ignorar retorno de carro.
      else if (caracter != '\r') {

        _buffer += caracter;
      }
    }
  }


private:

  BluetoothSerial _bluetooth;

  Controller* _controller;

  String _buffer;


  // PROCESAR COMANDO
  void procesarComando(String comando) {

    if (_controller == nullptr) {
      return;
    }

    comando.trim();

    if (comando.length() < 3) {
      return;
    }

    // Buscar ":".
    int posicionDosPuntos = comando.indexOf(':');

    if (posicionDosPuntos < 0) {
      return;
    }

    // Canal.
    char canal = comando.charAt(0);

    // Valor después de ":".
    String valorTexto = comando.substring(posicionDosPuntos + 1);
    valorTexto.trim();

    // DIRECCIÓN
    if (canal == 'D' || canal == 'd') {

      float direccion = valorTexto.toFloat();

      _controller->setDirection(direccion);
    }

    // GIRO DERECHA
    else if (canal == 'R' || canal == 'r') {

      bool activo = convertirBooleano(valorTexto);

      if (activo) {

        // Guardar estado antes de comenzar el giro.
        if (!_giroActivo) {
          _estadoAnteriorMovimiento = _controller->isMovementEnabled();
          _giroActivo = true;
        }

        _controller->setMovementMode(MotionPlanner::TURN_RIGHT);
      }

      else {

        // Regresar al estado anterior.
        _giroActivo = false;

        if (_estadoAnteriorMovimiento) {
          // Estaba caminando.
          _controller->setMovementMode(MotionPlanner::WALK);
        }

        else {
          // Estaba en HOME.
          _controller->setMovementEnabled(false);
        }
      }
    }

    // GIRO IZQUIERDA
    else if (canal == 'L' || canal == 'l') {

      bool activo = convertirBooleano(valorTexto);

      if (activo) {

        _controller->setMovementMode(MotionPlanner::TURN_LEFT);

      } else {

        actualizarModoGiro();
      }
    }

    // GIRO IZQUIERDA
    else if (canal == 'L' || canal == 'l') {

      bool activo = convertirBooleano(valorTexto);

      if (activo) {

        // Guardar estado antes de comenzar el giro.
        if (!_giroActivo) {
          _estadoAnteriorMovimiento = _controller->isMovementEnabled();
          _giroActivo = true;
        }

        _controller->setMovementMode(MotionPlanner::TURN_LEFT);
      }

      else {

        // Regresar al estado anterior.
        _giroActivo = false;

        if (_estadoAnteriorMovimiento) {
          // Estaba caminando.
          _controller->setMovementMode(MotionPlanner::WALK);
        }

        else {
          // Estaba en HOME.
          _controller->setMovementEnabled(false);
        }
      }
    }

    // ALTURA
    else if (canal == 'A' || canal == 'a') {

      float altura = valorTexto.toFloat();

      _controller->setHeight(altura);
    }

    // VELOCIDAD
    else if (canal == 'V' || canal == 'v') {

      float velocidad = valorTexto.toFloat();

      _controller->setSpeed(velocidad);
    }
  }

  // ACTUALIZAR MODO DESPUÉS DE SOLTAR R/L
  void actualizarModoGiro() {

    // Si ninguno de los dos botones está activo, regresar a WALK.
    _controller->setMovementMode(MotionPlanner::WALK);
  }

  // CONVERTIR TRUE/FALSE
  / bool convertirBooleano(String valor) {

    valor.trim();
    valor.toLowerCase();

    return valor == "true";
  }
};