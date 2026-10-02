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
  }


  // BEGIN
  void begin(Controller* controller) {

    _controller = controller;

    _bluetooth.begin("Hexapod");
  }


  // UPDATE
  void update() {

    // Leer solamente los datos disponibles.
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

    // Obtener canal.
    char canal = comando.charAt(0);

    // Buscar ':'.
    int posicionDosPuntos = comando.indexOf(':');

    if (posicionDosPuntos < 0) {
      return;
    }

    // Obtener valor como texto.
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

        _controller->setMovementMode(MotionPlanner::TURN_RIGHT);

      } else {

        // Al soltar R, volver a WALK.
        _controller->setMovementMode(MotionPlanner::WALK);
      }
    }

    // GIRO IZQUIERDA

    else if (canal == 'L' || canal == 'l') {

      bool activo = convertirBooleano(valorTexto);

      if (activo) {

        _controller->setMovementMode(MotionPlanner::TURN_LEFT);

      } else {

        // Al soltar L, volver a WALK.
        _controller->setMovementMode(MotionPlanner::WALK);
      }
    }

    // MARCHA / HOME

    else if (canal == 'H' || canal == 'h') {

      bool activo = convertirBooleano(valorTexto);

      _controller->setMovementEnabled(!activo);
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


  // CONVERTIR TRUE / FALSE
  bool convertirBooleano(String valor) {

    valor.trim();
    valor.toLowerCase();

    return valor == "true";
  }
};