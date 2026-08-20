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

    // Nombre que aparecerá al buscar el ESP32 por Bluetooth.
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

    // Formato mínimo:
    // R: 255
    if (comando.length() < 3) {
      return;
    }

    char canal = comando.charAt(0);

    int posicionDosPuntos = comando.indexOf(':');

    if (posicionDosPuntos < 0) {
      return;
    }

    int valor = comando.substring(posicionDosPuntos + 1).toInt();

    // Limitar entrada.
    valor = constrain(valor, 0, 255);


    // DIRECCIÓN
    if (canal == 'R' || canal == 'r') {

      float direccion = convertirDireccion(valor);

      _controller->setDirection(direccion);
    }


    // ALTURA
    else if (canal == 'G' || canal == 'g') {

      float altura = convertirAltura(valor);

      _controller->setHeight(altura);
    }


    // VELOCIDAD
    else if (canal == 'B' || canal == 'b') {

      float velocidad = convertirVelocidad(valor);

      _controller->setSpeed(velocidad);
    }
  }


  // R -> DIRECCIÓN
  float convertirDireccion(int valor) {

    // 0   -> +180°
    // 127.5 -> 0°
    // 255 -> -180°
    float direccion = 180.0f - ((float)valor * 360.0f / 255.0f);

    // Redondear al múltiplo de 2° más cercano.
    direccion = roundf(direccion / 2.0f) * 2.0f;

    return direccion;
  }


  // G -> ALTURA
  float convertirAltura(int valor) {

    // 0   -> -200 mm
    // 255 -> -100 mm
    return -200.0f + ((float)valor * 100.0f / 255.0f);
  }


  // B -> VELOCIDAD
  float convertirVelocidad(int valor) {

    // 0   -> 0.1
    // 255 -> 1.0
    return 0.1f + ((float)valor * 0.9f / 255.0f);
  }
};