#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class ValidacionServos {
public:

  ValidacionServos() {
    _pcaA = nullptr;
    _pcaB = nullptr;
    _pcaC = nullptr;
  }

  // Recibe las PCA que YA fueron inicializadas por Robot
  void begin(Adafruit_PWMServoDriver* pcaA, Adafruit_PWMServoDriver* pcaB, Adafruit_PWMServoDriver* pcaC) {
    _pcaA = pcaA;
    _pcaB = pcaB;
    _pcaC = pcaC;

    pinMode(25, INPUT);

    // Asegurar que al inicio ningún servo reciba señal
    apagarTodos();
  }

  // Ejecuta la prueba mientras D25 esté en HIGH
  void update() {
    if (digitalRead(25) == LOW) {
      apagarTodos();
      return;
    }

    // Probar los 18 servos uno por uno
    for (int servo = 0; servo < 18; servo++) {
      // Antes de empezar, todos apagados
      apagarTodos();

      if (!pinActivo())
        return;

      // -90°
      moverServo(servo, -90);
      if (!esperar(500))
        return;

      // 0°
      moverServo(servo, 0);
      if (!esperar(500))
        return;

      // 90°
      moverServo(servo, 90);
      if (!esperar(500))
        return;

      // Regresar a 0°
      moverServo(servo, 0);
      if (!esperar(500))
        return;

      // DEJAR DE MANDAR SEÑAL A ESTE SERVO
      apagarServo(servo);

      // Pequeña pausa antes del siguiente
      if (!esperar(300))
        return;
    }

    // Al terminar, absolutamente todos sin PWM
    apagarTodos();
  }


private:

  Adafruit_PWMServoDriver* _pcaA;
  Adafruit_PWMServoDriver* _pcaB;
  Adafruit_PWMServoDriver* _pcaC;

  static constexpr int SERVO_MIN = 600;
  static constexpr int SERVO_MAX = 2600;
  static constexpr int SERVO_FREQ = 50;


  // ==============================
  // Comprobar D25
  // ==============================
  bool pinActivo() {
    if (digitalRead(25) == LOW) {
      apagarTodos();
      return false;
    }

    return true;
  }


  // ==============================
  // Espera comprobando D25
  // ==============================
  bool esperar(unsigned long tiempo) {
    unsigned long inicio = millis();

    while (millis() - inicio < tiempo) {
      // Si D25 pierde voltaje,
      // apagar inmediatamente todos los servos
      if (digitalRead(25) == LOW) {
        apagarTodos();
        return false;
      }

      delay(5);
    }

    return true;
  }


  // ==============================
  // Mover un servo
  // ==============================
  void moverServo(int servo, float angulo) {
    if (_pcaA == nullptr || _pcaB == nullptr || _pcaC == nullptr)
      return;

    angulo = constrain(angulo, -90.0f, 90.0f);

    int us =
      SERVO_MIN + (angulo + 90.0f) * (SERVO_MAX - SERVO_MIN) / 180.0f;

    uint16_t ticks =
      (uint16_t)((us * SERVO_FREQ * 4096L) / 1000000L);

    // Servos 0 - 5 -> PCA A
    if (servo < 6) {
      _pcaA->setPWM(servo, 0, ticks);
    }

    // Servos 6 - 11 -> PCA B
    else if (servo < 12) {
      _pcaB->setPWM(servo - 6, 0, ticks);
    }

    // Servos 12 - 17 -> PCA C
    else {
      _pcaC->setPWM(servo - 12, 0, ticks);
    }
  }


  // ==============================
  // Apagar señal de UN servo
  // ==============================
  void apagarServo(int servo) {
    if (_pcaA == nullptr || _pcaB == nullptr || _pcaC == nullptr)
      return;

    if (servo < 6) {
      _pcaA->setPWM(servo, 0, 0);
    }

    else if (servo < 12) {
      _pcaB->setPWM(servo - 6, 0, 0);
    }

    else {
      _pcaC->setPWM(servo - 12, 0, 0);
    }
  }


  // ==============================
  // Apagar señal de TODOS
  // ==============================
  void apagarTodos() {
    if (_pcaA == nullptr || _pcaB == nullptr)
      return;

    for (int canal = 0; canal < 6; canal++) {
      _pcaA->setPWM(canal, 0, 0);
      _pcaB->setPWM(canal, 0, 0);
      _pcaC->setPWM(canal, 0, 0);
    }
  }
};