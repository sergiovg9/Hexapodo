/*
  Leg.cpp
  Cinemática inversa y control de una pata.
*/

#include "Leg.h"
#include <Math.h>

// Constructor
Leg::Leg() {

  // Ángulos matemáticos
  _anguloCadera_rad = 0.0;
  _anguloRodilla_rad = 0.0;
  _anguloTobillo_rad = 0.0;

  // Ángulos iniciales de los servos
  _caderaSuavizada_deg = 90.0;
  _rodillaSuavizada_deg = 90.0;
  _tobilloSuavizado_deg = 90.0;

  // Suavizado
  _factorSuavizado = 0.2;
}

// Inicialización
void Leg::init(Adafruit_PWMServoDriver* pwm, uint8_t canalCadera, uint8_t canalRodilla, uint8_t canalTobillo, double L1, double L2, double L3) {

  _pwm = pwm;

  _canalCadera = canalCadera;
  _canalRodilla = canalRodilla;
  _canalTobillo = canalTobillo;

  _L1_Coxa = L1;
  _L2_Femur = L2;
  _L3_Tibia = L3;
}

// Cinemática inversa
bool Leg::moveTo(double x_l, double y_l, double z_l) {
  // q1: giro horizontal de la pata
  double q1 = atan2(y_l, x_l);

  // Llevar el problema al plano del fémur y tibia
  double r = sqrt(x_l * x_l + y_l * y_l);

  double x = r - _L1_Coxa;
  double z = z_l;

  // q3: ángulo entre fémur y tibia
  double cos_q3 = (x * x + z * z - _L2_Femur * _L2_Femur - _L3_Tibia * _L3_Tibia) / (2.0 * _L2_Femur * _L3_Tibia);

  double q3 = -acos(cos_q3);

  // q2: ángulo del fémur
  double q2 = atan2(z, x) - atan2(_L3_Tibia * sin(q3), _L2_Femur + _L3_Tibia * cos(q3));

  // Guardar ángulos matemáticos
  _anguloCadera_rad = q1;
  _anguloRodilla_rad = q2;
  _anguloTobillo_rad = q3;

  // Solo para consulta o depuración
  anguloCadera_deg = q1 * RAD_TO_DEG;
  anguloRodilla_deg = q2 * RAD_TO_DEG;
  anguloTobillo_deg = q3 * RAD_TO_DEG;


  return true;
}

// Actualización de servos
void Leg::updateServos() {

  // Convertir los ángulos matemáticos a grados
  double q1_deg = _anguloCadera_rad * RAD_TO_DEG;
  double q2_deg = _anguloRodilla_rad * RAD_TO_DEG;
  double q3_deg = _anguloTobillo_rad * RAD_TO_DEG;

  // Convertir ángulo matemático a ángulo físico del servo
  double servoCadera = 90.0 + q1_deg;

  double servoRodilla = 90.0 + q2_deg;

  // El servo del tobillo está físicamente invertido.
  double servoTobillo = 90.0 - q3_deg;

  // Suavizado
  _caderaSuavizada_deg = _factorSuavizado * servoCadera + (1.0 - _factorSuavizado) * _caderaSuavizada_deg;
  _rodillaSuavizada_deg = _factorSuavizado * servoRodilla + (1.0 - _factorSuavizado) * _rodillaSuavizada_deg;
  _tobilloSuavizado_deg = _factorSuavizado * servoTobillo + (1.0 - _factorSuavizado) * _tobilloSuavizado_deg;

  // Mover servos
  _moverServo(_canalCadera, _caderaSuavizada_deg);
  _moverServo(_canalRodilla, _rodillaSuavizada_deg);
  _moverServo(_canalTobillo, _tobilloSuavizado_deg);
}

// Enviar ángulo al PCA9685
void Leg::_moverServo(uint8_t canal, double angulo_deg) {

  // Convertir grados a microsegundos
  double pulso = _PULSO_MIN + (angulo_deg / 180.0) * (_PULSO_MAX - _PULSO_MIN);

  // Convertir microsegundos a cuentas del PCA9685
  uint16_t valorPWM = (uint16_t)(pulso * 50.0 * 4096.0 / 1000000.0);

  _pwm->setPWM(canal, 0, valorPWM);
}