/*
  Leg.h
  Biblioteca para controlar una sola pata de 3-DOF del hexápodo.
  Contiene la matemática de la IK de la pata (Fase 1-2).
 */

#pragma once // Evita que este archivo se incluya múltiples veces

// Incluimos las bibliotecas que necesitamos
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class Leg {
public: // Funciones que podemos llamar desde fuera
    

    // Se llama cuando creamos un objeto Leg
    Leg(); 

    // --- Funciones de Configuración ---
    
    //Inicializa la pata con sus conexiones y geometría.
    void init(Adafruit_PWMServoDriver* pwm, uint8_t canalCadera, uint8_t canalRodilla, uint8_t canalTobillo, double L1, double L2, double L3);


    //Calcula los ángulos de servo para un objetivo local (x,y,z).
    bool moveTo(double x_l, double y_l, double z_l);
    
    // Aplica los ángulos calculados a los servos físicos.
    void updateServos();

    // --- Variables Públicas (para depuración) ---
    double anguloCadera_deg, anguloRodilla_deg, anguloTobillo_deg;


private: // Variables y funciones internas
    
    // --- Hardware ---
    Adafruit_PWMServoDriver* _pwm; // Puntero al controlador
    uint8_t _canalCadera, _canalRodilla, _canalTobillo;
    const int _PULSO_MIN = 600; 
    const int _PULSO_MAX = 2600; 

    // --- Geometría ---
    double _L1_Coxa, _L2_Femur, _L3_Tibia;

    // --- Estado (Ángulos) ---
    // Ángulos objetivo (calculados por moveTo)
    double _anguloCadera_rad, _anguloRodilla_rad, _anguloTobillo_rad;
    // Ángulos actuales (enviados al servo, con suavizado)
    double _caderaSuavizada_deg, _rodillaSuavizada_deg, _tobilloSuavizado_deg;
    double _factorSuavizado = 0.2; 

    
    //Función interna para mover un servo.

    void _moverServo(uint8_t canal, double angulo_deg);
};