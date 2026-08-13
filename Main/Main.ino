#include <Wire.h>

#include "Robot.h"
#include "Workspace.h"
#include "MotionPlanner.h"
#include "Gait.h"
//Temporal(?)
#include "ValidacionServos.h"

/*
#define PIN_GIRO_DERECHA 34
#define PIN_GIRO_IZQUIERDA 35
*/

// OBJETOS
Robot robot;
Workspace workspaceManager;
MotionPlanner motionPlanner;
Gait gait;
//Temporal(?)
ValidacionServos validacionServos;

// Valores predeterminados.
// Después Controller podrá modificar estas variables.
float direccionRobot = 50.0f;
float alturaRobot = -150.0f;

// Indica que es necesario calcular una nueva trayectoria.
bool nuevaTrayectoria = true;

// CONFIGURAR NUEVO COMANDO
// Esta función podrá ser llamada posteriormente por Controller.
void setCommand(float nuevaDireccion, float nuevaAltura) {
  // Evitar recalcular si el comando no cambió.
  if (nuevaDireccion == direccionRobot && nuevaAltura == alturaRobot) {
    return;
  }

  // Guardar nuevo comando.
  direccionRobot = nuevaDireccion;
  alturaRobot = nuevaAltura;

  // Solicitar un nuevo cálculo.
  nuevaTrayectoria = true;
}

// SETUP
void setup() {
  // Iniciar I2C.
  Wire.begin();

  delay(500);

  // Inicializar robot y PCA9685.
  robot.begin();

  //======Temporal(?)
  validacionServos.begin(
    robot.getPcaA(),
    robot.getPcaB());

  /*
  //Giro axial
  pinMode(PIN_GIRO_DERECHA, INPUT);
  pinMode(PIN_GIRO_IZQUIERDA, INPUT);
  */

  // Inicializar workspace.
  workspaceManager.begin();

  // Inicializar planificador.
  motionPlanner.begin(&robot, &workspaceManager);

  // Inicializar marcha.
  gait.begin(&robot, &motionPlanner);

  // Establecer posición HOME.
  robot.goHome();

  // Mover suavemente hasta HOME.
  unsigned long tiempoInicio = millis();

  while (millis() - tiempoInicio < 2000) {
    robot.update();
    delay(10);
  }


  // La primera trayectoria se calculará en loop().
  nuevaTrayectoria = true;
}

void loop() {
  //========Temporal?
  if (digitalRead(25) == HIGH) {
    validacionServos.update();
    return;
  }


  // ACTUALIZAR TRAYECTORIA

  // Solo recalcular cuando cambie dirección o altura.
  if (nuevaTrayectoria) {
    bool trayectoriaOK = motionPlanner.calculateTrajectories(direccionRobot, alturaRobot);

    // Solo usar la nueva trayectoria si es válida.
    if (trayectoriaOK) {
      // Reiniciar la marcha con la nueva trayectoria.
      gait.start();

      // Ya no es necesario recalcular.
      nuevaTrayectoria = false;
    }
  }

  // MARCHA
  // Recorrer las trayectorias ya calculadas.
  gait.update();
  // Suavizar y actualizar los servos.
  robot.update();
}