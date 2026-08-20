#include <Wire.h>

#include "Robot.h"
#include "Workspace.h"
#include "MotionPlanner.h"
#include "Gait.h"
#include "Controller.h"
// Temporal
#include "ValidacionServos.h"

// OBJETOS
Robot robot;
Workspace workspaceManager;
MotionPlanner motionPlanner;
Gait gait;
Controller controller;
// Temporal
ValidacionServos validacionServos;


// CONFIGURACIÓN
// Altura utilizada solamente para HOME.
float alturaHome = -110.0f;


// Estado de Gait.
// Indica si la marcha ya fue iniciada.
bool marchaIniciada = false;


// SETUP
void setup() {

  // Iniciar I2C.
  Wire.begin();

  // Inicializar robot y PCA9685.
  robot.begin();

  // TEMPORAL
  validacionServos.begin(robot.getPcaA(), robot.getPcaB());

  // Inicializar workspace.
  workspaceManager.begin();

  // Inicializar planificador.
  motionPlanner.begin(&robot, &workspaceManager);

  // Inicializar marcha.
  gait.begin(&robot, &motionPlanner);

  // Inicializar controlador.
  controller.begin();

  // HOME
  robot.goHome(&workspaceManager, alturaHome);
  // Mover suavemente hasta HOME.
  unsigned long tiempoInicio = millis();

  while (millis() - tiempoInicio < 2000) {

    robot.update();

    delay(10);
  }

  // Controller ya comienza con _changed = true,
  // por lo que la primera trayectoria se calculará
  // automáticamente en loop().
}


// LOOP
void loop() {

  // Validación TEMPORAL de servos
  if (digitalRead(25) == HIGH) {
    validacionServos.update();
    return;
  }

  // CONTROLLER: Procesar interrupciones y actualizar comandos.
  controller.update();

  // ACTUALIZAR TRAYECTORIA
  if (controller.hasChanged()) {

    bool trayectoriaOK = motionPlanner.calculateTrajectories(controller.getDirection(), controller.getHeight(), controller.getMovementMode());

    // Solamente aceptar el nuevo comando si MotionPlanner logró generar las trayectorias.
    if (trayectoriaOK) {
      // Iniciar Gait solamente una vez.
      if (!marchaIniciada) {
        gait.start();
        marchaIniciada = true;
      }

      // El comando ya fue procesado.
      controller.clearChanged();
    }
  }

  // MARCHA
  gait.update(controller.getSpeed());

  // Actualizar servos.
  robot.update();
}