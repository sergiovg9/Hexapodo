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

  // CAMBIO ENTRE MARCHA Y HOME
  if (controller.hasMovementStateChanged()) {

    // COMENZAR MARCHA
    if (controller.isMovementEnabled()) {

      bool trayectoriaOK = motionPlanner.calculateTrajectories(controller.getDirection(), controller.getHeight(), controller.getMovementMode());

      if (trayectoriaOK) {
        // Comenzar un nuevo ciclo de marcha.
        gait.start();
        controller.clearChanged();
      }
    }

    // REGRESAR A HOME
    else {
      // Detener completamente la marcha.
      gait.stop();

      // Mandar las seis patas a HOME.
      robot.goHome(&workspaceManager, alturaHome);
    }

    controller.clearMovementStateChanged();
  }

  // ACTUALIZAR TRAYECTORIA DURANTE LA MARCHA
  if (controller.isMovementEnabled()) {

    if (controller.hasChanged()) {

      bool trayectoriaOK = motionPlanner.calculateTrajectories(controller.getDirection(), controller.getHeight(), controller.getMovementMode());

      if (trayectoriaOK) {
        controller.clearChanged();
      }
    }

    // MARCHA
    gait.update(controller.getSpeed());
  }

  // Actualizar servos.
  robot.update();
}