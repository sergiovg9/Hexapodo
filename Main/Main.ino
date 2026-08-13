#include <Wire.h>

#include "Robot.h"
#include "Workspace.h"
#include "MotionPlanner.h"
#include "Gait.h"
//Temporal(?)
#include "ValidacionServos.h"

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

// Indica si la marcha ya fue iniciada.
bool marchaIniciada = false;
// ==========Control TEMPORAL de giro =======
const uint8_t PIN_GIRO_IZQUIERDA = 27;
const uint8_t PIN_GIRO_DERECHA = 26;

// La interrupción solamente avisa que cambió un pin.
volatile bool cambioModo = true;

// Modo actual del robot.
MotionPlanner::MovementMode modoMovimiento = MotionPlanner::WALK;


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

// INTERRUPCIÓN DE CAMBIO DE MODO
void IRAM_ATTR interrupcionModo() {
  // Solo avisamos al loop.
  cambioModo = true;
}

// SETUP
void setup() {
  // Iniciar I2C.
  Wire.begin();


  // Pines TEMPORALES de control de giro.
  pinMode(PIN_GIRO_IZQUIERDA, INPUT_PULLDOWN);
  pinMode(PIN_GIRO_DERECHA, INPUT_PULLDOWN);
  // Detectar tanto cuando se presiona como cuando se suelta.
  attachInterrupt(digitalPinToInterrupt(PIN_GIRO_IZQUIERDA), interrupcionModo, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GIRO_DERECHA), interrupcionModo, CHANGE);


  delay(500);

  // Inicializar robot y PCA9685.
  robot.begin();

  //======Temporal(?)
  validacionServos.begin(
    robot.getPcaA(),
    robot.getPcaB());

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

// ACTUALIZAR MODO DE MOVIMIENTO
void actualizarModoMovimiento() {

  // Solo entrar cuando una interrupción indicó un cambio.
  if (!cambioModo) {
    return;
  }

  // Limpiar bandera.
  cambioModo = false;

  // Leer los pines una sola vez.
  bool izquierda = digitalRead(PIN_GIRO_IZQUIERDA);
  bool derecha = digitalRead(PIN_GIRO_DERECHA);

  MotionPlanner::MovementMode nuevoModo;

  if (izquierda && !derecha) {
    // D27 HIGH -> giro izquierda.
    nuevoModo = MotionPlanner::TURN_LEFT;
  } else if (derecha && !izquierda) {
    // D26 HIGH -> giro derecha.
    nuevoModo = MotionPlanner::TURN_RIGHT;
  } else {
    // Ninguno activo, o ambos activos: volver a caminata normal.
    nuevoModo = MotionPlanner::WALK;
  }

  // Solo recalcular si realmente cambió el modo.
  if (nuevoModo != modoMovimiento) {
    modoMovimiento = nuevoModo;
    nuevaTrayectoria = true;
  }
}

void loop() {
  //========Temporal?
  if (digitalRead(25) == HIGH) {
    validacionServos.update();
    return;
  }

  // Revisar cambio de caminata/giro.
  actualizarModoMovimiento();


  // ACTUALIZAR TRAYECTORIA

  // Solo recalcular cuando cambie dirección o altura.
  if (nuevaTrayectoria) {
    bool trayectoriaOK = motionPlanner.calculateTrajectories(direccionRobot, alturaRobot, modoMovimiento);

    // Solo usar la nueva trayectoria si es válida.
    if (trayectoriaOK) {
      // Iniciar Gait solamente la primera vez.
      if (!marchaIniciada) {
        gait.start();
        marchaIniciada = true;
      }
      // En cambios posteriores NO reiniciar Gait.
      nuevaTrayectoria = false;
    }
  }

  // MARCHA
  // Recorrer las trayectorias ya calculadas.
  gait.update();
  // Suavizar y actualizar los servos.
  robot.update();
}