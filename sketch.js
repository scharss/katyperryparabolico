// --- Variables Físicas y Constantes ---

// Declarar variables de estado en 2D que serán inicializadas en setup()
let x;           // Posición horizontal (metros)
let y;           // Altitud (metros)
let vx;          // Velocidad horizontal (m/s, positivo hacia la derecha conceptual)
let vy;          // Velocidad vertical (m/s, positivo hacia arriba)
let ax;          // Aceleración horizontal (m/s^2)
let ay;          // Aceleración vertical (m/s^2)

let currentMass; // Masa actual de la nave (cambia tras separación) (kg)
let rocketMass;  // Masa del cohete completo (kg)
let capsuleMass; // Masa solo de la cápsula (kg)
let g;           // Aceleración debido a la gravedad (m/s^2)
let thrust;      // Empuje del motor (N) - Solo en fase Launch

let currentDragCoefficient; // Coeficiente de resistencia actual
let dragCoefficientRocket; // Coeficiente de resistencia para el cohete (Launch)
let dragCoefficientCapsule; // Coeficiente de resistencia para la cápsula (FreeFlight)
let dragCoefficientDrogues; // Coeficiente de resistencia para paracaídas de freno (Drogues)
let dragCoefficientMainChutes; // Coeficiente de resistencia para paracaídas principal (MainChutes/Landed)

let currentReferenceArea; // Área de referencia actual (m^2)
let referenceAreaRocket; // Área de referencia para resistencia (m^2) - Cohete
let referenceAreaCapsule; // Área de referencia para resistencia (m^2) - Cápsula
let referenceAreaDrogues; // Área efectiva con paracaídas de freno (m^2)
let referenceAreaMainChutes; // Área efectiva con paracaídas principal (m^2)

// Inclinación inicial sutil para generar una trayectoria parabólica (calcular en setup)
let launchTiltRadians; // Variable declarada, valor asignado en setup()
const LAUNCH_TILT_DEGREES_CONST = 0.5; // Pequeña inclinación en grados

// Modelo de Densidad del Aire (Simple exponencial)
let rho0;        // Densidad del aire a nivel del mar (kg/m^3)
let scaleHeight; // Altura de escala atmosférica (metros)
let airDensity;  // Densidad del aire calculada en función de la altitud

let maxQ = 0; // Máxima presión dinámica (Max Q)

// Variables conceptuales/visuales para descenso
let surfaceTemperatureConcept = "N/A"; // Representación conceptual de temperatura
let instantaneousTerminalVelocity = 0; // Velocidad terminal instantánea conceptual (magnitud de velocidad horizontal+vertical)


// --- Variables de Simulación y Estado ---
// Fases: "Ready", "Launch", "CapsuleFreeFlightAscent", "CapsuleFreeFlightDescent", "DrogueParachutes", "MainChutes", "Landed"
let phase;
let timeInPhase; // Tiempo transcurrido en la fase actual (segundos simulados)
let totalTime;   // Tiempo total transcurrido (segundos simulados)
let maxAltitudeReached; // Altitud máxima alcanzada (metros)
let maxDownrangeReached; // Alcance horizontal máximo (metros)
let gForce;      // Fuerza G actual

// --- Variables de Visualización ---
// Altura del panel superior para datos
const DATA_PANEL_HEIGHT = 270; // Aumentado para más datos 2D

// Área de simulación en píxeles (calculada en setup)
let simAreaXMin, simAreaXMax, simAreaYMin, simAreaYMax;

const MIN_DISPLAY_ALTITUDE_BUFFER_Y = 100; // Altitud mínima extra visible debajo de 0m
const DISPLAY_ALTITUDE_BUFFER_RATIO = 1.2; // Factor para mostrar un poco más que la altitud máxima

// Rango horizontal total a mostrar en la simulación (metros).
// Ahora esto define el "ancho" de la ventana de visualización que seguirá a la nave.
const HORIZONTAL_DISPLAY_RANGE = 6000; // Aumentado a 6km para ver más alcance

let gForceBarHeight = 150; // Altura del indicador de G-force en píxeles
let gForceScale = 6; // Escala para la barra de G-force (maxG para la visualización).

let naveWidthPixels = 15; // Ancho visual de la nave
let naveHeightPixels = 30; // Altura visual de la nave

// Estela de la trayectoria
let trajectoryPoints = [];
const MAX_TRAJECTORY_POINTS = 1500; // Limitar el número de puntos para rendimiento

// Colores (Declarar, inicializar en setup)
let backgroundColor; // Un solo color de fondo (oscuro)
let groundColor; // Color del suelo
let parachuteColor; // Color del paracaídas
let karmanLineColor; // Color para Línea Kármán
let trajectoryColor; // Color para la estela


// Colores para el texto (SIEMPRE visibles y fijos - CLAROS)
let textColorPrimary; // Color para texto principal (Blanco)
let textColorSecondary; // Color para texto secundario/alternativo (Gris claro)


// --- Factor de Velocidad de Simulación ---
// Queremos simular ~12 minutos (~720 segundos) en ~60-70 segundos de tiempo real.
const SIMULATION_SPEED_FACTOR = 11;


// --- CONSTANTES DE VUELO (Hardcoded) ---
const GRAVITY_CONST = 9.81;         // m/s^2

// Masa - Dividimos la masa total entre booster y cápsula
const ROCKET_MASS_CONST = 10000;    // kg (masa inicial del cohete completo)
const CAPSULE_MASS_CONST = 2000;    // kg (masa de la cápsula, aprox 20% del total)

// Empuje - Ajustado ligeramente
const LAUNCH_THRUST_CONST = 550000; // N (550 kN)

// --- Aerodynamic Constants ---
// Ajustados para simular arrastre en diferentes fases
const DRAG_COEFFICIENT_ROCKET_CONST = 0.28;
const DRAG_COEFFICIENT_CAPSULE_CONST = 1.0; // Mayor que el cohete, forma menos aerodinámica
const DRAG_COEFFICIENT_DROGUES_CONST = 1.8; // Aumenta significativamente con drogues
const DRAG_COEFFICIENT_MAIN_CHUTES_CONST = 2.2; // Muy alto con paracaídas principal

const REF_AREA_ROCKET_CONST = 10;   // m^2
const REF_AREA_CAPSULE_CONST = 7;   // m^2 (más pequeña que el cohete)
const REF_AREA_DROGUES_MULTIPLIER = 5; // Multiplicador para área efectiva con drogues
const REF_AREA_MAIN_CHUTES_MULTIPLIER = 30; // Multiplicador para área efectiva con paracaídas principal

// --- Atmospheric Model ---
const ATMOSPHERE_RHO0_CONST = 1.225; // Densidad del aire a nivel del mar
const ATMOSPHERE_SCALE_HEIGHT_CONST = 8000; // Altura de escala atmosférica (metros)

const ATMOSPHERE_MIN_DENSITY_ALTITUDE_CONST = 150000; // Nivel conceptual para densidad mínima muy baja
const ATMOSPHERE_MIN_DENSITY_MULTIPLIER = 0.0001; // Multiplicador para la densidad mínima


// --- Flight Sequence Parameters (Tiempos/Altitudes Simulados) ---
const LAUNCH_DURATION_SIMULATED_CONST = 70; // ~70 segundos de motor simulados (hasta MECO/Separación)

// Altitudes de despliegue de paracaídas (simuladas)
const DROGUES_DEPLOY_ALTITUDE_CONST = 7000; // ~7000 metros simulados para drogues
const MAIN_CHUTES_DEPLOY_ALTITUDE_CONST = 1500; // ~1500 metros simulados para paracaídas principal

// --- Altitud Límite Explícita (Línea de Kármán) ---
const KARMAN_LINE_ALT_LIMIT = 100000; // 100 km

// --- Configuración Inicial ---
function setup() {
  createCanvas(800, 800); // Aumentar el tamaño horizontal y vertical
  frameRate(60); // Intentar 60 FPS para simulación más fluida

  // Definir el área de simulación en píxeles
  simAreaXMin = 0;
  simAreaXMax = width;
  simAreaYMin = DATA_PANEL_HEIGHT + MIN_DISPLAY_ALTITUDE_BUFFER_Y;
  simAreaYMax = height - MIN_DISPLAY_ALTITUDE_BUFFER_Y;


  // --- Inicializar variables físicas y constantes DENTRO de setup ---
  g = GRAVITY_CONST;

  // Masas
  rocketMass = ROCKET_MASS_CONST;
  capsuleMass = CAPSULE_MASS_CONST;

  thrust = LAUNCH_THRUST_CONST;

  // Coeficientes de arrastre
  dragCoefficientRocket = DRAG_COEFFICIENT_ROCKET_CONST;
  dragCoefficientCapsule = DRAG_COEFFICIENT_CAPSULE_CONST;
  dragCoefficientDrogues = DRAG_COEFFICIENT_DROGUES_CONST;
  dragCoefficientMainChutes = DRAG_COEFFICIENT_MAIN_CHUTES_CONST;

  // Áreas de referencia
  referenceAreaRocket = REF_AREA_ROCKET_CONST;
  referenceAreaCapsule = REF_AREA_CAPSULE_CONST;
  referenceAreaDrogues = referenceAreaCapsule * REF_AREA_DROGUES_MULTIPLIER;
  referenceAreaMainChutes = referenceAreaCapsule * REF_AREA_MAIN_CHUTES_MULTIPLIER;

  // Modelo atmosférico
  rho0 = ATMOSPHERE_RHO0_CONST;
  scaleHeight = ATMOSPHERE_SCALE_HEIGHT_CONST;

  // Calcular inclinación inicial
  launchTiltRadians = radians(LAUNCH_TILT_DEGREES_CONST);


  // --- Inicializar variables de color DENTRO de setup ---
  backgroundColor = color(10, 10, 70); // Un solo color de fondo oscuro (Espacio/Noche)
  groundColor = color(100, 80, 50); // Color del suelo
  parachuteColor = color(255, 255, 255, 200); // Color blanco translúcido para paracaídas
  karmanLineColor = color(255, 100, 0); // Naranja para Kármán línea
  trajectoryColor = color(0, 200, 255, 150); // Cian translúcido para la estela


  // Colores para el texto (SIEMPRE visibles y fijos - CLAROS)
  textColorPrimary = color(255); // Blanco
  textColorSecondary = color(200); // Gris claro;


  // Inicializar variables de estado
  resetSimulation();

  // No se crean elementos DOM para controles.
}

// --- Bucle Principal de Dibujo y Simulación ---
function draw() {
  // Calculate real delta time in seconds
  let dt_real = deltaTime / 1000.0;
  // Calculate simulated delta time based on speed factor
  let dt_sim = dt_real * SIMULATION_SPEED_FACTOR;


  // --- Lógica de Simulación (Actualizar variables físicas) ---
  if (phase !== "Ready" && phase !== "Landed") {
      totalTime += dt_sim; // Use simulated time
      timeInPhase += dt_sim; // Use simulated time

      // Calcular densidad del aire en función de la altitud (modelo simple exponencial)
      let baseAirDensity = rho0 * exp(-y / scaleHeight);

      // Asegurarse de que haya una densidad mínima muy baja a altitudes muy altas
      let minAirDensityAtHighAlt = rho0 * exp(-ATMOSPHERE_MIN_DENSITY_ALTITUDE_CONST / scaleHeight) * ATMOSPHERE_MIN_DENSITY_MULTIPLIER;
      airDensity = max(baseAirDensity, minAirDensityAtHighAlt);
      airDensity = max(airDensity, 1e-15); // Evitar valores subnormales


      // Determinar Cd, Área de referencia y Masa actual según la fase
      switch (phase) {
          case "Launch":
              currentDragCoefficient = dragCoefficientRocket;
              currentReferenceArea = referenceAreaRocket;
              currentMass = rocketMass;
              // Calcular Max Q durante el ascenso propulsado
              let vMagnitudeLaunch = sqrt(vx*vx + vy*vy);
              let dynamicPressure = 0.5 * airDensity * vMagnitudeLaunch * vMagnitudeLaunch;
              if (dynamicPressure > maxQ) {
                  maxQ = dynamicPressure;
              }
              break;
          case "CapsuleFreeFlightAscent": // Post-separación, subiendo sin motor
          case "CapsuleFreeFlightDescent": // Bajando sin paracaídas (Reentrada conceptual)
              currentDragCoefficient = dragCoefficientCapsule;
              currentReferenceArea = referenceAreaCapsule;
              currentMass = capsuleMass; // Masa de la cápsula después de separación
              break;
          case "DrogueParachutes": // Descenso con drogues
              currentDragCoefficient = dragCoefficientDrogues;
              currentReferenceArea = referenceAreaDrogues;
              currentMass = capsuleMass;
              break;
          case "MainChutes": // Descenso con paracaídas principal
              currentDragCoefficient = dragCoefficientMainChutes;
              currentReferenceArea = referenceAreaMainChutes;
              currentMass = capsuleMass;
              break;
          default: // Should not happen, but fallback
               // Si llega aquí, usar valores de cápsula por defecto
              currentDragCoefficient = dragCoefficientCapsule;
              currentReferenceArea = referenceAreaCapsule;
              currentMass = capsuleMass;
               break;
      }

      // --- Calcular Fuerzas ---
      let forceX = 0;
      let forceY = -currentMass * g; // Gravedad siempre hacia abajo

      // Empuje (solo en Launch, vector inclinado)
      if (phase === "Launch") {
          // Aplicar thrust con la pequeña inclinación inicial calculada en setup
          forceX += thrust * sin(launchTiltRadians);
          forceY += thrust * cos(launchTiltRadians);
      }

      // Arrastre (opuesto al vector velocidad)
      let vMagnitude = sqrt(vx*vx + vy*vy);
      if (vMagnitude > 1e-6) { // Evitar división por cero o casi cero
           let dragMagnitude = 0.5 * airDensity * vMagnitude * vMagnitude * currentDragCoefficient * currentReferenceArea;
           // Vector de arrastre: -dragMagnitude * vectorUnitarioVelocidad
           forceX += -dragMagnitude * (vx / vMagnitude);
           forceY += -dragMagnitude * (vy / vMagnitude);
      }


      // Calcular aceleración neta (vectorial)
      // Asegurarse de que la masa no sea cero para evitar división por cero
      if (currentMass > 0) {
           ax = forceX / currentMass;
           ay = forceY / currentMass;
      } else {
           ax = 0;
           ay = 0;
      }


      // --- Integración (Método de Euler simple) ---
      vx += ax * dt_sim; // Use simulated time
      x += vx * dt_sim; // Use simulated time
      vy += ay * dt_sim; // Use simulated time
      y += vy * dt_sim; // Use simulated time


      // --- RESTRICCIÓN/TRANSICIÓN: Manejo del Apogeo y Línea de Kármán ---
      // El apogeo vertical se alcanza cuando la velocidad vertical (vy) se vuelve no positiva (<= 0).
      if (phase === "CapsuleFreeFlightAscent" && vy <= 0) {
          // Alcanzó apogeo vertical (vy <= 0) en fase de ascenso libre
          phase = "CapsuleFreeFlightDescent"; // Transición a descenso libre
          timeInPhase = 0; // Reiniciar tiempo en fase
          // Si el apogeo ocurrió por encima de 100km, forzar la altitud a 100km
          if (y > KARMAN_LINE_ALT_LIMIT) {
               y = KARMAN_LINE_ALT_LIMIT;
               vy = 0; // Velocidad vertical es 0 en el apogeo (o forzada a 0)
               // La velocidad horizontal (vx) se mantiene.
          }
           // En el apogeo vertical, la aceleración vertical inicial al empezar la caída es -g (después de recalcular fuerzas)
      }


      // Asegurar que la altitud no sea negativa (solo si no estamos ya aterrizados)
      if (y < 0 && phase !== "Landed") {
           y = 0;
           vy = 0;
           ay = 0; // Aceleración vertical cero en el suelo
           phase = "Landed";
           timeInPhase = 0;
           gForce = 1; // 1G en el suelo
           vx = 0; // Frenar velocidad horizontal al aterrizar
           ax = 0; // Aceleración horizontal cero en el suelo
      }


      // Actualizar altitud máxima y alcance máximo
      if (y > maxAltitudeReached) {
          maxAltitudeReached = y;
      }
      // El alcance horizontal máximo se rastrea como la distancia máxima desde el punto de lanzamiento (x=0)
       // Solo se actualiza si el apogeo ha pasado (vy <= 0) o si ya está descendiendo
       if (vy <= 0 || phase === "CapsuleFreeFlightDescent" || phase === "DrogueParachutes" || phase === "MainChutes" || phase === "Landed") {
           maxDownrangeReached = max(maxDownrangeReached, abs(x)); // Rastrea la distancia horizontal máxima
       }


      // --- Transiciones de Fase (Basadas en tiempo simulado y altitud) ---
      switch (phase) {
          case "Launch":
              // MECO y Separación ocurren después de un tiempo simulado
              if (timeInPhase >= LAUNCH_DURATION_SIMULATED_CONST) { // Use simulated time in comparison
                  phase = "CapsuleFreeFlightAscent";
                  timeInPhase = 0;
                  thrust = 0; // Apagar motor (MECO)
                  currentMass = capsuleMass; // Cambiar a masa de la cápsula (Separación)
                  // Max Q calculado ya está en Pascales, lo dividimos por 1000 para kPa para mostrar
                  maxQ = maxQ / 1000;
              }
               // Protección: Si empieza a caer en launch (error param), transiciona rápido
              if (vy < 0 && timeInPhase > 5) { // Use simulated time
                   phase = "CapsuleFreeFlightAscent"; // Pasar a ascenso libre si empieza a caer antes
                   timeInPhase = 0;
                   thrust = 0; // Apagar motor
                   currentMass = capsuleMass; // Separación
                   maxQ = maxQ / 1000; // Mostrar Max Q en kPa
              }
              break;
          case "CapsuleFreeFlightAscent":
              // La transición a Descenso Libre se maneja por la condición vy <= 0 (Apogeo Vertical).
              break;
          case "CapsuleFreeFlightDescent": // Reentrada conceptual ocurre en esta fase
              // Calcular variables conceptuales de reentrada
              // Temperatura superficial conceptual (basado en altitud y velocidad)
               let vMagnitudeReentry = sqrt(vx*vx + vy*vy); // Usar magnitud total
               let reentrySpeedThresholdLow = 150; // m/s
               let reentrySpeedThresholdHigh = 600; // m/s
               let reentryAltThresholdLow = 20000; // m
               let reentryAltThresholdHigh = 80000; // m // Empieza a sentir el aire denso

               if (y < reentryAltThresholdHigh && vMagnitudeReentry > reentrySpeedThresholdLow) {
                   if (y < reentryAltThresholdLow && vMagnitudeReentry > reentrySpeedThresholdHigh) {
                       surfaceTemperatureConcept = "Calentamiento Intenso";
                   } else if (y < reentryAltThresholdLow || vMagnitudeReentry > reentrySpeedThresholdLow * 1.5) { // Margen más amplio
                       surfaceTemperatureConcept = "Calentamiento Moderado";
                   } else {
                       surfaceTemperatureConcept = "Aumentando";
                   }
               } else {
                   surfaceTemperatureConcept = "Bajo/Normal";
               }

              // Despliegue de drogues
              if (y <= DROGUES_DEPLOY_ALTITUDE_CONST && vy < 0) { // Si está por debajo de altitud de drogues y descendiendo
                  phase = "DrogueParachutes";
                  timeInPhase = 0;
                  // Resetear conceptos de reentrada al salir de fase de alta velocidad
                  surfaceTemperatureConcept = "Enfriando Rápido"; // Empieza a enfriar
              }

               // Aterrizaje de emergencia si cae por debajo de 0 antes de paracaídas (debería ir a Drogues primero)
              if (y <= 0 && vy < 0) {
                   y = 0; vy = 0; ay = 0;
                   x = x; // Mantener posición horizontal
                   vx = 0; ax = 0; // Frenar horizontalmente
                   phase = "Landed"; timeInPhase = 0; gForce = 1;
                   surfaceTemperatureConcept = "Normal";
                   instantaneousTerminalVelocity = 0; // Resetear al aterrizar
              }
              break;

          case "DrogueParachutes":
               // Calcular velocidad terminal instantánea (simplificada verticalmente)
               let current_cd_drogues = dragCoefficientDrogues;
               let current_area_drogues = referenceAreaDrogues;
               let baseDensityForTerminalV = airDensity;
               let denominatorDrogues = (baseDensityForTerminalV * current_cd_drogues * current_area_drogues);
               instantaneousTerminalVelocity = (denominatorDrogues > 0.000001) ? sqrt(2 * currentMass * g / denominatorDrogues) : 0;


              // Despliegue de paracaídas principal
              if (y <= MAIN_CHUTES_DEPLOY_ALTITUDE_CONST && vy < 0) { // Si está por debajo de altitud de paracaídas principal y descendiendo
                  phase = "MainChutes";
                  timeInPhase = 0;
                  surfaceTemperatureConcept = "Normal"; // Ya ha enfriado
              }

               // Aterrizaje de emergencia si cae por debajo de 0 antes de paracaídas principal (debería ir a MainChutes primero)
              if (y <= 0 && vy < 0) {
                   y = 0; vy = 0; ay = 0;
                   x = x; // Mantener posición horizontal
                   vx = 0; ax = 0; // Frenar horizontalmente
                   phase = "Landed"; timeInPhase = 0; gForce = 1;
                   surfaceTemperatureConcept = "Normal";
                   instantaneousTerminalVelocity = 0; // Resetear al aterrizar
              }
              break;

          case "MainChutes":
              // Calcular velocidad terminal instantánea (simplificada verticalmente)
               let current_cd_main = dragCoefficientMainChutes;
               let current_area_main = referenceAreaMainChutes;
               let baseDensityForTerminalV_main = airDensity;
               let denominatorMain = (baseDensityForTerminalV_main * current_cd_main * current_area_main);
               instantaneousTerminalVelocity = (denominatorMain > 0.000001) ? sqrt(2 * currentMass * g / denominatorMain) : 0;


              if (y <= 0) { // Aterrizado
                  y = 0; vy = 0; ay = 0;
                   x = x; // Mantener posición horizontal
                  vx = 0; ax = 0; // Frenar horizontalmente
                  phase = "Landed"; timeInPhase = 0; gForce = 1;
                  surfaceTemperatureConcept = "Normal";
                  instantaneousTerminalVelocity = 0; // Resetear al aterrizar
              }
              break;
      }

      // Asegurar que la altitud nunca sea negativa durante el vuelo (para cálculos de densidad, etc.)
      if (y < 0 && phase !== "Landed") y = 0;

      // Asegurar que la velocidad terminal instantánea sea no negativa
      instantaneousTerminalVelocity = max(0, instantaneousTerminalVelocity);

      // --- Guardar punto de trayectoria ---
      // Guardar un punto si la nave se ha movido lo suficiente o en fases importantes
       let lastPoint = trajectoryPoints.length > 0 ? trajectoryPoints[trajectoryPoints.length - 1] : null;
       let addPoint = false;

       // Añadir un punto si es el primer punto, o si se ha movido una distancia mínima
       if (lastPoint === null) {
           addPoint = true;
       } else {
           let distSq = (x - lastPoint.x) * (x - lastPoint.x) + (y - lastPoint.y) * (y - lastPoint.y);
           let minDistSq = 5 * 5; // Añadir si se ha movido 5 metros (ajustar según velocidad de simulación y fluidez deseada)
           if (distSq > minDistSq) {
              addPoint = true;
           }
       }

       // Asegurarse de añadir points en momentos clave, como el apogeo vertical o el despliegue de paracaídas
       if (phase === "CapsuleFreeFlightDescent" && timeInPhase < dt_sim * 2 && timeInPhase > 0) { // Justo después del apogeo vertical
            addPoint = true;
       }
       if (phase === "DrogueParachutes" && timeInPhase < dt_sim * 2 && timeInPhase > 0) { // Justo al desplegar drogues
           addPoint = true;
       }
        if (phase === "MainChutes" && timeInPhase < dt_sim * 2 && timeInPhase > 0) { // Justo al desplegar principal
           addPoint = true;
       }
        if (phase === "Landed" && timeInPhase < dt_sim * 2 && timeInPhase > 0) { // Justo al aterrizar
           addPoint = true;
       }


       if (addPoint) {
            trajectoryPoints.push(createVector(x, y));
           // Limitar el número de puntos
           if (trajectoryPoints.length > MAX_TRAJECTORY_POINTS) {
                trajectoryPoints.shift(); // Eliminar el punto más antiguo
           }
       }


  } // Fin if phase != "Ready" && "Landed"

   // Asegurar altitud no negativa en Ready tampoco
  if (phase === "Ready" && y < 0) y = 0;


  // --- Calcular G-Force ---
  // Magnitud de Acel. no gravitacional = sqrt(ax*ax + (ay+g)*(ay+g))
  // gForce = Magnitud / g
  let nonGravAccelMag = sqrt(ax * ax + (ay + g) * (ay + g));
  gForce = nonGravAccelMag / g;

  // Asegurar que la G-force sea al menos 0 en la visualización de la barra
  // y un máximo para la escala de la barra
  let displayGForce = max(0, min(gForce, gForceScale));


  // --- Visualización ---

  // Establecer el fondo a un solo color
  background(backgroundColor); // Usar el color oscuro del espacio como fondo fijo

  // Calcular maxDisplayAltitudeCurrent para escalar la vista vertical dinámicamente
  // Mostrar un poco más que el máximo alcanzado, pero al menos un rango mínimo
  let targetMaxAlt = max(maxAltitudeReached * DISPLAY_ALTITUDE_BUFFER_RATIO, KARMAN_LINE_ALT_LIMIT * 1.2, 10000);
  maxDisplayAltitudeCurrent = targetMaxAlt;

  // --- Calcular el rango horizontal de metros a mostrar, centrado en la nave ---
  let xMinDisplay = x - HORIZONTAL_DISPLAY_RANGE / 2;
  let xMaxDisplay = x + HORIZONTAL_DISPLAY_RANGE / 2;

  // --- Mapear coordenadas físicas (x, y) a píxeles en el ÁREA DE SIMULACIÓN VISUAL ---
  // Mapea el rango horizontal [xMinDisplay, xMaxDisplay] al rango de píxeles horizontal [simAreaXMin, simAreaXMax]
  // Mapea la altitud [0, maxDisplayAltitudeCurrent] al rango de píxeles vertical [simAreaYMax, simAreaYMin] (invertido)
  let xMapped = map(x, xMinDisplay, xMaxDisplay, simAreaXMin, simAreaXMax);
  let yMapped = map(y, 0, maxDisplayAltitudeCurrent, simAreaYMax, simAreaYMin);

  // Clamp la posición visual de la nave al área de simulación visual (con margen del tamaño de la nave)
  // Estas son las coordenadas EN PIXELES donde se dibujará la nave.
  let clampedNavePixelX = constrain(xMapped, simAreaXMin + naveWidthPixels/2, simAreaXMax - naveWidthPixels/2);
  let clampedNavePixelY = constrain(yMapped, simAreaYMin + naveHeightPixels/2, simAreaYMax - naveHeightPixels/2);


  // Dibujar el suelo - En la base del área de simulación visual
  stroke(0);
  strokeWeight(2);
  line(simAreaXMin, simAreaYMax, simAreaXMax, simAreaYMax);
  fill(groundColor);
  noStroke();
  rectMode(CORNER); // Volver a modo CORNER para rectángulos de área
  rect(simAreaXMin, simAreaYMax, simAreaXMax - simAreaXMin, MIN_DISPLAY_ALTITUDE_BUFFER_Y);


  // --- Dibujar la trayectoria ---
  noFill();
  stroke(trajectoryColor);
  strokeWeight(1.5);
  beginShape();
  for (let i = 0; i < trajectoryPoints.length; i++) {
      let pX = trajectoryPoints[i].x;
      let pY = trajectoryPoints[i].y;
      // Mapear cada punto de la trayectoria a píxeles usando el *rango de visualización actual*
      let mappedPX = map(pX, xMinDisplay, xMaxDisplay, simAreaXMin, simAreaXMax);
      let mappedPY = map(pY, 0, maxDisplayAltitudeCurrent, simAreaYMax, simAreaYMin);
      // Solo dibujar puntos que estén dentro del área de simulación visual (con un pequeño buffer)
       if (mappedPX >= simAreaXMin - 10 && mappedPX <= simAreaXMax + 10 && mappedPY >= simAreaYMin - 10 && mappedPY <= simAreaYMax + 10) {
          vertex(mappedPX, mappedPY);
      }
  }
  endShape();


  // --- Dibujar la nave (o partes de ella) y paracaídas ---
  rectMode(CENTER);
  stroke(0); // Borde negro

  // USAR clampedNavePixelX y clampedNavePixelY PARA DIBUJAR LA NAVE Y ELEMENTOS RELATIVOS

  if (phase === "Launch" || phase === "Ready") {
      // Dibujar el cohete completo
      fill(200); // Gris claro
      rect(clampedNavePixelX, clampedNavePixelY, naveWidthPixels, naveHeightPixels * 2); // Cohete es más alto
      // Dibujar la cápsula encima (parte superior)
      fill(255); // Blanco
      let capsuleTopPixelY = clampedNavePixelY - naveHeightPixels/2; // Posición relativa visual Y
      rect(clampedNavePixelX, capsuleTopPixelY, naveWidthPixels, naveHeightPixels);

  } else if (phase !== "Landed") {
      // Dibujar solo la cápsula
      fill(255); // Blanco
      rect(clampedNavePixelX, clampedNavePixelY, naveWidthPixels, naveHeightPixels);

      // Dibujar paracaídas si están desplegados
      if (phase === "DrogueParachutes" || phase === "MainChutes") {
          // Calcular posición visual de los paracaídas encima de la cápsula
          let parachutePixelY = clampedNavePixelY - naveHeightPixels/2 - (phase === "MainChutes" ? 40 : 20); // Posición visual encima

          fill(parachuteColor);
          noStroke(); // Sin borde para los paracaídas

          if (phase === "DrogueParachutes") {
              // Dibujar pequeños drogues (conceptualmente)
              ellipse(clampedNavePixelX - naveWidthPixels/2, parachutePixelY, 10, 10);
              ellipse(clampedNavePixelX + naveWidthPixels/2, parachutePixelY, 10, 10);
          } else if (phase === "MainChutes") {
              // Dibujar paracaídas principal más grande (concepto simple)
              ellipse(clampedNavePixelX, parachutePixelY, 40, 25); // Forma ovalada simple para campana
              // Dibujar líneas conectando si queremos más detalle
              stroke(parachuteColor);
              strokeWeight(1);
              line(clampedNavePixelX, parachutePixelY + 12, clampedNavePixelX - naveWidthPixels/4, clampedNavePixelY - naveHeightPixels/2);
              line(clampedNavePixelX, parachutePixelY + 12, clampedNavePixelX + naveWidthPixels/4, clampedNavePixelY - naveHeightPixels/2);
          }
          stroke(0); // Restaurar borde para la nave si es necesario después
      }

      // Dibujar booster si acaba de separarse (visual conceptual, no física)
       if (phase === "CapsuleFreeFlightAscent" && timeInPhase < 15) { // Mostrar brevemente después de separación
           fill(200); // Gris claro
           // Calcular la posición visual del booster en relación con la cápsula y el desplazamiento horizontal conceptual
           let boosterOffsetY = naveHeightPixels * 1.5; // Posición visual por debajo
           let boosterOffsetX = map(timeInPhase, 0, 15, 0, -(simAreaXMax - simAreaXMin) * 0.1); // Ajustar el desplazamiento horizontal visual

           let boosterPixelX = clampedNavePixelX + boosterOffsetX;
           let boosterPixelY = clampedNavePixelY + boosterOffsetY;

           rect(boosterPixelX, boosterPixelY, naveWidthPixels, naveHeightPixels * 1.5); // Dibujar booster
           fill(textColorSecondary);
           textAlign(RIGHT, BOTTOM);
           textSize(10);
           text("Booster", boosterPixelX - naveWidthPixels/2, boosterPixelY + naveHeightPixels*1.5/2);

           // Agregar un texto conceptual de "Separación" cerca de la nave
           if (timeInPhase < 5) {
               fill(textColorPrimary);
               textAlign(CENTER, CENTER);
               textSize(14);
               text("Separación", clampedNavePixelX, clampedNavePixelY + naveHeightPixels);
           }
       }
  }


  // Dibujar línea de la Línea de Kármán (100 km) - En el área de simulación visual
  const KARMAN_LINE_ALT = 100000; // metros
  if (KARMAN_LINE_ALT > 0 && KARMAN_LINE_ALT < maxDisplayAltitudeCurrent) {
      // Mapear la altitud de Kármán al área de simulación visual
      let karmanYPixel = map(KARMAN_LINE_ALT, 0, maxDisplayAltitudeCurrent, simAreaYMax, simAreaYMin);
       // Solo dibujar la línea si está dentro de los límites verticales del área de simulación visual
       if (karmanYPixel >= simAreaYMin && karmanYPixel <= simAreaYMax) {
            stroke(karmanLineColor); // Naranja
            strokeWeight(1);
            line(simAreaXMin, karmanYPixel, simAreaXMax, karmanYPixel); // Dibuja a través de todo el ancho del área de simulación
            fill(karmanLineColor); // Naranja
            noStroke();
            textAlign(LEFT, BOTTOM); // Alinear texto a la izquierda y debajo
            textSize(12);

            // Ajuste de posición vertical del texto de Kármán
            let karmanTextY = karmanYPixel - 2;
            if (karmanYPixel < simAreaYMin + 20) { karmanTextY = karmanYPixel + 12; }
            karmanTextY = constrain(karmanTextY, simAreaYMin + 10, simAreaYMax - 10);

            fill(textColorPrimary); // Usar color fijo y claro para las etiquetas en área de simulación
            text(`Línea de Kármán: ${nf(KARMAN_LINE_ALT, 0, 0)}m (100 km)`, simAreaXMin + 5, karmanTextY);
       }
  }


  // Dibujar línea de altitud de paracaídas (Drogues) si es visible - En el área de simulación
  if (DROGUES_DEPLOY_ALTITUDE_CONST > 0 && DROGUES_DEPLOY_ALTITUDE_CONST < maxDisplayAltitudeCurrent) {
      let droguesYPixel = map(DROGUES_DEPLOY_ALTITUDE_CONST, 0, maxDisplayAltitudeCurrent, simAreaYMax, simAreaYMin);
      if (droguesYPixel >= simAreaYMin && droguesYPixel <= simAreaYMax) {
           stroke(parachuteColor); // Blanco translúcido
           strokeWeight(1);
           line(simAreaXMin, droguesYPixel, simAreaXMax, droguesYPixel); // Dibuja a través de todo el ancho
           fill(textColorPrimary); // Usar color fijo y claro
           noStroke();
           textAlign(LEFT, BOTTOM);
           textSize(12);
           text(`Despliegue Drogues: ${nf(DROGUES_DEPLOY_ALTITUDE_CONST, 0, 0)}m`, simAreaXMin + 5, droguesYPixel - 2);
      }
  }

  // Dibujar línea de altitud de paracaídas (Principal) si es visible - En el área de simulación
   if (MAIN_CHUTES_DEPLOY_ALTITUDE_CONST > 0 && MAIN_CHUTES_DEPLOY_ALTITUDE_CONST < maxDisplayAltitudeCurrent) {
      let mainChutesYPixel = map(MAIN_CHUTES_DEPLOY_ALTITUDE_CONST, 0, maxDisplayAltitudeCurrent, simAreaYMax, simAreaYMin);
      if (mainChutesYPixel >= simAreaYMin && mainChutesYPixel <= simAreaYMax) {
           stroke(parachuteColor); // Blanco translúcido
           strokeWeight(1);
           line(simAreaXMin, mainChutesYPixel, simAreaXMax, mainChutesYPixel); // Dibuja a través de todo el ancho
           fill(textColorPrimary); // Usar color fijo y claro
           noStroke();
           textAlign(LEFT, BOTTOM);
           textSize(12);
           text(`Despliegue Paracaídas Principal: ${nf(MAIN_CHUTES_DEPLOY_ALTITUDE_CONST, 0, 0)}m`, simAreaXMin + 5, mainChutesYPixel - 2);
      }
   }


   // Dibujar el origen X (Launch Site) en el área de simulación
    let launchSiteXPixel = map(0, xMinDisplay, xMaxDisplay, simAreaXMin, simAreaXMax);
     // Dibujar una línea vertical sutil en el origen X
     // Solo dibujarla si está dentro de la vista horizontal actual
    if (launchSiteXPixel >= simAreaXMin && launchSiteXPixel <= simAreaXMax) {
        stroke(textColorSecondary);
        strokeWeight(0.5);
        line(launchSiteXPixel, simAreaYMin, launchSiteXPixel, simAreaYMax);
        fill(textColorPrimary);
        noStroke();
        textAlign(CENTER, BOTTOM);
        textSize(10);
        text("Launch Site (x=0)", launchSiteXPixel, simAreaYMax - 5);
    }


  // --- Mostrar información de vuelo detallada por fase (EN PANEL DE DATOS) ---

  fill(textColorPrimary); // Usar color fijo para el panel de datos (White)
  noStroke();
  textSize(14);
  textAlign(LEFT, TOP);

  let textX = 10;
  let textY = 10; // Posición Y relativa al inicio del canvas
  let textSpacing = 18; // Espacio vertical entre líneas de texto

  // Información General
  text(`Fase: ${phase}`, textX, textY); textY += textSpacing;
  text(`Altitud (Y): ${nf(y, 0, y > 1000 ? 0 : 1)} m`, textX, textY); textY += textSpacing; // Mostrar decimales solo a baja altitud
  text(`Posición Horizontal (X): ${nf(x, 0, 1)} m`, textX, textY); textY += textSpacing;
  text(`Velocidad Vertical (Vy): ${nf(vy, 0, 1)} m/s`, textX, textY); textY += textSpacing;
  text(`Velocidad Horizontal (Vx): ${nf(vx, 0, 1)} m/s`, textX, textY); textY += textSpacing;
  text(`Acel. Vertical (Ay): ${nf(ay, 0, 2)} m/s²`, textX, textY); textY += textSpacing;
   text(`Acel. Horizontal (Ax): ${nf(ax, 0, 2)} m/s²`, textX, textY); textY += textSpacing;
  text(`Tiempo Total (Simulado): ${nf(totalTime, 0, 1)} s`, textX, textY); textY += textSpacing; // Aclarar que es tiempo simulado

  if (phase !== "Ready") {
      text(`Alt. Máx Alcanzada: ${nf(maxAltitudeReached, 0, 1)} m`, textX, textY); textY += textSpacing;
       text(`Alcance Máx Alcanzado: ${nf(maxDownrangeReached, 0, 1)} m`, textX, textY); textY += textSpacing;
  }
   if (maxQ > 0.1) { // Mostrar MaxQ solo si es significativo
       text(`Max Q: ${nf(maxQ, 0, 2)} kPa`, textX, textY); textY += textSpacing; // Mostrar en kPa
   }


  textY += textSpacing; // Espacio extra antes de variables por fase

  // Recalcular magnitudes de arrastre y sustentación para mostrar (ya se usaron en la física)
  // Usamos las variables actuales calculadas en el switch de fases
   let vMagnitudeDisplay = sqrt(vx*vx + vy*vy); // Usar magnitud total
   let currentDragMagnitudeDisplay = 0.5 * airDensity * vMagnitudeDisplay * vMagnitudeDisplay * currentDragCoefficient * currentReferenceArea;
   // No hay sustentación física simulada en 2D, no mostrar magnitud.

  // Variables específicas por fase
  fill(textColorSecondary); // Usar color secundario fijo para detalles de fase (Gris claro)

  switch (phase) {
      case "Ready":
          fill(textColorPrimary); // Mensaje principal en color primario fijo (White)
          textAlign(CENTER, CENTER);
          textSize(20);
          // Posicionar el mensaje "Presiona ESPACIO" en el área de simulación visual
          text("Presiona ESPACIO para Iniciar Simulación", width/2, simAreaYMin + (simAreaYMax - simAreaYMin)/2); // Centro del área de simulación
          break;

      case "Launch":
          text(`-- Fase Lanzamiento --`, textX, textY); textY += textSpacing;
          text(`Empuje: ${nf(thrust, 0, 0)} N`, textX, textY); textY += textSpacing;
          text(`Masa Actual: ${nf(currentMass, 0, 0)} kg`, textX, textY); textY += textSpacing;
          text(`Cd (Cohete): ${nf(currentDragCoefficient, 0, 2)}`, textX, textY); textY += textSpacing;
          text(`Area Ref. (Cohete): ${nf(currentReferenceArea, 0, 1)} m²`, textX, textY); textY += textSpacing;
          text(`Densidad Aire: ${nf(airDensity, 0, 4)} kg/m³`, textX, textY); textY += textSpacing;
          text(`Fuerza Arrastre (Magnitud): ${nf(currentDragMagnitudeDisplay, 0, 0)} N`, textX, textY); textY += textSpacing;
          text(`Velocidad Magnitud: ${nf(vMagnitudeDisplay, 0, 1)} m/s`, textX, textY); textY += textSpacing;
          text(`Tiempo Motor Restante (Simulado): ${nf(max(0, LAUNCH_DURATION_SIMULATED_CONST - timeInPhase), 0, 1)} s (MECO)`, textX, textY); textY += textSpacing; // Usar constante y aclarar
          break;

      case "CapsuleFreeFlightAscent":
          text(`-- Fase Ascenso Libre (Cápsula) --`, textX, textY); textY += textSpacing;
          text(`Masa Actual: ${nf(currentMass, 0, 0)} kg`, textX, textY); textY += textSpacing;
          text(`Cd (Cápsula): ${nf(currentDragCoefficient, 0, 2)}`, textX, textY); textY += textSpacing;
          text(`Area Ref. (Cápsula): ${nf(currentReferenceArea, 0, 1)} m²`, textX, textY); textY += textSpacing;
          text(`Densidad Aire: ${nf(airDensity, 0, 5)} kg/m³`, textX, textY); textY += textSpacing; // Muy baja a alta altitud
          text(`Fuerza Arrastre (Magnitud): ${nf(currentDragMagnitudeDisplay, 0, 2)} N`, textX, textY); textY += textSpacing; // Muy baja
          text(`Velocidad Magnitud: ${nf(vMagnitudeDisplay, 0, 1)} m/s`, textX, textY); textY += textSpacing;
           if (vy <= 0 && y > 100) { // Indica que ha alcanzado el apogeo (vertical), evitar mostrarlo en el suelo
               fill(textColorPrimary);
               text(`APOGEO VERTICAL ALCANZADO`, textX, textY); textY += textSpacing;
               fill(textColorSecondary);
           }
          break;

      case "CapsuleFreeFlightDescent": // Reentrada conceptual ocurre en esta fase
          text(`-- Fase Descenso Libre (Reentrada) --`, textX, textY); textY += textSpacing;
          text(`Masa Actual: ${nf(currentMass, 0, 0)} kg`, textX, textY); textY += textSpacing;
          text(`Cd (Cápsula): ${nf(currentDragCoefficient, 0, 2)}`, textX, textY); textY += textSpacing;
          text(`Area Ref. (Cápsula): ${nf(currentReferenceArea, 0, 1)} m²`, textX, textY); textY += textSpacing;
          text(`Densidad Aire: ${nf(airDensity, 0, 4)} kg/m³`, textX, textY); textY += textSpacing; // Aumentando
          text(`Fuerza Arrastre (Magnitud): ${nf(currentDragMagnitudeDisplay, 0, 0)} N (Aumentando)`, textX, textY); textY += textSpacing; // Aumentando
          text(`Velocidad Magnitud: ${nf(vMagnitudeDisplay, 0, 1)} m/s`, textX, textY); textY += textSpacing;
          text(`Alt. despliegue Drogues: ${nf(DROGUES_DEPLOY_ALTITUDE_CONST, 0, 0)} m`, textX, textY); textY += textSpacing;
          text(`Temp. Superficie (concept.): ${surfaceTemperatureConcept}`, textX, textY); textY += textSpacing;
          break;

      case "DrogueParachutes":
          text(`-- Fase Descenso con Drogues --`, textX, textY); textY += textSpacing;
          text(`Masa Actual: ${nf(currentMass, 0, 0)} kg`, textX, textY); textY += textSpacing;
          text(`Cd (Drogues): ${nf(currentDragCoefficient, 0, 2)}`, textX, textY); textY += textSpacing;
          text(`Area Efectiva (Drogues): ${nf(currentReferenceArea, 0, 1)} m²`, textX, textY); textY += textSpacing;
          text(`Densidad Aire: ${nf(airDensity, 0, 4)} kg/m³`, textX, textY); textY += textSpacing;
          text(`Fuerza Arrastre (Magnitud): ${nf(currentDragMagnitudeDisplay, 0, 0)} N (Alto)`, textX, textY); textY += textSpacing; // Alto
          text(`Velocidad Magnitud: ${nf(vMagnitudeDisplay, 0, 1)} m/s`, textX, textY); textY += textSpacing;
          text(`Velocidad Terminal (vertical simple): ${nf(instantaneousTerminalVelocity, 0, 1)} m/s`, textX, textY); textY += textSpacing;
           text(`Alt. despliegue Paracaídas Principal: ${nf(MAIN_CHUTES_DEPLOY_ALTITUDE_CONST, 0, 0)} m`, textX, textY); textY += textSpacing;
           text(`Temp. Superficie (concept.): ${surfaceTemperatureConcept}`, textX, textY); textY += textSpacing;
          break;

      case "MainChutes":
          text(`-- Fase Descenso con Paracaídas Principal --`, textX, textY); textY += textSpacing;
          text(`Masa Actual: ${nf(currentMass, 0, 0)} kg`, textX, textY); textY += textSpacing;
          text(`Cd (Principal): ${nf(currentDragCoefficient, 0, 2)}`, textX, textY); textY += textSpacing;
          text(`Area Efectiva (Principal): ${nf(currentReferenceArea, 0, 1)} m²`, textX, textY); textY += textSpacing;
          text(`Densidad Aire: ${nf(airDensity, 0, 4)} kg/m³`, textX, textY); textY += textSpacing;
          text(`Fuerza Arrastre (Magnitud): ${nf(currentDragMagnitudeDisplay, 0, 0)} N (MUY ALTO)`, textX, textY); textY += textSpacing; // Muy grande
           text(`Velocidad Magnitud: ${nf(vMagnitudeDisplay, 0, 1)} m/s`, textX, textY); textY += textSpacing;
          text(`Velocidad Terminal (vertical simple): ${nf(instantaneousTerminalVelocity, 0, 1)} m/s`, textX, textY); textY += textSpacing;
           text(`Temp. Superficie (concept.): ${surfaceTemperatureConcept}`, textX, textY); textY += textSpacing;
          break;

      case "Landed":
          fill(0, 150, 0); // Verde para el mensaje principal
          textAlign(CENTER, CENTER);
          textSize(24);
          // Posicionar el mensaje "Aterrizaje Exitoso" en el área de simulación visual
          text("¡Aterrizaje Exitoso!", width/2, simAreaYMin + (simAreaYMax - simAreaYMin)/2 - 30); // Centro del área de simulación - ajustar Y
          fill(textColorPrimary); // Texto informativo en blanco
          textSize(16);
          // Posicionar el texto informativo en el área de simulación
          text(`Altitud Máxima Alcanzada: ${nf(maxAltitudeReached, 0, 1)} m`, width/2, simAreaYMin + (simAreaYMax - simAreaYMin)/2 + 0);
          text(`Alcance Máx Alcanzado: ${nf(maxDownrangeReached, 0, 1)} m`, width/2, simAreaYMin + (simAreaYMax - simAreaYMin)/2 + 20);
          text(`Tiempo Total de Vuelo (Simulado): ${nf(totalTime, 0, 1)} s`, width/2, simAreaYMin + (simAreaYMax - simAreaYMin)/2 + 40);
          text(`Max Q (Ascenso): ${nf(maxQ, 0, 2)} kPa`, width/2, simAreaYMin + (simAreaYMax - simAreaYMin)/2 + 60); // Mostrar Max Q al final
          break;
  }


  // --- Indicador de G-force (EN PANEL DE DATOS) ---
  let gForceDisplay = nf(gForce, 0, 2);
  fill(textColorPrimary); // Usar color principal fijo para la etiqueta "G-Force" en el panel de datos (White)
  noStroke();
  textSize(14);
  textAlign(RIGHT, TOP);
  // Posicionar la etiqueta "G-Force" en el panel de datos
  text(`G-Force: ${gForceDisplay} G`, width - 10, 10);

  // Dibujar barra de G-force - En el panel de datos
  let gBarWidth = 30;
  let gBarX = width - gBarWidth - 10;
  let gBarY = 30; // Empezar un poco más abajo que la etiqueta "G-Force"
  let gBarBaseY = gBarY + gForceBarHeight; // Base de la barra (corresponde a 0G)

  // Barra de 0 a GForceScale Gs
  // Mapear la G-force (clampada) a la altura de la barra.
   let mappedG = map(displayGForce, 0, gForceScale, 0, gForceBarHeight);
   mappedG = constrain(mappedG, 0, gForceBarHeight); // Asegurar que está dentro del rango de la barra


  // Color de la barra (verde a amarillo a rojo)
  let gColor;
  if (gForce < 1.5) {
      gColor = color(0, 200, 0); // Verde (normal/bajo)
  } else if (gForce < 3) {
      gColor = lerpColor(color(0, 200, 0), color(255, 200, 0), map(gForce, 1.5, 3, 0, 1)); // Verde a amarillo
  } else {
      gColor = lerpColor(color(255, 200, 0), color(255, 0, 0), map(gForce, 3, gForceScale, 0, 1)); // Amarillo a rojo
  }

  // Dibujar solo la barra de color - En el panel de datos
  rectMode(CORNER); // Volver a modo CORNER para la barra
  fill(gColor); // Barra de G-force actual
  // La barra crece hacia arriba desde la base (0G)
  let barHeight = mappedG;
  rect(gBarX, gBarBaseY - barHeight, gBarWidth, barHeight);


  // Marcas en la barra (0G, 1G, 2G, etc.) - En el panel de datos
  stroke(150); // Marcas gris medio
  strokeWeight(1);
  fill(textColorPrimary); // Usar color principal fijo para las etiquetas de G en el panel de datos (White)
  textAlign(RIGHT, CENTER);
  textSize(10);

  for (let i = 0; i <= gForceScale; i++) {
      let markerY = gBarBaseY - map(i, 0, gForceScale, 0, gForceBarHeight);
      // Dibujar línea de marca solo si está dentro de la altura de la barra
      if (markerY >= gBarY && markerY <= gBarBaseY) {
          line(gBarX, markerY, gBarX + gBarWidth, markerY);
          text(`${i}G`, gBarX - 5, markerY);
      }
  }
  // Asegurarse de que la marca 1G esté siempre visible si la escala lo permite
  if (gForceScale >= 1) {
      let markerY1G = gBarBaseY - map(1, 0, gForceScale, 0, gForceBarHeight);
       if (markerY1G >= gBarY && markerY1G <= gBarBaseY) {
           stroke(150);
           line(gBarX, markerY1G, gBarX + gBarWidth, markerY1G);
            fill(textColorPrimary);
            textAlign(RIGHT, CENTER);
            textSize(10);
            text(`1G`, gBarX - 5, markerY1G);
       }
  }


   // Dibujar una línea separadora entre el panel de datos y el área de simulación visual
   stroke(0);
   strokeWeight(1);
   line(0, DATA_PANEL_HEIGHT, width, DATA_PANEL_HEIGHT);


}

// --- Función para Reiniciar la Simulación ---
function resetSimulation() {
  x = 0; // Posición horizontal inicial (en el launch site)
  y = 0; // Altitud inicial (en el suelo)
  vx = 0; // Velocidad horizontal inicial
  vy = 0; // Velocidad vertical inicial
  ax = 0; // Aceleración horizontal inicial
  ay = 0; // Aceleración vertical inicial

  phase = "Ready";
  timeInPhase = 0;
  totalTime = 0;
  maxAltitudeReached = 0; // Reiniciar al inicio
  maxDownrangeReached = 0; // Reiniciar alcance
  gForce = 1; // 1G en el suelo

  thrust = LAUNCH_THRUST_CONST; // Restablecer el empuje
  currentMass = rocketMass; // Masa inicial es la del cohete completo
  currentDragCoefficient = dragCoefficientRocket;
  currentReferenceArea = referenceAreaRocket;
  maxQ = 0; // Resetear Max Q
  instantaneousTerminalVelocity = 0; // Resetear

  // Restablecer variables conceptuales de descenso
  surfaceTemperatureConcept = "N/A";

  // Limpiar la estela de trayectoria
  trajectoryPoints = [];
   // Asegurarse de que el primer punto de la trayectoria sea el origen
   trajectoryPoints.push(createVector(x, y));

}

// --- Manejar Evento de Teclado (Espacio para Iniciar/Reiniciar) ---
function keyPressed() {
  if (key === ' ') {
    if (phase === "Ready" || phase === "Landed") {
        // Iniciar simulación o Reiniciar después de aterrizar
        resetSimulation(); // Asegurarse de resetear antes de iniciar
        phase = "Launch";
        timeInPhase = 0; // Reiniciar tiempo en fase
        totalTime = 0; // Reiniciar tiempo total
        maxAltitudeReached = y; // Registrar la altitud inicial (0)
         maxDownrangeReached = abs(x); // Registrar el alcance inicial (0)
        maxQ = 0; // Resetear Max Q al inicio del vuelo
    }
  }
}

// Función auxiliar para obtener el signo de un número
function sign(x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0; // Return 0 for 0
}