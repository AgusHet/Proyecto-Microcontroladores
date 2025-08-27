/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include "ff.h"
#include "user_diskio_spi.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	M_ERR,
	M_STDBY,
	M_SIM,
	M_TEST,
	M_MEM
} mode_t;	// Modos de operacion

typedef enum {
	INPUT_NORMAL,			// Interpretar comandos
	INPUT_VERIF,			// Modo de espera inicial para comenzar rutina de verificación
	INPUT_CONFIRM_SD,		// Espera 'y'/'n' para registrar sesión en SD
	INPUT_ID,				// Espera a que se ingrese ID de sesión
	INPUT_VER_REPRODUCIR,	// Espera 'v'/'r'
	INPUT_ELEGIR_REG,		// Num. registro
	INPUT_ERR_SD,			// Reintentar luego de err SD Modo MEM
	INPUT_ERR_MPU,			// Reintentar luego de err MPU Modo SIM
	INPUT_SERVO_WAIT_B1,   	// Botón para mover servo
	INPUT_SERVO_CONFIRM,    // Confirmación UART para verif. servos
	INPUT_CAL
} input_state_t;	// Tipos de respuesta que espera y maneja UART

typedef struct {
    float kp;           // Ganancia proporcional
    float ki;           // Ganancia integral
    float kd;           // Ganancia derivativa
    float prev_error;   // Error anterior
    float integral;     // Suma de errores (integral)
    float max_output;   // Límite máximo de salida
    float min_output;   // Límite mínimo de salida
} PID_Controller;	// Estructura para el controlador PID
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// YAW
#define REF_YAW 95.0f
#define MIN_YAW 70.0f
#define MAX_YAW 120.0f
// PITCH
#define REF_PITCH 95.0f
#define MIN_PITCH 70.0f
#define MAX_PITCH 120.0f
// MPU
#define MAX_XY 10.0f
#define MIN_XY -10.0f
// FILTRO
#define POND_GYRO 0.65f	// 0.6
#define POND_ACC 0.35f	// 0.4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// enums
mode_t modo;
input_state_t input_state;

// UART
char rx_buffer[512];
char tx_buffer[1024];
uint8_t rx_index = 0;
uint8_t rx_char;
const char clear[] = "\033[2J\033[H";

// FLAGS
uint8_t flagVerif = 0;  	// rutina de verificación aprobada (1)
uint8_t flagCal = 0;		// sensor calibrado (1) / sensor sin calibrar (0)
uint8_t flagCalCase = 0;
uint8_t flagRef = 0;		// servos en pos. de referencia (1) / sin información (0)
uint8_t flagMem = 0;		// hay registros en la sd (1) / no hay registros en la sd (0)
uint8_t flagReg = 0;		// registrar sesión (1) / no registrar sesión (0)
uint8_t flagRepro = 0;		// reproducción en curso (1) / (0)
uint8_t flagPID = 0;		// PID activado (1) / PID desactivado (0) - por default funciona a lazo abierto

// Relacionado a servomotores:
float pitch_actual;
float yaw_actual;
uint8_t servo_actual;	// (1) yaw, (2) pitch
uint32_t servo_timer = 0;
uint8_t servo_step = 0;
uint8_t servo_sequence_active = 0;

// Para manejo de archivos SD, etc.:
extern FATFS SDFatFS;			// Instancia de la estructura FATFS
char nombre_archivo[32];		// Para almacenar el nombre completo del registro creado
char registro_elegido[32];
char linea_repro[128];      	// Buffer para línea actual durante reproduccion
FIL archivo_log;				// Manejador de archivos
FIL archivo_repro;         		// Manejador global para reproducción
uint32_t time_stamp;
uint16_t num_reg_elegido;
uint32_t contador_repro = 0;  	// Contador de líneas reproducidas

// MPU
int16_t ax, ay, az, gx, gy, gz;
int16_t gx_corregido, gy_corregido;
int32_t sum_x_gyro = 0;
int32_t sum_y_gyro = 0;
float gx_dps, gy_dps;
float offset_x_gyro, offset_y_gyro;
float ang_filtrado_x, ang_filtrado_y;
float ref_x, ref_y;
float ang_filtrado_cal_x, ang_filtrado_cal_y;
float ang_pitch_sat, ang_yaw_sat;
float ang_accx = 0.0f;
float ang_accy = 0.0f;
float ang_gyrox = 0.0f;
float ang_gyroy = 0.0f;

// PID
PID_Controller pid_yaw;		// Struct PID eje yaw
PID_Controller pid_pitch;	// Stuct PID eje pitch

// Se plantean mismas ganancias para ambos ejes, pero se arma con dif. variables para aplicaciones futuras
float Kp_yaw = 1.0f;
float Ki_yaw = 10.0f;
float Kd_yaw = 0.0f;

float Kp_pitch = 1.0f;
float Ki_pitch = 10.0f;
float Kd_pitch = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* ============================================================================
 * PROTOTIPOS DE FUNCIONES
 * ============================================================================ */
// UART
void clear_buffer(void);				// Limpiar buffer RX
void myprintf(const char* fmt, ...);	// Función propia para imprimir x UART
void show_prompt(void);					// >>
void uart_error(const char* str);		// Imprimir error x UART

// INTÉRPRETE DE COMANDOS
void interprete(void);

// MÁQUINA DE ESTADOS & relacionados
void maq_estados(void);
void cambiar_modo(mode_t est);
void reset_leds(void);				// Apagar todos los indicadores luminosos

// VERIFICACIÓN DE COMPONENTES & relacionados
void rutina_verificacion(void);
uint8_t mount_sd(void);
uint8_t verif_sd(void);
uint8_t verif_mpu(void);
uint8_t i2c_bus_is_free(void);
void i2c_recover_bus(void);
uint8_t verif_servos(void);
void servo_sequence_task(void);

// MPU
uint8_t calc_gyro_offsets(void);
uint8_t calibrar_mpu(void);

// SERVOMOTORES
uint16_t ang_to_pwm(float angle);
void modificar_pwm(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pwm_pulse);	// Modificar ancho de pulso
void servos_ref(void);				// Enviar ambos servos a su posición de referencia
void yaw_pitch(float p, float y);	// Enviar consignas pitch yaw

// MODO MEM - SD & relacionados
uint8_t hay_registros(void);					// Verificar si hay registros en la memoria SD para ver/reproducir
void listar_registros(void);					// Mostrar registros disponibles
void ver_registro(const char* nombre);			// Mostrar contenidos de un archivo
void reproducir_registro(const char* nombre);	// Reproducir un registro con los servomotores
void detener_registro(void);					// Detener registro de manera segura (flagReg = 0, cerrar archivo)

// MODO SIM
float sat_ang_mpu(float ang);					// Saturación del ángulo medido por el MPU
float mpu_to_servo(float ang_mpu, uint8_t ind);	// Mapear ángulo MPU a ángulo servo
uint16_t filt_to_pwm(float raw, uint8_t ind);	// Medición filtrada -> Ancho de pulso PWM
void set_yp_reg(float ang, uint8_t ind);		// Establece valores para poder acceder a ellos globalmente en la creación de los registros
void cal_angulos_acelerometro(int16_t ax, int16_t ay, int16_t az, float *ang_accx, float *ang_accy);
void lectura_filtrado_mpu(float dt);				// Raw -> Filt
void lectura_filtrado_mpu_calibrada(float dt);		// Filt -> Cal
void mpu6050_print_error(uint8_t err_code);		// Procesar códigos de error

// PID
void init_pid_controllers(void);
void init_pid(PID_Controller* pid, float kp, float ki, float kd, float max_output, float min_output);
float calculate_pid(PID_Controller* pid, float setpoint, float measured_value, float dt);
void reset_all_pids(void);
void reset_pid(PID_Controller* pid);
uint16_t filt_to_pwm_pid(float measured_angle, uint8_t ind, float dt); // Medición filtrada -> Ancho de pulso PWM - cuando PID está activado
void set_kp(uint8_t axis, float kp);
void set_ki(uint8_t axis, float ki);
void set_kd(uint8_t axis, float kd);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
 * DEFINICIÓN DE FUNCIONES
 * ============================================================================ */

/* ----------------------------------------------------------------------------
 * UART
 * ---------------------------------------------------------------------------- */

// Limpiar buffer de recepción
void clear_buffer(void) {
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
}

// Función propia para transmitir datos via UART
void myprintf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(tx_buffer, sizeof(tx_buffer), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
}

// Notificación de errores via UART
void uart_error(const char* str) {
	sprintf(tx_buffer, "*ERR: %s\r\n", str);
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
}

// Mostrar prompt via UART
void show_prompt(void) {
	const char prompt[] = ">> ";
	HAL_UART_Transmit(&huart2, (uint8_t*)prompt, strlen(prompt), 10);
}

/* ----------------------------------------------------------------------------
 * INTÉRPRETE DE COMANDOS
 * ---------------------------------------------------------------------------- */

void interprete(void) {
	// Verificar que el comando tenga al menos 2 caracteres (':' + comando)
	if (strlen(rx_buffer) < 2) {
		uart_error("COMANDO INVÁLIDO");
		return;
	}

	switch (rx_buffer[1]) {

	case 'C':
		if (modo == M_STDBY) {
			if (verif_mpu()) {
				myprintf("Deje el sensor sobre una superficie plana. Este proceso tarda unos segundos\r\n");
				myprintf("Escriba 'y' para comenzar o 'n' para salir\r\n");
				input_state = INPUT_CAL;
			} else {
				flagCalCase = 1;
				cambiar_modo(M_ERR);
				uart_error("NO ES POSIBLE LEER EL SENSOR PARA CALIBRAR");
				myprintf("Escriba 'y' para reintentar conexión o 'n' para salir\r\n");
				input_state = INPUT_ERR_MPU;
			}
		} else {
			uart_error("COMANDO DISPONIBLE EN MODO STDBY");
		}
		break;

	case 'R':
		if (modo == M_STDBY) {
			if (!flagRef) {
				servos_ref(); 		// lleva a los dos servomotores a su posicion de referencia
				flagRef = 1;		// indicador de referencia
				myprintf("EN POSICIÓN DE REFERENCIA\r\n");
			} else {
				myprintf("EN POSICIÓN DE REFERENCIA\r\n");
			}
		} else {
			uart_error("COMANDO DISPONIBLE EN MODO STDBY");
		}
		break;

	case 'S':
		if (modo == M_STDBY) {
			if (flagCal && flagRef) {
				uint8_t res = verif_mpu();
				if (res) {
					myprintf("¿Desea registrar la sesión? (y/n)\r\n");
					input_state = INPUT_CONFIRM_SD; // se maneja en el callback
				} else {
					cambiar_modo(M_ERR);
					myprintf("Escriba 'y' para reintentar o 'n' para salir\r\n");
					myprintf("*ALRT: NO PODRÁ USAR ':S' SIN CONEXIÓN AL SENSOR\r\n");
					input_state = INPUT_ERR_MPU;
				}
			} else {
				if (!flagCal) uart_error("FALTA CALIBRACIÓN (':C')");
				if (!flagRef) uart_error("FALTA REFERENCIA (':R')");
			}
		} else {
			uart_error("COMANDO DISPONIBLE EN MODO STDBY");
		}
		break;

	case 'T':
		if (modo == M_STDBY) {
			if (flagRef) {
				cambiar_modo(M_TEST);
				myprintf("\r\nEnvíe una consigna ':[Y/P] [val]' o escriba ':E' para salir\r\n");
			} else {
				uart_error("FALTA REFERENCIA (':R')");
			}
		} else {
			uart_error("COMANDO DISPONIBLE EN MODO STDBY");
		}
		break;


	case 'M':
		if (modo == M_STDBY) {
			if (mount_sd()) {
				flagMem = hay_registros();	// devuelve 1 o 0
				if (flagRef && flagMem) {
					cambiar_modo(M_MEM);
					myprintf("======================= REGISTROS DISP. ========================\r\n");
					listar_registros();
					myprintf("================================================================\r\n\r\n");
					myprintf("Seleccione un registro por número para continuar o escriba ':E' para salir:\r\n");
					input_state = INPUT_ELEGIR_REG; // se sigue manejando en el callback
				} else {
					if (!flagRef) uart_error("FALTA REFERENCIA (':R')");
					if (!flagMem) uart_error("MEMORIA VACÍA");
				}
			} else {
				cambiar_modo(M_ERR);
				uart_error("NO SE DETECTA SD");
				myprintf("Escriba 'y' para reintentar o 'n' para salir\r\n");
				myprintf("*ALRT: NO PODRÁ USAR ':M' SIN UNA TARJETA SD\r\n");
				input_state = INPUT_ERR_SD;
			}
		} else {
			uart_error("COMANDO DISPONIBLE EN MODO STDBY");
		}
		break;

	case 'Y':
	    if (modo == M_TEST) {
	        if (strlen(rx_buffer) < 4) {
	            uart_error("FORMATO INVÁLIDO. Use ':Y [val]'");
	            break;
	        }

	        char* ptr = rx_buffer + 3; // después de ":Y "
	        float yaw = atof(ptr);

	        if (yaw < MIN_YAW || yaw > MAX_YAW) {
	            uart_error("VALOR FUERA DE RANGO PARA YAW");
	            myprintf("MIN_YAW: %.2f° | MAX_YAW: %.2f°\r\n", MIN_YAW, MAX_YAW);
	            break;
	        }

	        yaw_actual = yaw;
	        uint16_t pwm_yaw = ang_to_pwm(yaw_actual);
	        modificar_pwm(&htim2, TIM_CHANNEL_1, pwm_yaw);

	        myprintf("YAW = %.2f° (PWM: %u)\r\n", yaw_actual, pwm_yaw);
	        myprintf("\r\nVuelva a enviar una consigna o ingrese ':E' para salir\r\n");

	    } else {
	        uart_error("COMANDO DISPONIBLE EN MODO PRUEBA");
	    }
	    break;

	case 'P':
	    if (modo == M_TEST) {
	        if (strlen(rx_buffer) < 4) {
	            uart_error("FORMATO INVÁLIDO. Use ':P [val]'");
	            break;
	        }

	        char* ptr = rx_buffer + 3; // después de ":P "
	        float pitch = atof(ptr);

	        if (pitch < MIN_PITCH || pitch > MAX_PITCH) {
	            uart_error("VALOR FUERA DE RANGO PARA PITCH");
	            myprintf("MIN_PITCH: %.2f° | MAX_PITCH: %.2f°\r\n", MIN_PITCH, MAX_PITCH);
	            break;
	        }

	        pitch_actual = pitch;
	        uint16_t pwm_pitch = ang_to_pwm(pitch_actual);
	        modificar_pwm(&htim2, TIM_CHANNEL_2, pwm_pitch);

	        myprintf("PITCH = %.2f° (PWM: %u)\r\n", pitch_actual, pwm_pitch);
	        myprintf("\r\nVuelva a enviar una consigna o ingrese ':E' para salir\r\n");

	    } else {
	        uart_error("COMANDO DISPONIBLE EN MODO PRUEBA");
	    }
	    break;


	case 'L':
		if (modo == M_STDBY) {
			if (rx_buffer[2] == 'C') {
				if (!flagPID) {
					flagPID = 1;
					myprintf("LAZO CERRADO. CONTROL PID ACTIVADO PARA MODO SIM\r\n");
				} else {
					myprintf("El control PID ya se encuentra activado\r\n");
				}
			} else if (rx_buffer[2] == 'A') {
				if (flagPID) {
					flagPID = 0;
					myprintf("LAZO ABIERTO. CONTROL PID DESACTIVADO PARA MODO SIM\r\n");
				} else {
					myprintf("El control PID ya se encuentra desactivado\r\n");
				}
			} else {
				uart_error("COMANDO INCOMPLETO");
			}
		} else {
			uart_error("COMANDO DISPONIBLE EN MODO STDBY");
		}
		break;

	case 'K':
	    if (modo == M_STDBY) {

	        // Mostrar toda la configuración si se ingresa solo ":K"
	        if (strlen(rx_buffer) == 2) {
	        	myprintf("\r\nPID = %d\r\n", flagPID);
	        	myprintf("--------------------------------------\r\n");
	        	myprintf("              GANANCIAS               \r\n");
	            myprintf("Kp Y: %.2f | Ki Y: %.2f | Kd Y: %.2f\r\n",
	                     pid_yaw.kp, pid_yaw.ki, pid_yaw.kd);
	            myprintf("Kp P: %.2f | Ki P: %.2f | Kd P: %.2f\r\n",
	                     pid_pitch.kp, pid_pitch.ki, pid_pitch.kd);
	            myprintf("--------------------------------------\r\n\r\n");
	            break;
	        }

	        // Para otros comandos debe tener al menos 5 caracteres: ":KP Y"
	        if (strlen(rx_buffer) < 5) {
	            uart_error("COMANDO INCOMPLETO. FORMATO: ':K[P/I/D] [Y/P] [val]'");
	            break;
	        }

	        char tipo = rx_buffer[2];  // Debe ser 'P', 'I' o 'D'
	        char eje = rx_buffer[4];   // Debe ser 'Y' o 'P'

	        if ((tipo != 'P' && tipo != 'I' && tipo != 'D') ||
	            (eje != 'Y' && eje != 'P')) {
	            uart_error("FORMATO INVÁLIDO. Use ':K[P/I/D] [Y/P] [val]'");
	            break;
	        }

	        uint8_t axis = (eje == 'Y') ? 0 : 1;

	        // Modo consulta (solo :KP Y)
	        if (strlen(rx_buffer) <= 5) {
	            switch (tipo) {
	                case 'P':
	                    myprintf("Kp [%s] = %.2f\r\n", axis == 0 ? "YAW" : "PITCH",
	                             axis == 0 ? pid_yaw.kp : pid_pitch.kp);
	                    break;
	                case 'I':
	                    myprintf("Ki [%s] = %.2f\r\n", axis == 0 ? "YAW" : "PITCH",
	                             axis == 0 ? pid_yaw.ki : pid_pitch.ki);
	                    break;
	                case 'D':
	                    myprintf("Kd [%s] = %.2f\r\n", axis == 0 ? "YAW" : "PITCH",
	                             axis == 0 ? pid_yaw.kd : pid_pitch.kd);
	                    break;
	            }
	            break;
	        }

	        // Modo asignación (con valor)
	        if (strlen(rx_buffer) < 7) {
	            uart_error("VALOR FALTANTE. FORMATO: ':K[P/I/D] [Y/P] [val]'");
	            break;
	        }

	        char* valor_str = rx_buffer + 6;
	        float valor = atof(valor_str);

	        if (valor == 0.0f && valor_str[0] != '0') {
	            uart_error("VALOR NO VÁLIDO");
	            break;
	        }

	        if (valor < 0.0f) {
	            uart_error("NO SE PERMITEN GANANCIAS NEGATIVAS");
	            break;
	        }

	        switch (tipo) {
	            case 'P':
	                set_kp(axis, valor);
	                myprintf("Kp [%s] = %.2f\r\n", axis == 0 ? "YAW" : "PITCH", valor);
	                break;
	            case 'I':
	                set_ki(axis, valor);
	                myprintf("Ki [%s] = %.2f\r\n", axis == 0 ? "YAW" : "PITCH", valor);
	                break;
	            case 'D':
	                set_kd(axis, valor);
	                myprintf("Kd [%s] = %.2f\r\n", axis == 0 ? "YAW" : "PITCH", valor);
	                break;
	        }

	    } else {
	        uart_error("COMANDO DISPONIBLE EN MODO STDBY");
	    }
	    break;

	case 'E':
		if (modo != M_STDBY) {
			cambiar_modo(M_STDBY);
		} else {
			myprintf("Ya se encuentra en MODO STDBY\r\n");
		}

		break;

	case 'H':
		if (modo == M_STDBY) {
			myprintf(
			    "\r\n========================== COMANDOS ============================\r\n"
				"Calibrar sensor               --> :C                    | STDBY\r\n"
				"Referenciar motores           --> :R                    | STDBY\r\n"
				"Lazo cerrado (PID)/abierto    --> :L[C/A]               | STDBY\r\n"
				"Establecer ganancias PID Y/P  --> :K[P/I/D] [Y/P] [val] | STDBY\r\n"
				"Ver ganancias PID             --> :K[P/I/D] [Y/P] o :K  | STDBY\r\n"
				"Modo 'Simulación'             --> :S                    | STDBY\r\n"
				"Modo 'Test'                   --> :T                    | STDBY\r\n"
				"Consigna angular Y/P          --> :[Y/P] [val]          | TEST\r\n"
			    "Modo 'Memoria'                --> :M                    | STDBY\r\n"
			    "Salir a Modo 'Standby'        --> :E                    | -----\r\n"
			    "Mostrar ayuda                 --> :H                    | STDBY\r\n"
			    "================================================================\r\n"
			    "INFO: Cal = %d | Ref = %d | PID = %d\r\n\r\n",
			    flagCal, flagRef, flagPID
			);
		} else {
			uart_error("COMANDO DISPONIBLE EN MODO STDBY");
		}
		break;

	default:
		uart_error("COMANDO NO RECONOCIDO. Use ':H' para ver ayuda");
		break;
	}
}

/* ----------------------------------------------------------------------------
 * MÁQUINA DE ESTADOS
 * ---------------------------------------------------------------------------- */

void cambiar_modo(mode_t est) {
	modo = est;
	maq_estados();	// llamar a maq_estados
}

void maq_estados(void) {
    switch (modo) {
        case M_ERR:
        	reset_leds();	// Apagar todos los LEDs
        	HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_SET);	// Encender LED Rojo
        	// REVISAR:
        	detener_registro();		// Detener registro de manera segura
        	flagRepro = 0;			// Detener reproducción
        	reset_all_pids();		// Resetear PID
            break;
        case M_STDBY:
        	myprintf("----------------\r\n");
        	myprintf("MODO: STANDBY\r\n");
        	myprintf("----------------\r\n");
        	reset_leds();			// Apagar todos los LEDs
        	detener_registro();		// Detener registro de manera segura
        	flagRepro = 0;			// Detener reproducción
        	reset_all_pids();		// Resetear PID
            break;
        case M_SIM:
        	myprintf("----------------\r\n");
        	myprintf("MODO: SIMULACIÓN\r\n");
        	myprintf("----------------\r\n");
        	reset_leds();	// Apagar todos los LEDs
        	flagRef = 0;	// Reset flagRef porque los servos pueden haber quedado en cualquier lado
            break;
        case M_TEST:
        	myprintf("----------------\r\n");
        	myprintf("MODO: TEST\r\n");
        	myprintf("----------------\r\n");
        	reset_leds();	// Apagar todos los LEDs
        	HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_SET); // Encender LED Verde
        	flagRef = 0;	// Reset flagRef porque los servos pueden haber quedado en cualquier lado
            break;
        case M_MEM:
            myprintf("----------------\r\n");
        	myprintf("MODO: MEMORIA\r\n");
        	myprintf("----------------\r\n");
        	reset_leds();	// Apagar todos los LEDs
        	HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_SET); // Encender LED Azul
            break;
    }
}

void reset_leds(void) {
	HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_RESET);
}

/* ----------------------------------------------------------------------------
 * VERIFICACIÓN DE COMPONENTES
 * ---------------------------------------------------------------------------- */

// SD -------------------------------------------------------------------------

uint8_t mount_sd(void) {
	FRESULT fres;

    USER_SPI_initialize(0);

	// desmontar y limpiar
	f_mount(NULL, "", 0);

	// montar la tarjeta SD
	fres = f_mount(&SDFatFS, "", 1);

	if (fres != FR_OK) {
		myprintf("SD NO MONTADA - f_mount (%d)\r\n", fres);
	    return 0;  // falla
	}

	return 1;
}

// ----
uint8_t verif_sd(void) {

    DWORD free_clusters, free_sectors, total_sectors;
    FATFS *getFreeFs;
    FRESULT fres;

    if (!mount_sd()) {
    	return 0;  // falla
    }

    // Obtener estadísticas de la tarjeta
    fres = f_getfree("", &free_clusters, &getFreeFs);

    if (fres != FR_OK) {
    	myprintf("NO SE PUEDE LEER LA SD - f_getfree (%d)\r\n", fres);
        return 0;  // falla
    }

    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    myprintf("------------------------\r\n");
    myprintf("Espacio\r\n");
    myprintf(
        "TOTAL: %lu KiB\r\n"
        "LIBRE: %lu KiB\r\n",
        total_sectors / 2, free_sectors / 2);
    myprintf("------------------------\r\n");

    return 1;  // ok
}

// SERVOS ----------------------------------------------------------------------

uint8_t verif_servos(void) {
	servo_actual = 1;  // Comenzar con servo 1 (YAW)

	myprintf("-----------------------------------------------------------------\r\n");
	myprintf("Presione el botón para comenzar la verificación del SERVO 1 (YAW)\r\n");

	// Cambiar estado para esperar botón
	input_state = INPUT_SERVO_WAIT_B1;
	HAL_UART_Receive_IT(&huart2, &rx_char, 1);

	return 1; // Retorna inmediatamente, la verificación continúa de forma asíncrona
}

void servo_sequence_task(void) {
    if (!servo_sequence_active) return;

    if (servo_timer > 0) {
        servo_timer--;
        return;
    }

    if (servo_actual == 1) {
        // Secuencia SERVO 1 (YAW)
        switch (servo_step) {
            case 0:
                modificar_pwm(&htim2, TIM_CHANNEL_1, ang_to_pwm(REF_YAW));
                servo_timer = 500; // 0.5s
                servo_step = 1;
                break;
            case 1:
                modificar_pwm(&htim2, TIM_CHANNEL_1, ang_to_pwm(MIN_YAW));
                servo_timer = 500; // 0.5s
                servo_step = 2;
                break;
            case 2:
                modificar_pwm(&htim2, TIM_CHANNEL_1, ang_to_pwm(MAX_YAW));
                servo_sequence_active = 0;
                servo_step = 0;
                myprintf("¿Se movió correctamente el servo? (y/n): ");
                input_state = INPUT_SERVO_CONFIRM;
                break;
        }
    } else {
        // Secuencia SERVO 2 (PITCH)
        switch (servo_step) {
            case 0:
                modificar_pwm(&htim2, TIM_CHANNEL_2, ang_to_pwm(REF_PITCH));
                servo_timer = 500; // 0.5s
                servo_step = 1;
                break;
            case 1:
                modificar_pwm(&htim2, TIM_CHANNEL_2, ang_to_pwm(MIN_PITCH));
                servo_timer = 500; // 0.5s
                servo_step = 2;
                break;
            case 2:
                modificar_pwm(&htim2, TIM_CHANNEL_2, ang_to_pwm(MAX_PITCH));
                servo_sequence_active = 0;
                servo_step = 0;
                myprintf("¿Se movió correctamente el servo? (y/n): ");
                input_state = INPUT_SERVO_CONFIRM;
                break;
        }
    }
}

// MPU ----------------------------------------------------------------------

uint8_t i2c_bus_is_free(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Configurar PB8 (SCL) y PB9 (SDA) como entrada flotante
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 2. Leer el estado de los pines
    GPIO_PinState scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    GPIO_PinState sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    // 3. Evaluar: si ambos están en alto, el bus está libre
    return (sda == GPIO_PIN_SET && scl == GPIO_PIN_SET);
}

void i2c_recover_bus(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Desinicializar el periférico I2C
    HAL_I2C_DeInit(&hi2c1);

    // 2. Configurar SDA (PB9) y SCL (PB8) como GPIO de salida open-drain
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Liberar el bus: generar hasta 9 pulsos de reloj en SCL para desbloquear SDA
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            break;  // SDA está libre, no es necesario seguir

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL high
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL low
    }

    // 4. Generar condición de STOP manual
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);  // SDA low
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);    // SCL high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);    // SDA high

    // 5. Reconfigurar los pines como I2C y reinicializar I2C1
    MX_I2C1_Init();
}

uint8_t verif_mpu(void) {

    void i2c_recover_bus(void);

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    // 1. Verificación física
    if (!i2c_bus_is_free()) {
    	mpu6050_print_error(99);
        cambiar_modo(M_ERR);
        input_state = INPUT_VERIF;
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
        return 0;
    }

    // 2. Reset completo del periférico
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    MX_I2C1_Init();

    // 3. Verificar si el sensor responde
    uint8_t a0 = mpu6050_ready(&hi2c1);
    if (a0 != MPU6050_OK) {
        mpu6050_print_error(a0);
        return 0;
    }

    // 4. Inicializar
    uint8_t a1 = mpu6050_init(&hi2c1);
    if (a1 != MPU6050_OK) {
        mpu6050_print_error(a1);
        return 0;
    }

    // 5. Configurar
    uint8_t a2 = mpu6050_config(&hi2c1);
    if (a2 != MPU6050_OK) {
        mpu6050_print_error(a2);
        return 0;
    }

    return 1;	// todo ok
}


void rutina_verificacion(void) {
    // Resetear flag de verificación al inicio
    flagVerif = 0;

    myprintf("\r\nVerificando SD...\r\n");
    // Verificar SD
    if (!verif_sd()) {
    	cambiar_modo(M_ERR);
        input_state = INPUT_VERIF;
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
        return;  // Si falla, salir con flagVerif = 0
    }
    myprintf("SD: OK\r\n");

    myprintf("\r\nVerificando sensor...\r\n");
    // Verificar MPU
     if (!verif_mpu()) {
		cambiar_modo(M_ERR);
		input_state = INPUT_VERIF;
		HAL_UART_Receive_IT(&huart2, &rx_char, 1);
		return;  // Si falla, salir con flagVerif = 0
     }
    myprintf("Sensor: OK\r\n");

    myprintf("\r\nVerificando servomotores...\r\n");
    // Verificar servos
    verif_servos();
}

/* ----------------------------------------------------------------------------
 * MODO MEM / SD
 * ---------------------------------------------------------------------------- */

// TODO - buscar solo archivos .txt
uint8_t hay_registros(void) {
    DIR dir;
    FILINFO fno;

    f_opendir(&dir, "/");

    while (1) {
        f_readdir(&dir, &fno);
        if (fno.fname[0] == 0) break; // Fin del directorio

        if (!(fno.fattrib & AM_DIR)) {
            // Se encontró al menos un archivo
            f_closedir(&dir);
            return 1; // No vacía
        }
    }

    f_closedir(&dir);
    return 0; // Vacía
}

// Cerrar archivo, dejar de registrar
void detener_registro(void) {
    if (flagReg) {
        f_sync(&archivo_log);   // Sync final
        f_close(&archivo_log);  // Cerrar
        flagReg = 0;
    }
}

void listar_registros(void) {
    DIR dir;
    FILINFO fno;
    FRESULT fres;
    uint8_t contador = 0;

    fres = f_opendir(&dir, "/");
    if (fres != FR_OK) {
        uart_error("NO SE PUDO ABRIR EL DIRECTORIO");
        return;
    }

    while (1) {
            fres = f_readdir(&dir, &fno);
            if (fres != FR_OK || fno.fname[0] == 0) break; // Error o fin del directorio

            if (!(fno.fattrib & AM_DIR)) {
            	    contador++;

            	    if (fno.fsize < 1024) {
            	        sprintf(tx_buffer, "%2d. %-20s [%lu bytes]\r\n",
            	                contador, fno.fname, fno.fsize);
            	    } else {
            	        sprintf(tx_buffer, "%2d. %-20s [%lu KB]\r\n",
            	                contador, fno.fname, fno.fsize / 1024);
            	    }

            	    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
            }
        }

        f_closedir(&dir);

        if (contador == 0) {
        	uart_error("NO SE ENCONTRARON REGISTROS");
        } else {
        	myprintf("\r\nTotal: %d registro(s) encontrado(s)\r\n", contador);
        }
}

void ver_registro(const char* nombre) {
    FIL archivo;
    FRESULT fres;
    char linea[128];
    int contador = 0;

    fres = f_open(&archivo, nombre, FA_READ);
    if (fres != FR_OK) {
        uart_error("NO SE PUDO ABRIR EL ARCHIVO");
        return;
    }

    // Leer primera línea
    if (!f_gets(linea, sizeof(linea), &archivo)) {
        uart_error("ERROR AL LEER EL ARCHIVO");
        f_close(&archivo);
        return;
    }

    // Ver si es un archivo nuevo (con línea "PID:")
    if (strncmp(linea, "PID:", 4) == 0) {
        myprintf("\r\n[INFO] %s", linea); // Mostrar línea PID
        uint8_t pid_enabled = atoi(linea + 5);

        if (pid_enabled == 1) {
            if (f_gets(linea, sizeof(linea), &archivo)) {
                myprintf("\r\n[INFO] %s", linea); // Mostrar línea de ganancias
            }
        }

        // Saltar línea de cabecera
        f_gets(linea, sizeof(linea), &archivo);
    } else {
        // Si no empieza con "PID:", volver al inicio del archivo
        f_lseek(&archivo, 0);
        f_gets(linea, sizeof(linea), &archivo);  // Leer cabecera
    }

    myprintf("\r\n======================= VISTA PREVIA =======================\r\n");
    myprintf("  t [ms] | yaw_sens | pitch_sens | yaw_servo | pitch_servo\r\n");
    myprintf("------------------------------------------------------------\r\n");

    while (contador < 15 && !f_eof(&archivo)) {
        char* res = f_gets(linea, sizeof(linea), &archivo);
        if (res == NULL) break;

        if (linea[0] == '\0' || linea[0] == '\r' || linea[0] == '\n') continue;

        char* token = strtok(linea, ";");
        int t_ms = 0;
        float yaw_sens = 0.0f, pitch_sens = 0.0f;
        int yaw_servo = 0, pitch_servo = 0;
        int i = 0;

        while (token != NULL && i < 5) {
            while (*token == ' ') token++;

            switch (i) {
                case 0: t_ms = atoi(token); break;
                case 1: yaw_sens = atof(token); break;
                case 2: pitch_sens = atof(token); break;
                case 3: yaw_servo = atoi(token); break;
                case 4: pitch_servo = atoi(token); break;
            }

            token = strtok(NULL, ";");
            i++;
        }

        if (i == 5) {
            snprintf(tx_buffer, sizeof(tx_buffer), "%8d | %8.2f | %10.2f | %9d | %11d\r\n",
                     t_ms, yaw_sens, pitch_sens, yaw_servo, pitch_servo);
            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
            contador++;
        }
    }

    myprintf("============================================================\r\n\r\n");
    f_close(&archivo);
}

void reproducir_registro(const char* nombre) {
    FRESULT fres;

    fres = f_open(&archivo_repro, nombre, FA_READ);
    if (fres != FR_OK) {
        uart_error("NO SE PUDO ABRIR EL ARCHIVO");
        return;
    }

    // Leer primera línea
    char linea_temp[128];
    if (!f_gets(linea_temp, sizeof(linea_temp), &archivo_repro)) {
        uart_error("ERROR AL LEER EL ARCHIVO");
        f_close(&archivo_repro);
        return;
    }

    if (strncmp(linea_temp, "PID:", 4) == 0) {
        uint8_t pid_enabled = atoi(linea_temp + 5);

        if (pid_enabled == 1) {
            // Saltar línea con ganancias
            f_gets(linea_temp, sizeof(linea_temp), &archivo_repro);
        }

        // Saltar línea de cabecera
        f_gets(linea_temp, sizeof(linea_temp), &archivo_repro);
    } else {
        // Si no comienza con "PID", asumimos archivo antiguo → retroceder al inicio
        f_lseek(&archivo_repro, 0);
    }

    flagRepro = 1;
    contador_repro = 0;
    myprintf("PUEDE ESCRIBIR :E EN CUALQUIER MOMENTO PARA SALIR\r\n");
}


/* ----------------------------------------------------------------------------
 * CALIBRACIÓN
 * ---------------------------------------------------------------------------- */

//// Calcular offset gyro
//void calc_gyro_offsets(void) {
//
//	for (int i = 0; i < 1500; i++) {
//		mpu6050_read(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);
//
//		sum_x_gyro += gx;
//		sum_y_gyro += gy;
//	}
//
//	offset_x_gyro = (float) sum_x_gyro / 1500.0f;	// Promedio
//	offset_y_gyro = (float) sum_y_gyro / 1500.0f;
//}
//
//// Calibrar sensor - encontrar offsets medidas filtradas
//void calibrar_mpu(void) {
//
//	float sum_ang_filtrado_x = 0.0f;
//	float sum_ang_filtrado_y = 0.0f;
//
//	for (int i = 0; i < 1500; i++) {
//
//		static float dt1 = 0.005f;
//
//		lectura_filtrado_mpu(dt1);
//
//		sum_ang_filtrado_x += ang_filtrado_x;
//		sum_ang_filtrado_y += ang_filtrado_y;
//	}
//
//	ref_x = sum_ang_filtrado_x / 1500.0f;	// Promedio
//	ref_y = sum_ang_filtrado_y / 1500.0f;
//}

// Calcular offset gyro con manejo de errores
uint8_t calc_gyro_offsets(void) {

    for (int i = 0; i < 1500; i++) {
        uint8_t res = mpu6050_read(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);

        if (res != MPU6050_OK) {
            myprintf("\r\n");
            mpu6050_print_error(res);
            cambiar_modo(M_ERR);
            myprintf("ALRT: FALLA EN LECTURA DE MPU DURANTE CALIBRACIÓN\r\n");
            myprintf("Escriba 'y' para reintentar o 'n' para salir\r\n");
            //show_prompt();
            input_state = INPUT_ERR_MPU;
            HAL_UART_Receive_IT(&huart2, &rx_char, 1);
            return res; // Retornar el código de error
        }

        sum_x_gyro += gx;
        sum_y_gyro += gy;
    }

    offset_x_gyro = (float) sum_x_gyro / 1500.0f; // Promedio
    offset_y_gyro = (float) sum_y_gyro / 1500.0f;

    return MPU6050_OK; // Éxito
}

// Calibrar sensor con manejo de errores
uint8_t calibrar_mpu(void) {

    float sum_ang_filtrado_x = 0.0f;
    float sum_ang_filtrado_y = 0.0f;

    for (int i = 0; i < 1500; i++) {

        static float dt1 = 0.005f;

        // Verificar si lectura_filtrado_mpu detectó error
        uint8_t res = mpu6050_read(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);

        if (res != MPU6050_OK) {
            myprintf("\r\n");
            mpu6050_print_error(res);
            cambiar_modo(M_ERR);
            myprintf("ALRT: FALLA EN LECTURA DE MPU DURANTE CALIBRACIÓN\r\n");
            myprintf("Escriba 'y' para reintentar o 'n' para salir\r\n");
            //show_prompt();
            input_state = INPUT_ERR_MPU;
            HAL_UART_Receive_IT(&huart2, &rx_char, 1);
            return res; // Retornar el código de error
        }

        // Continuar con el procesamiento normal
        cal_angulos_acelerometro(ax, ay, az, &ang_accx, &ang_accy);

        gx_corregido = gx - (int16_t) offset_x_gyro;
        gy_corregido = gy - (int16_t) offset_y_gyro;

        gx_dps = mpu6050_gyro_raw_to_dps(gx_corregido);
        gy_dps = mpu6050_gyro_raw_to_dps(gy_corregido);

        ang_gyrox = ang_filtrado_x + gx_dps * dt1;
        ang_gyroy = ang_filtrado_y + gy_dps * dt1;

        ang_filtrado_x = (POND_GYRO * ang_gyrox + POND_ACC * ang_accx);
        ang_filtrado_y = (POND_GYRO * ang_gyroy + POND_ACC * ang_accy);

        sum_ang_filtrado_x += ang_filtrado_x;
        sum_ang_filtrado_y += ang_filtrado_y;
    }

    ref_x = sum_ang_filtrado_x / 1500.0f; // Promedio
    ref_y = sum_ang_filtrado_y / 1500.0f;

    return MPU6050_OK; // Éxito
}

/* ----------------------------------------------------------------------------
 * MPU
 * ---------------------------------------------------------------------------- */

// Sat medida mpu
float sat_ang_mpu(float ang) {
	if (ang > MAX_XY) {
		return MAX_XY;
	} else if (ang < MIN_XY) {
		return MIN_XY;
	} else {
		return ang;
	}
}

// Pasar de medicion saturada de mpu a angulo servo
float mpu_to_servo(float ang_mpu, uint8_t ind) {
	if (ind == 0) {
		// YAW
		float m = (MAX_YAW - MIN_YAW) / (MAX_XY - MIN_XY);
		return m*(ang_mpu - MIN_XY) + MIN_YAW;
	} else if (ind == 1) {
		// PITCH
		float m = (MAX_PITCH - MIN_PITCH) / (MAX_XY - MIN_XY);
		return m*(ang_mpu - MIN_XY) + MIN_PITCH;
	} else {
		return 0.0f;
	}
}

// Pasar de angulo filtrado mpu a valor pwm servo
uint16_t filt_to_pwm(float filt, uint8_t ind) {
	float ang_sat = sat_ang_mpu(filt);
	set_yp_reg(ang_sat, ind);
	float yp = mpu_to_servo(ang_sat, ind);
	return ang_to_pwm(yp);
}

void set_yp_reg(float ang, uint8_t ind) {
	if (ind == 0) {
		ang_yaw_sat = ang;
	} else if (ind == 1) {
		ang_pitch_sat = ang;
	}
}

void cal_angulos_acelerometro(int16_t ax, int16_t ay, int16_t az, float *ang_accx, float *ang_accy) {
    // Calcular ángulo de rotación alrededor del eje X en grados
    *ang_accx = atan2((float)ay, (float)az) * 180.0f / M_PI;

    // Calcular ángulo de rotación alrededor del eje Y en grados
    *ang_accy = atan2(-(float)ax, sqrt((float)ay * (float)ay + (float)az * (float)az)) * 180.0f / M_PI;
}

void lectura_filtrado_mpu(float dt) {

    // 1. Intentar leer el sensor
    uint8_t res = mpu6050_read(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);

    if (res != MPU6050_OK) {
    	myprintf("\r\n");
        mpu6050_print_error(res);

        if (modo == M_SIM) {
        	cambiar_modo(M_ERR);
            myprintf("*ALRT: FALLA EN LECTURA DE MPU DURANTE MODO SIM\r\n");
            myprintf("Escriba 'y' para reintentar o 'n' para salir\r\n");
            show_prompt();
            input_state = INPUT_ERR_MPU;
            HAL_UART_Receive_IT(&huart2, &rx_char, 1);
        }

        return;
    }

    // 2. Calcular ángulos del acelerómetro
    cal_angulos_acelerometro(ax, ay, az, &ang_accx, &ang_accy);

    // 3. Corregir medidas del giroscopio con los offsets
    gx_corregido = gx - (int16_t) offset_x_gyro;
    gy_corregido = gy - (int16_t) offset_y_gyro;

    gx_dps = mpu6050_gyro_raw_to_dps(gx_corregido);
    gy_dps = mpu6050_gyro_raw_to_dps(gy_corregido);

    // 4. Integración del giroscopio
    ang_gyrox = ang_filtrado_x + gx_dps * dt;
    ang_gyroy = ang_filtrado_y + gy_dps * dt;

    // 5. Filtro complementario
    ang_filtrado_x = (POND_GYRO * ang_gyrox + POND_ACC * ang_accx);
    ang_filtrado_y = (POND_GYRO * ang_gyroy + POND_ACC * ang_accy);
}


void lectura_filtrado_mpu_calibrada(float dt) {
	lectura_filtrado_mpu (dt);

	ang_filtrado_cal_x = ang_filtrado_x - ref_x;
	ang_filtrado_cal_y = ang_filtrado_y - ref_y;

}


void mpu6050_print_error(uint8_t err_code) {
    switch (err_code) {
        case MPU6050_ERR_NO_I2C:
            uart_error("MPU6050 > NO RESPONDE POR I2C");
            break;
        case MPU6050_ERR_READ_REG:
            uart_error("MPU6050 > NO FUE POSIBLE LEER EL REG. WHO_AM_I");
            break;
        case MPU6050_ERR_WHO_AM_I:
            uart_error("MPU6050 > VALOR INESPERADO WHO_AM_I");
            break;
        case MPU6050_ERR_PWR_MGMT_WRITE:
            uart_error("MPU6050 > NO FUE POSIBLE DESPERTAR EL SENSOR");
            break;
        case MPU6050_ERR_GYRO_CONFIG:
            uart_error("MPU6050 > NO FUE POSIBLE CONFIGURAR GYRO");
            break;
        case MPU6050_ERR_ACC_CONFIG:
            uart_error("MPU6050 > NO FUE POSIBLE CONFIGURAR EL ACCEL");
            break;
        case MPU6050_ERR_FILTER_CONFIG:
            uart_error("MPU6050 > NO FUE POSIBLE CONFIGURAR EL FILTRO LPF");
            break;
        case MPU6050_ERR_RESET:
            uart_error("MPU6050 > FALLO AL RESETEAR EL SENSOR");
            break;
        case MPU6050_ERR_READ_DATA:
            uart_error("MPU6050 > NO ES POSIBLE LEER EL SENSOR");
            break;
        case 99:
        	uart_error("MPU6050 > BUS I2C BLOQUEADO");
        	break;
        default:
            uart_error("MPU6050 > ERROR DESCONOCIDO");
            break;
    }
}

/* ----------------------------------------------------------------------------
 * SERVOMOTORES
 * ---------------------------------------------------------------------------- */

uint16_t ang_to_pwm(float angle) {
	return (uint16_t)(angle * (2000.0f / 180.0f) + 500.0f);
}

void modificar_pwm(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pwm_pulse) {
	__HAL_TIM_SET_COMPARE(htim, channel, pwm_pulse);
}

// Uso: modificar_pwm(&htim2, TIM_CHANNEL_1, pwm_pulse), modificar_pwm(&htim2, TIM_CHANNEL_2, pwm_pulse)

void servos_ref(void) {

	// SERVO 1 - YAW
	uint16_t pwm_ref_yaw = ang_to_pwm(REF_YAW);
	modificar_pwm(&htim2, TIM_CHANNEL_1, pwm_ref_yaw);

	// SERVO 2 - PITCH
	uint16_t pwm_ref_pitch = ang_to_pwm(REF_PITCH);
	modificar_pwm(&htim2, TIM_CHANNEL_2, pwm_ref_pitch);
}

void yaw_pitch(float y, float p) {

	if (y < MIN_YAW) {
		y = MIN_YAW;
	}

	if (y > MAX_YAW) {
		y = MAX_YAW;
	}

	if (p < MIN_PITCH) {
		p = MIN_PITCH;
	}

	if (p > MAX_PITCH) {
		p = MAX_PITCH;
	}

	uint16_t yy = ang_to_pwm(y);
	uint16_t pp = ang_to_pwm(p);

	modificar_pwm(&htim2, TIM_CHANNEL_1, yy);
	modificar_pwm(&htim2, TIM_CHANNEL_2, pp);

}

/* ----------------------------------------------------------------------------
 * PID
 * ---------------------------------------------------------------------------- */

// Inicialización del PID
void init_pid(PID_Controller* pid, float kp, float ki, float kd, float max_output, float min_output) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->max_output = max_output;
    pid->min_output = min_output;
}

// PID con anti-windup
float calculate_pid(PID_Controller* pid, float setpoint, float measured_value, float dt) {
    // Calcular error
    float error = setpoint - measured_value;

    // ZONA MUERTA: Si el error es muy pequeño, tratarlo como cero
    if (fabs(error) < 0.5f) {  // 0.5 grados de zona muerta
        error = 0.0f;
        pid->integral *= 0.95f;  // Reducir integral gradualmente
    }

    // Término proporcional
    float proportional = pid->kp * error;

    // Término derivativo
    float derivative = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Calcular salida temporal (sin integral actualizado)
    float temp_output = proportional + (pid->ki * pid->integral) + derivative;

    // Anti-windup condicional: solo integrar si la salida NO está saturada
    if (temp_output <= pid->max_output && temp_output >= pid->min_output) {
        // La salida no está saturada, podemos acumular el integral
        pid->integral += error * dt;
    }
    // Si está saturada, NO acumulamos el integral (anti-windup)

    // Término integral (con el valor actualizado o mantenido)
    float integral = pid->ki * pid->integral;

    // Salida total del PID
    float output = proportional + integral + derivative;

    // Saturar salida final
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }

    return output;
}

// Resetear PID
void reset_pid(PID_Controller* pid) {
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

// Inicialización de los controladores PID
void init_pid_controllers(void) {
    // Para YAW (eje Y)
    init_pid(&pid_yaw,
             Kp_yaw,
             Ki_yaw,
             Kd_yaw,
             (MAX_YAW - MIN_YAW)/2.0f,   // max_output
             -(MAX_YAW - MIN_YAW)/2.0f // min_output
    );

    // Para PITCH (eje X)
    init_pid(&pid_pitch,
             Kp_pitch,
             Ki_pitch,
             Kd_pitch,
             (MAX_PITCH - MIN_PITCH)/2.0f,   // max_output
             -(MAX_PITCH - MIN_PITCH)/2.0f // min_output
    );
}

// Función para usar PID en la interr. Modo SIM
uint16_t filt_to_pwm_pid(float measured_angle, uint8_t ind, float dt) {
    float setpoint = 0.0f; // Se desea mantener el cohete vertical (0.0f en ambos ejes)
    float pid_output;
    float servo_angle;

    float ang_sat = sat_ang_mpu(measured_angle);

    if (ind == 0) {
        // YAW
        pid_output = calculate_pid(&pid_yaw, setpoint, -ang_sat, dt);
        // El PID nos da la corrección necesaria, la aplicamos al centro del servo
        servo_angle = REF_YAW + pid_output;
        // Guardar valor saturado del angulo medido
        set_yp_reg(ang_sat, ind);
    } else if (ind == 1) {
        // PITCH
        pid_output = calculate_pid(&pid_pitch, setpoint, -ang_sat, dt);
        servo_angle = REF_PITCH + pid_output;
        // Guardar valor saturado del angulo medido
        set_yp_reg(ang_sat, ind);
    } else {
        return 0;
    }

    return ang_to_pwm(servo_angle);
}

// Funciones para ajustar parámetros PID
void set_kp(uint8_t axis, float kp) {
    if (axis == 0) {
        pid_yaw.kp = kp;
    } else if (axis == 1) {
        pid_pitch.kp = kp;
    }
}

void set_ki(uint8_t axis, float ki) {
    if (axis == 0) {
        pid_yaw.ki = ki;
    } else if (axis == 1) {
        pid_pitch.ki = ki;
    }
}

void set_kd(uint8_t axis, float kd) {
    if (axis == 0) {
        pid_yaw.kd = kd;
    } else if (axis == 1) {
        pid_pitch.kd = kd;
    }
}

// Función para resetear los PIDs
void reset_all_pids(void) {
    reset_pid(&pid_yaw);
    reset_pid(&pid_pitch);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	/* ============================================================================
	 * INICIALIZACIÓN
	 * ============================================================================ */

	// Inicializar PWM en los timers y canales correspondientes
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Servo 1 (YAW)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Servo 2 (PITCH)
	// Inicializar timer para espera mov. servomotores (verif.)
	HAL_TIM_Base_Start_IT(&htim3);
	// Inicializar timer para parpadeo de LED Verde (Modo SIM)
	HAL_TIM_Base_Start_IT(&htim4);
	// Inicializar timer para lectura sensor, movimiento servos, registros, etc. Modo SIM/Modo MEM
	HAL_TIM_Base_Start_IT(&htim5);

	/* ============================================================================
	 * RUTINA DE VERIFICACIÓN DE COMPONENTES
	 * ============================================================================ */
	HAL_UART_Transmit(&huart2, (uint8_t*) clear, strlen(clear), 10); 	// Limpiar terminal
	clear_buffer();														// Limpiar buffer

	// Mensajes iniciales
	myprintf("================================================================\r\n");
	myprintf("BIENVENIDO\r\n");
	myprintf("================================================================\r\n\r\n");

	myprintf("*ALRT: SE REQUIERE REALIZAR VERIFICACIÓN DE COMPONENTES\r\n");
	myprintf("Presione cualquier tecla para comenzar...\r\n");

	modo = M_ERR;					// Comienza en modo ERR por default
	input_state = INPUT_VERIF;  	// Comienza esperando confrimación para realizar rutina de verificación

	maq_estados();					// Llamada a maq_estados

	// Inicializar interrupciones UART
	HAL_UART_Receive_IT(&huart2, &rx_char, 1);

	// Inicializar PID
	init_pid_controllers();

	// Como está en INPUT_VERIF, entra a ese case del switch

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8400-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 500-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_AZUL_Pin|LED_VERDE_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Button_Pin */
  GPIO_InitStruct.Pin = B1_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ROJO_Pin */
  GPIO_InitStruct.Pin = LED_ROJO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ROJO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_AZUL_Pin LED_VERDE_Pin */
  GPIO_InitStruct.Pin = LED_AZUL_Pin|LED_VERDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ============================================================================
 * CALLBACK UART
 * ============================================================================ */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {

        // INPUT_VERIF SE MANEJA APARTE
        if (input_state == INPUT_VERIF) {
        	myprintf("\r\n================================================================\r\n");
            myprintf("INICIANDO RUTINA DE VERIFICACIÓN...\r\n");
            rutina_verificacion();

            // SOLO revisar flagVerif si NO estamos en proceso de verificación de servos
            if (input_state != INPUT_SERVO_WAIT_B1 && input_state != INPUT_SERVO_CONFIRM) {
                if (flagVerif == 0) {
                	uart_error("FALLA EN LA VERIFICACIÓN");
                	myprintf("\r\nPresione cualquier tecla para reintentar...\r\n");
                }
            }

            clear_buffer();
            HAL_UART_Receive_IT(&huart2, &rx_char, 1);
            return;
        }
    	if (rx_char == '\r' || rx_char == '\n') {
    		rx_buffer[rx_index] = 0;

    		 myprintf("\r\n");	// Nueva linea

    		 if (rx_index > 0) {
    			 switch(input_state) {
    			 case INPUT_VERIF:
    				 // se maneja fuera del switch
    				 break;

                 case INPUT_SERVO_WAIT_B1:
                     // En este estado solo se espera el botón, no comandos UART
                     uart_error("PRESIONE EL BOTÓN PARA CONTINUAR");
                     break;

                 case INPUT_SERVO_CONFIRM:
                     if (strcmp(rx_buffer, "y") == 0 || strcmp(rx_buffer, "Y") == 0) {

                         if (servo_actual == 1) {
                             myprintf("SERVO 1 (YAW): OK\r\n");
                             servo_actual = 2;
                             myprintf("\r\nPresione el botón para comenzar la verificación del SERVO 2 (PITCH)\r\n");
                             input_state = INPUT_SERVO_WAIT_B1;
                         } else {
                             myprintf("SERVO 2 (PITCH): OK\r\n");
                         	 myprintf("-----------------------------------------------------------------\r\n");
                             myprintf("Servomotores: OK\r\n");

                             flagVerif = 1;
                             myprintf("\r\nVERIFICACIÓN COMPLETADA\r\n");
                             myprintf("================================================================\r\n");
                             cambiar_modo(M_STDBY);
                             myprintf("\r\nUse ':H' para ver los comandos disponibles\r\n");
                             input_state = INPUT_NORMAL;
                         }

                     } else if (strcmp(rx_buffer, "n") == 0 || strcmp(rx_buffer, "N") == 0) {
                         myprintf("SERVO %d FALLÓ\r\n", servo_actual);
                         cambiar_modo(M_ERR);
                         uart_error("FALLA EN LA VERIFICACIÓN");
                         myprintf("\r\nPresione cualquier tecla para reintentar...\r\n");
                         input_state = INPUT_VERIF;

                     } else {
                         uart_error("RESPUESTA INVÁLIDA. Use 'y' o 'n'");
                         myprintf("¿Se movió correctamente el servo? (y/n): ");
                     }
                     break;


    			 case INPUT_NORMAL:
    				 if (rx_buffer[0] == ':') {
    					 interprete();	// Interpretar comando
    				 } else {
    					 uart_error("COMANDOS DEBEN EMPEZAR CON ':'");
    				 }
    				 break;

                 case INPUT_CONFIRM_SD:
                     if (strcmp(rx_buffer, "y") == 0 || strcmp(rx_buffer, "Y") == 0) {
                    	 if (mount_sd()) {
                    		 myprintf("REGISTRO ACTIVADO\r\n");
                    		 myprintf("Ingrese ID de la sesión (debe ser un entero XX):\r\n");
                    		 input_state = INPUT_ID;
                    	 } else {
                    		 cambiar_modo(M_ERR);
                    		 uart_error("NO SE DETECTA SD");
                    		 myprintf("Escriba 'y' para reintentar o 'n' para continuar sin registro\r\n");
                    		 break;
                    	 }
                     } else if (strcmp(rx_buffer, "n") == 0 || strcmp(rx_buffer, "N") == 0) {
                         myprintf("REGISTRO DESACTIVADO\r\n");
                         myprintf("Simulación iniciada. Escriba ':E' para finalizar sesión\r\n");
                         cambiar_modo(M_SIM);
                         input_state = INPUT_NORMAL;
                     } else {
                         uart_error("RESPUESTA INVÁLIDA. Use 'y' o 'n'");
                     }
                     break;

                 case INPUT_ID:
                     myprintf("ID ingresado: %s\r\n", rx_buffer);
                     // Validacion - ID debe ser un entero de 2 digitos
                     if (strlen(rx_buffer) == 2 && isdigit((unsigned char)rx_buffer[0]) && isdigit((unsigned char)rx_buffer[1])) {
                         snprintf(nombre_archivo, sizeof(nombre_archivo), "reg_%s.txt", rx_buffer);

                         // Revisar si el archivo ya existe
                         FIL fcheck;
                         if (f_open(&fcheck, nombre_archivo, FA_READ) == FR_OK) {
                             f_close(&fcheck);
                             uart_error("EL ARCHIVO YA EXISTE\r\n");
                             myprintf("Intente con otro ID:\r\n");
                             break; // Salir
                         }

                         // El archivo NO existe, intentar crearlo
                         if (f_open(&archivo_log, nombre_archivo, FA_CREATE_NEW | FA_WRITE) != FR_OK) {
                             uart_error("NO SE PUDO CREAR EL ARCHIVO. Vuelva a intentarlo");
                             myprintf("Saliendo...\r\n");
                             cambiar_modo(M_STDBY);
                             input_state = INPUT_NORMAL;
                             break;
                         }

                         UINT escritos;
                         char linea[128];

                         // 1. Línea que indica si PID está activado
                         snprintf(linea, sizeof(linea), "PID: %d\r\n", flagPID);
                         f_write(&archivo_log, linea, strlen(linea), &escritos);

                         // 2. Si PID activado, escribir ganancias
                         if (flagPID) {
                             snprintf(linea, sizeof(linea),
                                      "Yaw: Kp = %.2f, Ki = %.2f, Kd = %.2f; Pitch: Kp = %.2f, Ki = %.2f, Kd = %.2f\r\n",
                                      pid_yaw.kp, pid_yaw.ki, pid_yaw.kd,
                                      pid_pitch.kp, pid_pitch.ki, pid_pitch.kd);
                             f_write(&archivo_log, linea, strlen(linea), &escritos);
                         }

                         // 3. Escribir la cabecera de columnas
                         const char *cabecera = "time_stamp; yaw_sens; pitch_sens; yaw_servo; pitch_servo;\r\n";
                         f_write(&archivo_log, cabecera, strlen(cabecera), &escritos);

                         // 4. Sincronizar y continuar como antes
                         f_sync(&archivo_log);

                         myprintf("ARCHIVO %s CREADO CON EXITO\r\n", nombre_archivo);
                         myprintf("Simulación iniciada. Escriba ':E' para finalizar sesión\r\n");
                         time_stamp = 0;
                         flagReg = 1;
                         cambiar_modo(M_SIM);
                         input_state = INPUT_NORMAL;

                     } else {
                         uart_error("FORMATO INVÁLIDO. Ingrese un ID de 2 dígitos (ej: 01):");
                     }
                     break;

                 case INPUT_VER_REPRODUCIR:
                	 if (strcmp(rx_buffer, "v") == 0 || strcmp(rx_buffer, "V") == 0) {
                         myprintf("Mostrando registro %s...\r\n", registro_elegido);
                         ver_registro(registro_elegido);
                         myprintf("\r\nSaliendo...\r\n");
                         cambiar_modo(M_STDBY);
                         input_state = INPUT_NORMAL;
                         myprintf("\r\nVUELVA A INGRESAR ':M' PARA VER O REPRODUCIR OTRO REGISTRO\r\n");
                	 } else if (strcmp(rx_buffer, "r") == 0 || strcmp(rx_buffer, "R") == 0) {
                         myprintf("Reproduciendo registro %s...\r\n", registro_elegido);
                         reproducir_registro(registro_elegido);
                         input_state = INPUT_NORMAL;
                         flagRef = 0;
                	 } else {
                		 uart_error("RESPUESTA INVÁLIDA. Use 'v' o 'r'");
                	 }
                	 break;

				case INPUT_ELEGIR_REG:
					if (rx_buffer[0] == ':') {
						if (rx_buffer[1] == 'E') {
							cambiar_modo(M_STDBY);
							input_state = INPUT_NORMAL;
						} else {
							uart_error("COMANDO INVÁLIDO");
							myprintf("Intente nuevamente:\r\n");
							break;
						}
					} else {
						num_reg_elegido = atoi(rx_buffer);
						if (num_reg_elegido <= 0) {
							uart_error(
									"NÚMERO INVÁLIDO. Debe ser mayor o igual a 1.");
							myprintf(
									"Ingrese nuevamente el número del registro:\r\n");
							break;
						}

						{
							DIR dir;
							FILINFO fno;
							FRESULT fres;
							uint8_t contador = 0;

							fres = f_opendir(&dir, "/");
							if (fres != FR_OK) {
								uart_error("NO SE PUDO ABRIR EL DIRECTORIO");
								myprintf("Saliendo...");
								cambiar_modo(M_STDBY);
								input_state = INPUT_NORMAL;
								break;
							}

							while (1) {
								fres = f_readdir(&dir, &fno);
								if (fres != FR_OK || fno.fname[0] == 0)
									break; // Fin o error

								if (!(fno.fattrib & AM_DIR)) {
									contador++;
									if (contador == num_reg_elegido) {
										strncpy(registro_elegido, fno.fname,
												sizeof(registro_elegido));
										registro_elegido[sizeof(registro_elegido)
												- 1] = '\0'; // seguridad

										f_closedir(&dir);

										myprintf(
												"REGISTRO SELECCIONADO: %s\r\n\r\n",
												registro_elegido);

										myprintf(
												"¿Desea VER o REPRODUCIR el registro? (v/r)\r\n");
										input_state = INPUT_VER_REPRODUCIR;
									}
								}
							}

							if (contador < num_reg_elegido) {
								uart_error("NÚMERO FUERA DE RANGO");
								myprintf(
										"Ingrese un número válido (1 a %d):\r\n",
										contador);
							}

							f_closedir(&dir);
						}
					}

					break;

                 case INPUT_ERR_SD:
                	 if (strcmp(rx_buffer, "y") == 0 || strcmp(rx_buffer, "Y") == 0) {
                		 myprintf("\r\nIntentando recuperar SD...\r\n");
                         if (mount_sd()) {
                        	 myprintf("SD: OK\r\n");
                        	 myprintf("\r\nSaliendo...\r\n");
                        	 cambiar_modo(M_STDBY);
                        	 input_state = INPUT_NORMAL;
                        	 myprintf("\r\nVUELVA A INGRESAR ':M' PARA VER O REPRODUCIR REGISTROS\r\n");
                         }
                	 } else if (strcmp(rx_buffer, "n") == 0 || strcmp(rx_buffer, "N") == 0) {
                         myprintf("\r\nSaliendo...\r\n");
                         cambiar_modo(M_STDBY);
                         input_state = INPUT_NORMAL;
                	 } else {
                		 uart_error("RESPUESTA INVÁLIDA. Use 'y' o 'n'");
                	 }
                	 break;

                 case INPUT_ERR_MPU:
                	 if (strcmp(rx_buffer, "y") == 0 || strcmp(rx_buffer, "Y") == 0) {
                		 myprintf("\r\nIntentando recuperar sensor...\r\n");
                		 uint8_t res1 = verif_mpu();
                         if (res1) {
                        	 uint8_t res2 = mpu6050_read(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);
                        	 if (!res2) {
                            	 myprintf("Sensor: OK\r\n");
                            	 myprintf("\r\nSaliendo...\r\n");
                            	 cambiar_modo(M_STDBY);
                            	 input_state = INPUT_NORMAL;
                            	 if (flagCalCase) {
                            		 myprintf("\r\nVUELVA A INGRESAR ':C' PARA CALIBRAR\r\n");
                            		 flagCalCase = 0;
                            	 } else {
                            		 myprintf("\r\nVUELVA A INGRESAR ':S' PARA INICIAR SIMULACIÓN\r\n");
                            	 }
                        	 } else {
                        		 mpu6050_print_error(res2);
                        		 myprintf("Escriba 'y' para reintentar o 'n' para salir\r\n");
                        	 }
                         } else {
                        	 mpu6050_print_error(res1);
                        	 myprintf("Escriba 'y' para reintentar o 'n' para salir\r\n");
                         }
                	 } else if (strcmp(rx_buffer, "n") == 0 || strcmp(rx_buffer, "N") == 0) {
                         myprintf("\r\nSaliendo...\r\n");
                         cambiar_modo(M_STDBY);
                         input_state = INPUT_NORMAL;
                	 } else {
                		 uart_error("RESPUESTA INVÁLIDA. Use 'y' o 'n'");
                	 }
                	 break;

//                 case INPUT_CAL:
//                	 if (strcmp(rx_buffer, "y") == 0 || strcmp(rx_buffer, "Y") == 0) {
//              			myprintf("COMENZANDO CALIBRACIÓN...\r\n");
//              			calc_gyro_offsets();
//              			calibrar_mpu();
//              			flagCal = 1;		// indicador de calibración realizada
//              			myprintf("SENSOR CALIBRADO\r\n");
//              			myprintf("Offset X: %.2f°, Offset Y: %.2f°\r\n", ref_x, ref_y);
//              			input_state = INPUT_NORMAL;
//                	 } else if (strcmp(rx_buffer, "n") == 0 || strcmp(rx_buffer, "N") == 0) {
//                		myprintf("Cancelando...\r\n");
//                		myprintf("Use ':C' nuevamente para calibrar\r\n");
//                		input_state = INPUT_NORMAL;
//                	 } else {
//                		 uart_error("RESPUESTA INVÁLIDA. Use 'y' o 'n'");
//                	 }
//                	 break;
                 case INPUT_CAL:
                     if (strcmp(rx_buffer, "y") == 0 || strcmp(rx_buffer, "Y") == 0) {
                         myprintf("COMENZANDO CALIBRACIÓN...\r\n");

                         flagCalCase = 1;

                         // Verificar errores durante calibración de gyro
                         uint8_t res_gyro = calc_gyro_offsets();
                         if (res_gyro != MPU6050_OK) {
                             // El error ya fue manejado dentro de la función
                             break;
                         }

                         // Verificar errores durante calibración de filtro
                         uint8_t res_filtro = calibrar_mpu();
                         if (res_filtro != MPU6050_OK) {
                             // El error ya fue manejado dentro de la función
                             break;
                         }

                         // Solo si ambas calibraciones fueron exitosas
                         flagCalCase = 0;
                         flagCal = 1; // indicador de calibración realizada
                         myprintf("SENSOR CALIBRADO\r\n");
                         myprintf("Offset X: %.2f°, Offset Y: %.2f°\r\n", ref_x, ref_y);
                         input_state = INPUT_NORMAL;

                     } else if (strcmp(rx_buffer, "n") == 0 || strcmp(rx_buffer, "N") == 0) {
                         myprintf("Cancelando...\r\n");
                         myprintf("Use ':C' nuevamente para calibrar\r\n");
                         input_state = INPUT_NORMAL;
                     } else {
                         uart_error("RESPUESTA INVÁLIDA. Use 'y' o 'n'");
                     }
                     break;
    			 }
    		 }

             clear_buffer();
             if (input_state != INPUT_SERVO_WAIT_B1) {
                 show_prompt();
             }

    	} else if  (rx_char == '\b' || rx_char == 0x7F) {
    		if (rx_index > 0) {
    			rx_index--;
    		    myprintf("\b \b");	// backspace
    		}
    	} else {
    		if (rx_index < sizeof(rx_buffer) - 1) {
    			rx_buffer[rx_index++] = rx_char;
    		    HAL_UART_Transmit(&huart2, &rx_char, 1, 10);  // echo
    		}
    	}

    	HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    }
}

/* ============================================================================
 * CALLBACK INTERRUPCIONES X TIMER
 * ============================================================================ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIMER 3 --> VERIF. SERVOS (Manejo del tiempo para no usar delays)
	if (htim->Instance == TIM3) {
    	if (input_state == INPUT_SERVO_WAIT_B1) {
    		servo_sequence_task();
    	}
	}
	// TIMER 4 --> PARPADEO LED VERDE MODO SIM
    if (htim->Instance == TIM4) {
    	if (modo == M_SIM) {
    		HAL_GPIO_TogglePin(GPIOB, LED_VERDE_Pin);
    	}
    }
    // TIMER 5 --> LECTURA MPU, MOVIMIENTO DE SERVOS, (REGISTRO), REPRODUCCION DE REGISTROS
    else if (htim->Instance == TIM5) {
    	if (modo == M_SIM) {

    		uint16_t yy;
			uint16_t pp;

    		if (!flagPID) {
    			static float dt = 0.05f; // Tiempo de muestreo en segundos (interrupciones cada 50 ms)

        		// Lectura del acelerometro y giroscopo del MPU, calculo de ángulos, filtrado
        		lectura_filtrado_mpu_calibrada(dt);

    			// Calcular valor PWM a partir del angulo filtrado
    			yy = filt_to_pwm(ang_filtrado_cal_y, 0);
    			pp = filt_to_pwm(ang_filtrado_cal_x, 1);

        	    // Modificar PWM
        		modificar_pwm(&htim2, TIM_CHANNEL_1, yy);
        	    modificar_pwm(&htim2, TIM_CHANNEL_2, pp);
    		} else {
				static float dt = 0.05f; // Tiempo de muestreo en segundos (interrupciones cada 50 ms)

				// Lectura del acelerometro y giroscopo del MPU, calculo de ángulos, filtrado
				lectura_filtrado_mpu_calibrada(dt);

				// Calcular valor PWM usando PID
				yy = filt_to_pwm_pid(ang_filtrado_cal_y, 0, dt);
				pp = filt_to_pwm_pid(ang_filtrado_cal_x, 1, dt);

				// Modificar PWM
				modificar_pwm(&htim2, TIM_CHANNEL_1, yy);
				modificar_pwm(&htim2, TIM_CHANNEL_2, pp);
    		}
    	    // Si se activó el modo registro ----------------------------------
    		if (flagReg == 1) {
                // Escribir en archivo
                char linea[64];
                snprintf(linea, sizeof(linea), "%lu; %f; %f; %u; %u;\r\n",
                         time_stamp, ang_yaw_sat, ang_pitch_sat, yy, pp);
                time_stamp += 50;

                FRESULT fres;
                UINT escritos;

                fres = f_write(&archivo_log, linea, strlen(linea), &escritos);
                if (fres != FR_OK || escritos != strlen(linea)) {
                    uart_error("\r\nFALLÓ LA ESCRITURA EN LA SD");
                    flagReg = 0;
                    f_close(&archivo_log);
                    myprintf("*ALRT: SE DETUVO EL REGISTRO\r\n");
                    myprintf("Puede continuar sin registro. Escriba :E para finalizar simulación\r\n");
                    show_prompt();
                    return;
                }

                fres = f_sync(&archivo_log);
                if (fres != FR_OK) {
                    uart_error("\r\nFALLÓ LA SINCRONIZACIÓN CON LA SD");
                    flagReg = 0;
                    f_close(&archivo_log);
                    myprintf("*ALRT: SE DETUVO EL REGISTRO\r\n");
                    myprintf("Puede continuar sin registro. Escriba :E para finalizar simulación\r\n");
                    show_prompt();
                    return;
                }

    		}
    	}
        if (modo == M_MEM && flagRepro) {

//            char* res = f_gets(linea_repro, sizeof(linea_repro), &archivo_repro);

//            if (res == NULL) {
//                f_close(&archivo_repro);
//                // todo: agregar ================
//                myprintf("\r\nFIN DE LA REPRODUCCIÓN\r\n");
//                myprintf("Total de líneas procesadas: %lu\r\n\r\n", contador_repro);
//                flagRepro = 0;
//                myprintf("Saliendo...\r\n");
//                cambiar_modo(M_STDBY);
//                myprintf("\r\nVUELVA A INGRESAR ':M' PARA VER O REPRODUCIR OTRO REGISTRO\r\n");
//                show_prompt();
//                return;
//            }

            char* res = f_gets(linea_repro, sizeof(linea_repro), &archivo_repro);

            if (res == NULL) {
                if (f_error(&archivo_repro)) {
                    uart_error("\r\nFALLA AL LEER EL ARCHIVO (¿SD DESCONECTADA?)");
                    f_close(&archivo_repro);
                    flagRepro = 0;
                    cambiar_modo(M_ERR);
                    input_state = INPUT_ERR_SD;
                    myprintf("*ALRT: REVISAR LA CONEXIÓN DE LA SD\r\n");
                    myprintf("Escriba 'y' para intentar recuperar SD o 'n' para salir\r\n");
                    show_prompt();
                    return;
                }

                // Si no hay error, es EOF
                f_close(&archivo_repro);
                myprintf("\r\nFIN DE LA REPRODUCCIÓN\r\n");
                myprintf("Total de líneas procesadas: %lu\r\n\r\n", contador_repro);
                flagRepro = 0;
                myprintf("Saliendo...\r\n");
                cambiar_modo(M_STDBY);
                myprintf("\r\nVUELVA A INGRESAR ':M' PARA VER O REPRODUCIR OTRO REGISTRO\r\n");
                show_prompt();
                return;
            }

            // Saltar líneas vacías
            if (linea_repro[0] == '\0' || linea_repro[0] == '\r' || linea_repro[0] == '\n') {
                return;
            }

            // Parsear línea
            char* token = strtok(linea_repro, ";");
            int i = 0;
            uint16_t yaw_servo = 0, pitch_servo = 0;

            while (token && i < 5) {
                while (*token == ' ') token++; // eliminar espacios al inicio

                switch (i) {
                    case 0: break;	// t
                    case 1: break;	// yaw_sens
                    case 2: break;	// pitch_sens
                    case 3: yaw_servo = (uint16_t)atoi(token); break;
                    case 4: pitch_servo = (uint16_t)atoi(token); break;
                }

                i++;
                token = strtok(NULL, ";");
            }

            if (i == 5) {
                modificar_pwm(&htim2, TIM_CHANNEL_1, yaw_servo);
                modificar_pwm(&htim2, TIM_CHANNEL_2, pitch_servo);
                contador_repro++;
            }
        }
    }
}

/* ============================================================================
 * CALLBACK INTERRUPCIÓN BUTTON
 * ============================================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Button_Pin) {
        if (servo_actual == 1) {
            myprintf("Moviendo SERVO 1 (YAW)...\r\n");
        } else {
            myprintf("Moviendo SERVO 2 (PITCH)...\r\n");
            servo_actual = 2;
        }

        // Iniciar secuencia
        servo_sequence_active = 1;
        servo_step = 0;
        servo_timer = 0;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
