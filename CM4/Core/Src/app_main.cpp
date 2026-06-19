#include "app_main.h"
#include "main.h"
#include "cmsis_os2.h"
#include "Fbus.hpp"

extern UART_HandleTypeDef huart8; 

FBus receiver(&huart8);

void app_main(void)
{
  for(;;)
  {
    osDelay(1);
  }
}

void fbus_loop(void)
{
  for(;;)
  {
    if (receiver.init()) {
        // Erfolgreich gestartet
    }
    while (1) {
        // Beispiel: Sende alle 100ms eine fiktive Drehzahl (Sensor ID 0x0500 für RPM)
        static uint32_t lastTick = 0;
        if (HAL_GetTick() - lastTick > 100) {
            lastTick = HAL_GetTick();
            uint32_t motor_rpm = 3500; 
            receiver.queueTelemetry(0x0500, motor_rpm);
        }

        // Empfangene Kanäle auslesen (z.B. CH1 für Lenkung/Gas)
        uint16_t rc_throttle = receiver.getChannel(0); 
        
        // HIER: Übergabe der Daten an den CM7-Kern via Inter-Processor Communication (IPC)
        // z.B. über HSEM (Hardware Semaphores) oder Shared RAM (SRAM4)
    }
    osDelay(1);
  }
}

extern "C" void FBus_HandleByte(uint8_t byte) {
    receiver.handleRxByte(byte);
}
