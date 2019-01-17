This program simply flashes the LED at every 500 ms count and 1000ms timeout. i.e Pulse at 500 and Auto-reload of 1000

Important thing to remember while using Timers:
Always check the frequency is right in STM32CubeMX because usually timers frequency is 2x times the bus maximum frequency.

Internal Clock means the clock to the Timers (do not confuse with internal HSI clock).
