.. _mimxrt1160_evk:

NXP MIMXRT1160-EVK
##################

Overview
********

The dual core i.MX RT1160 runs on the Cortex-M7 core at 600 MHz and on the
Cortex-M4 at 240 MHz. The i.MX RT1160 MCU offers support over a wide
temperature range and is qualified for consumer, industrial and automotive
markets.

.. image:: ./mimxrt1160_evk.jpg
   :width: 600px
   :align: center
   :alt: MIMXRT1160-EVK

Hardware
********

- MIMXRT1166DVM6A MCU

  - 600MHz Cortex-M7 & 240Mhz Cortex-M4
  - 2MB SRAM with 512KB of TCM for Cortex-M7 and 256KB of TCM for Cortex-M4

- Memory

  - 512 Mbit SDRAM
  - 128 Mbit QSPI Flash
  - 512 Mbit Octal Flash
  - TF socket for SD card

- Display

  - MIPI LCD connector

- Ethernet

  - 10/100 Mbit/s Ethernet PHY
  - 10/100/1000 Mbit/s Ethernet PHY

- USB

  - USB 2.0 OTG connector
  - USB 2.0 host connector

- Audio

  - 3.5 mm audio stereo headphone jack
  - Board-mounted microphone
  - Left and right speaker out connectors

- Power

  - 5 V DC jack

- Debug

  - JTAG 20-pin connector
  - OpenSDA with DAPLink

- Sensor

  - MIPI camera sensor connector

- Expansion port

  - Arduino interface

- CAN bus connector

For more information about the MIMXRT1160 SoC and MIMXRT1160-EVK board, see
these references:

- `i.MX RT1160 Website`_
- `i.MX RT1160 Datasheet`_
- `i.MX RT1160 Reference Manual`_
- `MIMXRT1160-EVK Website`_
- `MIMXRT1160-EVK Board Hardware User's Guide`_

Supported Features
==================

The mimxrt1160_evk board configuration supports the following hardware
features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| SYSTICK   | on-chip    | systick                             |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| COUNTER   | on-chip    | counter                             |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+
| SPI       | on-chip    | spi                                 |
+-----------+------------+-------------------------------------+
| I2C       | on-chip    | i2c                                 |
+-----------+------------+-------------------------------------+
| ADC       | on-chip    | adc                                 |
+-----------+------------+-------------------------------------+
| CAN       | on-chip    | flexcan                             |
+-----------+------------+-------------------------------------+
| WATCHDOG  | on-chip    | watchdog                            |
+-----------+------------+-------------------------------------+
| PWM       | on-chip    | pwm                                 |
+-----------+------------+-------------------------------------+
| DMA       | on-chip    | dma                                 |
+-----------+------------+-------------------------------------+
| GPT       | on-chip    | gpt                                 |
+-----------+------------+-------------------------------------+

The default configuration can be found in the defconfig file:
``boards/arm/mimxrt1160_evk/mimxrt1160_evk_cm7_defconfig``

Other hardware features are not currently supported by the port.


Connections and I/Os
====================

The MIMXRT1160 SoC has six pairs of pinmux/gpio controllers.

+---------------+-----------------+---------------------------+
| Name          | Function        | Usage                     |
+===============+=================+===========================+
| WAKEUP        | GPIO            | SW7                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_04    | GPIO            | LED                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_24    | LPUART1_TX      | UART Console              |
+---------------+-----------------+---------------------------+
| GPIO_AD_25    | LPUART1_RX      | UART Console              |
+---------------+-----------------+---------------------------+
| GPIO_LPSR_00  | CAN3_TX         | flexcan                   |
+---------------+-----------------+---------------------------+
| GPIO_LPSR_01  | CAN3_RX         | flexcan                   |
+---------------+-----------------+---------------------------+
| GPIO_AD_29    | SPI1_CS0        | spi                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_28    | SPI1_CLK        | spi                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_30    | SPI1_SDO        | spi                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_31    | SPI1_SDI        | spi                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_08    | LPI2C1_SCL      | i2c                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_09    | LPI2C1_SDA      | i2c                       |
+---------------+-----------------+---------------------------+
| GPIO_LPSR_05  | LPI2C5_SCL      | i2c                       |
+---------------+-----------------+---------------------------+
| GPIO_LPSR_04  | LPI2C5_SDA      | i2c                       |
+---------------+-----------------+---------------------------+
| GPIO_AD_04    | FLEXPWM1_PWM2   | pwm                       |
+---------------+-----------------+---------------------------+

System Clock
============

The MIMXRT1160 SoC is configured to use the 32 KHz low frequency oscillator on
the board as a source for the GPT timer to generate a system clock.

Serial Port
===========

The MIMXRT1160 SoC has 12 UARTs. One is configured for the console and the
remaining are not used.

Programming and Debugging
*************************

Build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

Configuring a Debug Probe
=========================

A debug probe is used for both flashing and debugging the board. This board is
configured by default to use the :ref:`opensda-daplink-onboard-debug-probe`,
however the :ref:`pyocd-debug-host-tools` do not yet support programming the
external flashes on this board so you must reconfigure the board for one of the
following debug probes instead.

:ref:`jlink-external-debug-probe`
---------------------------------

Install the :ref:`jlink-debug-host-tools` and make sure they are in your search
path.

Attach a J-Link 20-pin connector to J1. Check that jumpers J6 and J7
are **off** (they are on by default when boards ship from the factory) to
ensure SWD signals are disconnected from the OpenSDA microcontroller.

Configuring a Console
=====================

Regardless of your choice in debug probe, we will use the OpenSDA
microcontroller as a usb-to-serial adapter for the serial console. Check that
jumpers J5 and J8 are **on** (they are on by default when boards ship from
the factory) to connect UART signals to the OpenSDA microcontroller.

Connect a USB cable from your PC to J11.

Use the following settings with your serial terminal of choice (minicom, putty,
etc.):

- Speed: 115200
- Data: 8 bits
- Parity: None
- Stop bits: 1

Flashing
========

Here is an example for the :ref:`hello_world` application.

Before power on the board, make sure SW1 is set to 0001b

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: mimxrt1160_evk_cm7
   :goals: flash

Power off the board, and change SW1 to 0010b. Then power on the board and
open a serial terminal, reset the board (press the SW4 button), and you should
see the following message in the terminal:

.. code-block:: console

   ***** Booting Zephyr OS v2.6.0-xxxx-xxxxxxxxxxxxx *****
   Hello World! mimxrt1160_evk_cm7

Debugging
=========

Here is an example for the :ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: mimxrt1160_evk_cm7
   :goals: debug

Open a serial terminal, step through the application in your debugger, and you
should see the following message in the terminal:

.. code-block:: console

   ***** Booting Zephyr OS v2.4.0-xxxx-xxxxxxxxxxxxx *****
   Hello World! mimxrt1160_evk_cm7

.. _MIMXRT1160-EVK Website:
   https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt1160-evaluation-kit:MIMXRT1160-EVK

.. _MIMXRT1160-EVK Board Hardware User's Guide:
   https://www.nxp.com/docs/en/user-guide/UM11617.pdf

.. _i.MX RT1160 Website:
   https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/i-mx-rt-crossover-mcus/i-mx-rt1160-crossover-mcu-family-high-performance-mcu-with-arm-cortex-m7-and-cortex-m4-cores:i.MX-RT1160

.. _i.MX RT1160 Datasheet:
   https://www.nxp.com/docs/en/data-sheet/IMXRT1160CEC.pdf

.. _i.MX RT1160 Reference Manual:
   https://www.nxp.com/webapp/Download?colCode=IMXRT1160RM
