:orphan:

.. _migration_3.7:

Migration guide to Zephyr v3.7.0 (Working Draft)
################################################

This document describes the changes required when migrating your application from Zephyr v3.6.0 to
Zephyr v3.7.0.

Any other changes (not directly related to migrating applications) can be found in
the :ref:`release notes<zephyr_3.7>`.

.. contents::
    :local:
    :depth: 2

Build System
************

* Completely overhauled the way SoCs and boards are defined. This requires all
  out-of-tree SoCs and boards to be ported to the new model. See the
  :ref:`hw_model_v2` for more detailed information.

Kernel
******

Boards
******

Modules
*******

MCUboot
=======

zcbor
=====

Device Drivers and Devicetree
*****************************

* The :dtcompatible:`nxp,kinetis-pit` pit driver has changed it's compatible
  to :dtcompatible:`nxp,pit` and has been updated to support multiple channels.
  To configure the individual channels, you must add a child node with the
  compatible :dtcompatible:`nxp,pit-channel` and configure as below.
  The :kconfig:option:`CONFIG_COUNTER_MCUX_PIT` has also been renamed to
  :kconfig:option:`CONFIG_COUNTER_NXP_PIT` with regards to the renaming
  of the binding for the pit.
  example:

  .. code-block:: devicetree

    / {
        pit0: pit@40037000 {
            /* Other Pit DT Attributes */
            compatible = "nxp,pit";
            status = "disabled";
            num-channels = <1>;
            #address-cells = <1>;
            #size-cells = <0>;

            pit0_channel0: pit0_channel@0 {
                compatible = "nxp,pit-channel";
                reg = <0>;
                status = "disabled";
            };
    };


Analog-to-Digital Converter (ADC)
=================================

Bluetooth HCI
=============

Charger
=======

* Dropped ``constant-charge-current-max-microamp`` property in ``charger_max20335`` driver because
  it did not reflect real chip functionality.

* Added enum key to ``constant-charge-voltage-max-microvolt`` property in ``maxim,max20335-charger``
  binding to indicate invalid devicetree values at build time.

Controller Area Network (CAN)
=============================

* Removed the following deprecated CAN controller devicetree properties. Out-of-tree boards using
  these properties need to switch to using the ``bus-speed``, ``sample-point``, ``bus-speed-data``,
  and ``sample-point-data`` devicetree properties for specifying the initial CAN bitrate:

  * ``sjw``
  * ``prop-seg``
  * ``phase-seg1``
  * ``phase-seg1``
  * ``sjw-data``
  * ``prop-seg-data``
  * ``phase-seg1-data``
  * ``phase-seg1-data``

* Support for manual bus-off recovery was reworked:

  * Automatic bus recovery will always be enabled upon driver initialization regardless of Kconfig
    options. Since CAN controllers are initialized in "stopped" state, no unwanted bus-off recovery
    will be started at this point.
  * The Kconfig ``CONFIG_CAN_AUTO_BUS_OFF_RECOVERY`` was renamed (and inverted) to
    :kconfig:option:`CONFIG_CAN_MANUAL_RECOVERY_MODE`, which is disabled by default. This Kconfig
    option enables support for the :c:func:`can_recover()` API function and a new manual recovery mode
    (see the next bullet).
  * A new CAN controller operational mode :c:macro:`CAN_MODE_MANUAL_RECOVERY` was added. Support for
    this is only enabled if :kconfig:option:`CONFIG_CAN_MANUAL_RECOVERY_MODE` is enabled. Having
    this as a mode allows applications to inquire whether the CAN controller supports manual
    recovery mode via the :c:func:`can_get_capabilities` API function. The application can then
    either fail initialization or rely on automatic bus-off recovery. Having this as a mode
    furthermore allows CAN controller drivers not supporting manual recovery mode to fail early in
    :c:func:`can_set_mode` during application startup instead of failing when :c:func:`can_recover`
    is called at a later point in time.

Display
=======

Flash
=====

General Purpose I/O (GPIO)
==========================

Input
=====

* The ``analog-axis`` deadzone calibration value has been changed to be
  relative to the raw ADC values, similarly to min and max. The data structures
  and properties have been renamed to reflect that (from ``out-deadzone`` to
  ``in-deadzone``) and when migrating to the new definition the value should be
  scaled accordingly.

Interrupt Controller
====================

LED Strip
=========

* The property ``in-gpios`` defined in :dtcompatible:`worldsemi,ws2812-gpio` has been
  renamed to ``gpios``.

Sensors
=======

Serial
======

Timer
=====

Bluetooth
*********

Bluetooth Mesh
==============

* The model metadata pointer declaration of :c:struct:`bt_mesh_model` has been changed
  to add ``const`` qualifiers. The data pointer of :c:struct:`bt_mesh_models_metadata_entry`
  got ``const`` qualifier too. The model's metadata structure and metadata raw value
  can be declared as permanent constants in the non-volatile memory. (:github:`69679`)

Bluetooth Audio
===============

Bluetooth Classic
=================

* The source files of Host BR/EDR have been moved to ``subsys/bluetooth/host/classic``.
  The Header files of Host BR/EDR have been moved to ``include/zephyr/bluetooth/classic``.
  Removed the :kconfig:option:`CONFIG_BT_BREDR`. It is replaced by new option
  :kconfig:option:`CONFIG_BT_CLASSIC`. (:github:`69651`)

Networking
**********

* The zperf zperf_results struct is changed to support 64 bits transferred bytes (total_len)
  and test duration (time_in_us and client_time_in_us), instead of 32 bits. This will make
  the long-duration zperf test show with correct throughput result.
  (:github:`69500`)

* Each IPv4 address assigned to a network interface has an IPv4 netmask
  tied to it instead of being set for the whole interface.
  If there is only one IPv4 address specified for a network interface,
  nothing changes from the user point of view. But, if there is more than
  one IPv4 address / network interface, the netmask must be specified
  for each IPv4 address separately. (:github:`68419`)

* Virtual network interface API no longer has the `input` callback. The input callback was
  used to read the inner IPv4/IPv6 packets in an IP tunnel. This incoming tunnel read is now
  implemented in `recv` callback. (:github:`70549`)

* Modified the ``wifi connect`` command to use key-value format for the arguments. In the
  previous implementation, we were identifying an option using its position in the argument string.
  This made it difficult to deal with optional arguments or extending the support
  for other options. Having this key-value format makes it easier to extend the options that
  can be passed to the connect command.
  ``wifi -h`` will give more information about the usage of connect command.
  (:github:`70024`)

Other Subsystems
****************

LoRaWAN
=======

MCUmgr
======

Modem
=====

* The ``CONFIG_MODEM_CHAT_LOG_BUFFER`` Kconfig option was
  renamed to :kconfig:option:`MODEM_CHAT_LOG_BUFFER_SIZE`.

Shell
=====

ZBus
====

Userspace
*********

Architectures
*************

* Function :c:func:`arch_start_cpu` has been renamed to :c:func:`arch_cpu_start`.

* x86

  * Kconfigs ``CONFIG_DISABLE_SSBD`` and ``CONFIG_ENABLE_EXTENDED_IBRS``
    are deprecated. Use :kconfig:option:`CONFIG_X86_DISABLE_SSBD` and
    :kconfig:option:`CONFIG_X86_ENABLE_EXTENDED_IBRS` instead.

Xtensa
======
