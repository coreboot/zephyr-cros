:orphan:

.. _migration_4.0:

Migration guide to Zephyr v4.0.0 (Working Draft)
################################################

This document describes the changes required when migrating your application from Zephyr v3.7.0 to
Zephyr v4.0.0.

Any other changes (not directly related to migrating applications) can be found in
the :ref:`release notes<zephyr_4.0>`.

.. contents::
    :local:
    :depth: 2

Build System
************

Kernel
******

Boards
******

Modules
*******

Mbed TLS
========

Trusted Firmware-M
==================

LVGL
====

Device Drivers and Devicetree
*****************************

* The ``compatible`` of the LiteX ethernet controller has been renamed from
  ``litex,eth0`` to :dtcompatible:`litex,liteeth`. (:github:`75433`)

* The ``compatible`` of the LiteX uart controller has been renamed from
  ``litex,uart0`` to :dtcompatible:`litex,uart`. (:github:`74522`)

Controller Area Network (CAN)
=============================

Display
=======

Enhanced Serial Peripheral Interface (eSPI)
===========================================

GNSS
====

Input
=====

* :c:macro:`INPUT_CALLBACK_DEFINE` has now an extra ``user_data`` void pointer
  argument that can be used to reference any user data structure. To restore
  the current behavior it can be set to ``NULL``. A ``void *user_data``
  argument has to be added to the callback function arguments.

* The :dtcompatible:`analog-axis` ``invert`` property has been renamed to
  ``invert-input`` (there's now an ``invert-output`` available as well).

Interrupt Controller
====================

LED Strip
=========

Sensors
=======

Serial
======

Regulator
=========

* Internal regulators present in nRF52/53 series can now be configured using
  devicetree. The Kconfig options :kconfig:option:`CONFIG_SOC_DCDC_NRF52X`,
  :kconfig:option:`CONFIG_SOC_DCDC_NRF52X_HV`,
  :kconfig:option:`CONFIG_SOC_DCDC_NRF53X_APP`,
  :kconfig:option:`CONFIG_SOC_DCDC_NRF53X_NET` and
  :kconfig:option:`CONFIG_SOC_DCDC_NRF53X_HV` selected by board-level Kconfig
  options have been deprecated.

  Example for nRF52 series:

  .. code-block:: devicetree

      /* configure REG/REG1 in DC/DC mode */
      &reg/reg1 {
          regulator-initial-mode = <NRF5X_REG_MODE_DCDC>;
      };

      /* enable REG0 (HV mode) */
      &reg0 {
          status = "okay";
      };

  Example for nRF53 series:

  .. code-block:: devicetree

      /* configure VREGMAIN in DC/DC mode */
      &vregmain {
          regulator-initial-mode = <NRF5X_REG_MODE_DCDC>;
      };

      /* configure VREGRADIO in DC/DC mode */
      &vregradio {
          regulator-initial-mode = <NRF5X_REG_MODE_DCDC>;
      };

      /* enable VREGH (HV mode) */
      &vregh {
          status = "okay";
      };

Bluetooth
*********

Bluetooth HCI
=============

Bluetooth Mesh
==============

Bluetooth Audio
===============

Bluetooth Classic
=================

Bluetooth Host
==============

Bluetooth Crypto
================

Networking
**********

* The CoAP public API functions :c:func:`coap_get_block1_option` and
  :c:func:`coap_get_block2_option` have changed. The ``block_number`` pointer
  type has changed from ``uint8_t *`` to ``uint32_t *``. Additionally,
  :c:func:`coap_get_block2_option` now accepts an additional ``bool *has_more``
  parameter, to store the value of the more flag. (:github:`76052`)

Other Subsystems
****************

Flash map
=========

hawkBit
=======

MCUmgr
======

Modem
=====

Architectures
*************
