:orphan:

.. _migration_3.5:

Migration guide to Zephyr v3.5.0 (Working Draft)
################################################

This document describes the changes required or recommended when migrating your
application from Zephyr v3.4.0 to Zephyr v3.5.0.

Required changes
****************

* The kernel :c:func:`k_mem_slab_free` function has changed its signature, now
  taking a ``void *mem`` pointer instead of a ``void **mem`` double-pointer.
  The new signature will not immediately trigger a compiler error or warning,
  instead likely causing a invalid memory access at runtime. A new ``_ASSERT``
  statement, that you can enable with :kconfig:option:`CONFIG_ASSERT`, will
  detect if you pass the function memory not belonging to the memory blocks in
  the slab.

* The :kconfig:option:`CONFIG_BOOTLOADER_SRAM_SIZE` default value is now ``0`` (was
  ``16``). Bootloaders that use a part of the SRAM should set this value to an
  appropriate size. :github:`60371`

* The Kconfig option ``CONFIG_GPIO_NCT38XX_INTERRUPT`` has been renamed to
  :kconfig:option:`CONFIG_GPIO_NCT38XX_ALERT`.

* MCUmgr SMP version 2 error codes entry has changed due to a collision with an
  existing response in shell_mgmt. Previously, these errors had the entry ``ret``
  but now have the entry ``err``. ``smp_add_cmd_ret()`` is now deprecated and
  :c:func:`smp_add_cmd_err` should be used instead, ``MGMT_CB_ERROR_RET`` is
  now deprecated and :c:enumerator:`MGMT_CB_ERROR_ERR` should be used instead.
  SMP version 2 error code defines for in-tree modules have been updated to
  replace the ``*_RET_RC_*`` parts with ``*_ERR_*``.

* MCUmgr SMP version 2 error translation (to legacy MCUmgr error code) is now
  handled in function handlers by setting the ``mg_translate_error`` function
  pointer of :c:struct:`mgmt_group` when registering a group. See
  :c:type:`smp_translate_error_fn` for function details. Any SMP version 2
  handlers made for Zephyr 3.4 need to be updated to include these translation
  functions when the groups are registered.

* ``zephyr,memory-region-mpu`` was renamed ``zephyr,memory-attr`` and its type
  moved from 'enum' to 'int'. To have a seamless conversion this is the
  required change in the DT:

  .. code-block:: none

     - "RAM"         -> <( DT_MEM_ARM(ATTR_MPU_RAM) )>
     - "RAM_NOCACHE" -> <( DT_MEM_ARM(ATTR_MPU_RAM_NOCACHE) )>
     - "FLASH"       -> <( DT_MEM_ARM(ATTR_MPU_FLASH) )>
     - "PPB"         -> <( DT_MEM_ARM(ATTR_MPU_PPB) )>
     - "IO"          -> <( DT_MEM_ARM(ATTR_MPU_IO) )>
     - "EXTMEM"      -> <( DT_MEM_ARM(ATTR_MPU_EXTMEM) )>

* A new networking Kconfig option :kconfig:option:`CONFIG_NET_INTERFACE_NAME`
  defaults to ``y``. The option allows user to set a name to a network interface.
  During system startup a default name is assigned to the network interface like
  ``eth0`` to the first Ethernet network interface. The option affects the behavior
  of ``SO_BINDTODEVICE`` BSD socket option. If the Kconfig option is set to ``n``,
  which is how the system worked earlier, then the name of the device assigned
  to the network interface is used by the ``SO_BINDTODEVICE`` socket option.
  If the Kconfig option is set to ``y`` (current default), then the network
  interface name is used by the ``SO_BINDTODEVICE`` socket option.

* On all STM32 ADC, it is no longer possible to read sensor channels (Vref,
  Vbat or temperature) using the ADC driver. The dedicated sensor driver should
  be used instead. This change is due to a limitation on STM32F4 where the
  channels for temperature and Vbat are identical, and the impossibility of
  determining what we want to measure using solely the ADC API.

* The default C library used on most targets has changed from the built-in
  minimal C library to Picolibc. While both provide standard C library
  interfaces and shouldn't cause any behavioral regressions for applications,
  there are a few side effects to be aware of when migrating to Picolibc.

  * Picolibc enables thread local storage
    (:kconfig:option:`CONFIG_THREAD_LOCAL_STORAGE`) where supported. This
    changes some internal operations within the kernel that improve
    performance using some TLS variables. Zephyr places TLS variables in the
    memory reserved for the stack, so stack usage for every thread will
    increase by 8-16 bytes.

  * Picolibc uses the same malloc implementation as the minimal C library, but
    the default heap size depends on which C library is in use. When using the
    minimal C library, the default heap is zero bytes, which means that malloc
    will always fail. When using Picolibc, the default is 16kB with
    :kconfig:option:`CONFIG_MMU` or :kconfig:option:`ARCH_POSIX`, 2kB with
    :kconfig:option:`CONFIG_USERSPACE` and
    :kconfig:option:`CONFIG_MPU_REQUIRES_POWER_OF_TWO_ALIGNMENT`. For all
    other targets, the default heap uses all remaining memory on the system.
    You can change this by adjusting
    :kconfig:option:`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`.

  * Picolibc can either be built as part of the OS build or pulled from the
    toolchain. When building as part of the OS, the build will increase by
    approximately 1000 files.

  * When using the standard C++ library with Picolibc, both of those must come
    from the toolchain as the standard C++ library depends upon the C library
    ABI.

  * Picolibc removes the ``-ffreestanding`` compiler option. This allows
    significant compiler optimization improvements, but also means that the
    compiler will now warn about declarations of `main` which don't conform to
    the Zephyr required type -- ``int main(void)``.

  * Picolibc's default floating point input/output code is larger than the
    minimal C library version (this is necessary to conform with the C
    language "round trip" requirements for these operations). If you use
    :kconfig:option:`CONFIG_CBPRINTF_FP_SUPPORT`, you will see increased
    memory usage unless you also disable
    :kconfig:option:`CONFIG_PICOLIBC_IO_FLOAT_EXACT`, which switches Picolibc
    to a smaller, but inexact conversion algorithm. This requires building
    Picolibc as a module.

* The CAN controller timing API functions :c:func:`can_set_timing` and :c:func:`can_set_timing_data`
  no longer fallback to the (Re-)Synchronization Jump Width (SJW) value set in the devicetree
  properties for the given CAN controller upon encountering an SJW value corresponding to
  ``CAN_SJW_NO_CHANGE`` (which is no longer available). The caller will therefore need to fill in
  the ``sjw`` field in :c:struct:`can_timing`. To aid in this, the :c:func:`can_calc_timing` and
  :c:func:`can_calc_timing_data` functions now automatically calculate a suitable SJW. The
  calculated SJW can be overwritten by the caller if needed. The CAN controller API functions
  :c:func:`can_set_bitrate` and :c:func:`can_set_bitrate_data` now also automatically calculate a
  suitable SJW, but their SJW cannot be overwritten by the caller.

* Ethernet PHY devicetree bindings were updated to use the standard ``reg``
  property for the PHY address instead of a custom ``address`` property. As a
  result, MDIO controller nodes now require ``#address-cells`` and
  ``#size-cells`` properties. Similarly, Ethernet PHY devicetree nodes and
  corresponding driver were updated to consistently use the node name
  ``ethernet-phy`` instead of ``phy``. Devicetrees and overlays must be updated
  accordingly:

  .. code-block:: devicetree

     mdio {
         compatible = "mdio-controller";
         #address-cells = <1>;
         #size-cells = <0>;

         ethernet-phy@0 {
             compatible = "ethernet-phy";
             reg = <0>;
         };
     };

* The ``accept()`` callback's signature in :c:struct:`bt_l2cap_server` has
  changed to ``int (*accept)(struct bt_conn *conn, struct bt_l2cap_server
  *server, struct bt_l2cap_chan **chan)``,
  adding a new ``server`` parameter pointing to the :c:struct:`bt_l2cap_server`
  structure instance the callback relates to. :github:`60536`

* The RAM disk driver has been changed to support multiple instances and instantiation
  using devicetree. As a result, Kconfig option :kconfig:option:`CONFIG_DISK_RAM_VOLUME_SIZE`
  and Kconfig option :kconfig:option:`CONFIG_DISK_RAM_VOLUME_NAME` are removed,
  and the application using the RAM disk must instantiate it using devicetree,
  as in the following example:

  .. code-block:: devicetree

    / {
        ramdisk0 {
            compatible = "zephyr,ram-disk";
            disk-name = "RAM";
            sector-size = <512>;
            sector-count = <192>;
        };
    };

* The :dtcompatible:`goodix,gt911`, :dtcompatible:`xptek,xpt2046` and
  :dtcompatible:`hynitron,cst816s` drivers have been converted from Kscan to
  Input, they can still be used with Kscan applications by adding a
  :dtcompatible:`zephyr,kscan-input` node.

* The ``zephyr,gpio-keys`` binding has been merged into
  :dtcompatible:`gpio-keys` and the callback definition has been renamed from
  ``INPUT_LISTENER_CB_DEFINE`` to :c:macro:`INPUT_CALLBACK_DEFINE`.

* :c:macro:`CONTAINER_OF` now performs type checking, this was very commonly
  misused to obtain user structure from :c:struct:`k_work` pointers without
  passing from :c:struct:`k_work_delayable`. This would now result in a build
  error and have to be done correctly using
  :c:func:`k_work_delayable_from_work`.

* The :dtcompatible:`ti,bq274xx` driver was using incorrect units for capacity
  and power channels, these have been fixed and scaled by x1000 factor from the
  previous implementation, any application using them has to be changed
  accordingly.

Recommended Changes
*******************

* Setting the GIC architecture version by selecting
  :kconfig:option:`CONFIG_GIC_V1`, :kconfig:option:`CONFIG_GIC_V2` and
  :kconfig:option:`CONFIG_GIC_V3` directly in Kconfig has been deprecated.
  The GIC version should now be specified by adding the appropriate compatible, for
  example :dtcompatible:`arm,gic-v2`, to the GIC node in the device tree.

* Nordic nRF based boards using :kconfig:option:`CONFIG_NFCT_PINS_AS_GPIOS`
  to configure NFCT pins as GPIOs, should instead set the new UICR
  ``nfct-pins-as-gpios`` property in devicetree. It can be set like this in the
  board devicetree files:

  .. code-block:: devicetree

     &uicr {
         nfct-pins-as-gpios;
     };

* Nordic nRF based boards using :kconfig:option:`CONFIG_GPIO_AS_PINRESET`
  to configure reset GPIO as nRESET, should instead set the new UICR
  ``gpio-as-nreset`` property in devicetree. It can be set like this in the
  board devicetree files:

  .. code-block:: devicetree

     &uicr {
         gpio-as-nreset;
     };

* The :kconfig:option:`CONFIG_MODEM_GSM_PPP` modem driver is obsolete.
  Instead the new :kconfig:option:`CONFIG_MODEM_CELLULAR` driver should be used.
  As part of this :kconfig:option:`CONFIG_GSM_MUX` and :kconfig:option:`CONFIG_UART_MUX` are being
  marked as deprecated as well. The new modem subsystem :kconfig:option:`CONFIG_MODEM_CMUX`
  and :kconfig:option:`CONFIG_MODEM_PPP`` should be used instead.

* Device drivers should now be restricted to ``PRE_KERNEL_1``, ``PRE_KERNEL_2``
  and ``POST_KERNEL`` initialization levels. Other device initialization levels,
  including ``EARLY``, ``APPLICATION``, and ``SMP``, have been deprecated and
  will be removed in future releases. Note that these changes do not apply to
  initialization levels used in the context of the ``init.h`` API,
  e.g. :c:macro:`SYS_INIT`.

* The following CAN controller devicetree properties are now deprecated in favor specifying the
  initial CAN bitrate using the ``bus-speed``, ``sample-point``, ``bus-speed-data``, and
  ``sample-point-data`` properties:
  * ``sjw``
  * ``prop-seg``
  * ``phase-seg1``
  * ``phase-seg1``
  * ``sjw-data``
  * ``prop-seg-data``
  * ``phase-seg1-data``
  * ``phase-seg1-data``
