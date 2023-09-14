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

Recommended Changes
*******************

* Setting the GIC architecture version by selecting
  :kconfig:option:`CONFIG_GIC_V1`, :kconfig:option:`CONFIG_GIC_V2` and
  :kconfig:option:`CONFIG_GIC_V3` directly in Kconfig has been deprecated.
  The GIC version should now be specified by adding the appropriate compatible, for
  example :dtcompatible:`arm,gic-v2`, to the GIC node in the device tree.

Picolibc-related Changes
************************

The default C library used on most targets has changed from the built-in
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
