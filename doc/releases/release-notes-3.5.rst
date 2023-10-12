:orphan:

.. _zephyr_3.5:

Zephyr 3.5.0 (Working Draft)
############################

We are pleased to announce the release of Zephyr version 3.5.0.

Major enhancements with this release include:

* Added native_sim (successor to native_posix)

The following sections provide detailed lists of changes by component.

Security Vulnerability Related
******************************
The following CVEs are addressed by this release:

More detailed information can be found in:
https://docs.zephyrproject.org/latest/security/vulnerabilities.html

* CVE-2023-3725 `Zephyr project bug tracker GHSA-2g3m-p6c7-8rr3
  <https://github.com/zephyrproject-rtos/zephyr/security/advisories/GHSA-2g3m-p6c7-8rr3>`_

* CVE-2023-4257: Under embargo until 2023-10-12

* CVE-2023-4258 `Zephyr project bug tracker GHSA-m34c-cp63-rwh7
  <https://github.com/zephyrproject-rtos/zephyr/security/advisories/GHSA-m34c-cp63-rwh7>`_

* CVE-2023-4260 `Zephyr project bug tracker GHSA-gj27-862r-55wh
  <https://github.com/zephyrproject-rtos/zephyr/security/advisories/GHSA-gj27-862r-55wh>`_

* CVE-2023-4264 `Zephyr project bug tracker GHSA-rgx6-3w4j-gf5j
  <https://github.com/zephyrproject-rtos/zephyr/security/advisories/GHSA-rgx6-3w4j-gf5j>`_

* CVE-2023-4424: Under embargo until 2023-11-01

* CVE-2023-5055: Under embargo until 2023-11-01

* CVE-2023-5139: Under embargo until 2023-10-25

* CVE-2023-5184 `Zephyr project bug tracker GHSA-8x3p-q3r5-xh9g
  <https://github.com/zephyrproject-rtos/zephyr/security/advisories/GHSA-8x3p-q3r5-xh9g>`_


Kernel
******

* Added support for dynamic thread stack allocation via :c:func:`k_thread_stack_alloc`
* Added support for :c:func:`k_spin_trylock`
* Added :c:func:`k_object_is_valid` to check if a kernel object is valid. This replaces
  code that has been duplicated throughout the tree.

Architectures
*************

* ARM

  * Architectural support for Arm Cortex-M has been separated from Arm
    Cortex-A and Cortex-R. This includes separate source modules to handle
    tasks like IRQ management, exception handling, thread handling and swap.
    For implementation details see :github:`60031`.

* ARM

* ARM64

* RISC-V

* Xtensa

  * Added basic MMU v2 Support.

* POSIX

  * Has been reworked to use the native simulator.
  * New boards have been added.
  * For the new boards, embedded C libraries can be used, and conflicts with the host symbols
    and libraries avoided.
  * The :ref:`POSIX OS abstraction<posix_support>` is supported in these new boards.
  * AMP targets are now supported.
  * Added support for LLVM source profiling/coverage.

Bluetooth
*********

* Audio

* Direction Finding

* Host

* Mesh

* Controller

* HCI Driver

Boards & SoC Support
********************

* Added support for these SoC series:

  * Nuvoton NuMaker M46x series

* Removed support for these SoC series:

* Made these changes in other SoC series:

  * i.MX RT SOCs no longer enable CONFIG_DEVICE_CONFIGURATION_DATA by default.
    boards using external SDRAM should set CONFIG_DEVICE_CONFIGURATION_DATA
    and CONFIG_NXP_IMX_EXTERNAL_SDRAM to enabled.
  * i.MX RT SOCs no longer support CONFIG_OCRAM_NOCACHE, as this functionality
    can be achieved using devicetree memory regions
  * Refactored ESP32 SoC folders. So now these are a proper SoC series.

* Added support for these ARC boards:

* Added support for these ARM boards:

  * Nuvoton NuMaker Platform M467

* Added support for these ARM64 boards:

* Added support for these RISC-V boards:

* Added support for these X86 boards:

* Added support for these Xtensa boards:

  * Added ``esp32_devkitc_wroom`` and ``esp32_devkitc_wrover``.

  * Added ``esp32s3_luatos_core``.

  * Added ``m5stack_core2``.

  * Added ``qemu_xtensa_mmu`` utilizing Diamond DC233c SoC to support
    testing Xtensa MMU.

  * Added ``xiao_esp32s3``.

  * Added ``yd_esp32``.

* Added support for these POSIX boards:

  * :ref:`native_sim(_64) <native_sim>`
  * nrf5340bsim_nrf5340_cpu(net|app). A simulated nrf5340 SOC, which uses Babblesim for its radio
    traffic.

* Made these changes for ARC boards:

* Made these changes for ARM boards:

* Made these changes for ARM64 boards:

* Made these changes for RISC-V boards:

* Made these changes for X86 boards:

* Made these changes for Xtensa boards:

  * esp32s3_devkitm:

    * Added USB-CDC support.

    * Added CAN support.

* Made these changes for POSIX boards:

  * nrf52_bsim:

    * Has been reworked to use the native simulator as its runner.
    * Multiple HW models improvements and fixes. GPIO & GPIOTE peripherals added.

* Removed support for these ARC boards:

* Removed support for these ARM boards:

* Removed support for these ARM64 boards:

* Removed support for these RISC-V boards:

* Removed support for these X86 boards:

* Removed support for these Xtensa boards:

  * Removed ``esp32``. Use ``esp32_devkitc_*`` instead.

* Made these changes in other boards:

* Added support for these following shields:

Build system and infrastructure
*******************************

* SCA (Static Code Analysis)

  * Added support for CodeChecker

* Twister now supports ``required_snippets`` in testsuite .yml files, this can
  be used to include a snippet when a test is ran (and exclude any boards from
  running that the snippet cannot be applied to).

* Interrupts

  * Added support for shared interrupts

* Added support for setting MCUboot encryption key in sysbuild which is then
  propagated to the bootloader and target images to automatically create
  encrypted updates.

* Build time priority checking: enable build time priority checking by default.
  This fails the build if the initialization sequence in the final ELF file
  does not match the devicetree hierarchy. It can be turned off by disabling
  the :kconfig:option:`COFNIG_CHECK_INIT_PRIORITIES` option.

* Added a new ``initlevels`` target for printing the final device and
  :c:macro:`SYS_INIT` initialization sequence from the final ELF file.

* Reworked syscall code generations so that not all marshalling functions
  will be included in the final binary. Syscalls associated with disabled
  subsystems no longer have their marshalling functions generated.

* Partially enabled compiler warning about shadow variables for subset of
  in-tree code. Out-of-tree code needs to be patched before we can fully
  enable shadow variable warnings.

Drivers and Sensors
*******************

* ADC

* Battery-backed RAM

* CAN

  * Added support for TI TCAN4x5x CAN-FD controller with integrated transceiver
    (:dtcompatible:`ti,tcan4x5x`).
  * Added support for Microchip MCP251xFD CAN-FD controller (:dtcompatible:`microchip,mcp251xfd`).
  * Added support for CAN statistics to the Bosch M_CAN controller driver backend.
  * Switched the NXP S32 CANXL driver to use clock control for the CAN clock instead of hard-coding
    a CAN clock frequency in the devicetree.

* Clock control

  * Added support for Nuvoton NuMaker M46x

* Counter

* Crypto

* DAC

* DFU

* Disk

* Display

  * Added support for ST7735S (in ST7735R driver)

* DMA

* EEPROM

  * Added support for Fujitsu MB85RCxx series I2C FRAM (:dtcompatible:`fujitsu,mb85rcxx`).

* Entropy

  * Added a requirement for ``entropy_get_entropy()`` to be thread-safe because
    of random subsystem needs.

* ESPI

* Ethernet

  * Added :kconfig:option:`CONFIG_ETH_NATIVE_POSIX_RX_TIMEOUT` to set rx timeout for native posix.
  * Added support for adin2111.
  * Added support for NXP S32 GMAC.
  * Added support for promiscuous mode in eth_smsc91x.
  * Added support for STM32H5X SOC series.
  * Added support for MDIO Clause 45 APIs.
  * Added support for YD-ESP32 board Ethernet.
  * Fixed stm32 to generate more unique MAC address by using device id as a base for the MAC.
  * Fixed mcux to increase the PTP timestamp accuracy from 20us to 200ns.
  * Fixed Ethernet max header size when using VLAN.
  * Removed the ``mdio`` DT property. Please use :c:macro:`DT_INST_BUS()` in the driver instead.
  * Reworked the device node hierarchy in smsc91x.
  * Renamed the phy-dev property with phy-handle to match the Linux ethernet-controller binding
    and move it up to ethernet.yaml so that it can be used by other drivers.
  * Updated Ethernet PHY to use ``reg`` property in DT bindings.
  * Updated driver DT bindings to use ``ethernet-phy`` devicetree node name consistently.
  * Updated esp32 and sam-gmac DT so that the phy is pointed by a phandle rather than
    a child node, this makes the phy device a child of mdio.

* Flash

  * Introduce npcx flash driver that supports two or more spi nor flashes via a
    single Flash Interface Unit (FIU) module and Direct Read Access (DRA) mode
    for better performance.
  * Added support for Nuvoton NuMaker M46x embedded flash

* FPGA

* Fuel Gauge

* GPIO

  * Added support for Nuvoton NuMaker M46x

* hwinfo

* I2C

* I2S

* I3C

  * ``i3c_cdns``:

    * Fixed build error when :kconfig:option:`CONFIG_I3C_USE_IBI` is disabled.

    * Fixed transfer issue when controller is busy. Now wait for controller to
      idle before proceeding with another transfer.

* IEEE 802.15.4

  * A new mandatory method attr_get() was introduced into ieee802154_radio_api.
    Drivers need to implement at least
    IEEE802154_ATTR_PHY_SUPPORTED_CHANNEL_PAGES and
    IEEE802154_ATTR_PHY_SUPPORTED_CHANNEL_RANGES.
  * The hardware capabilities IEEE802154_HW_2_4_GHZ and IEEE802154_HW_SUB_GHZ
    were removed as they were not aligned with the standard and some already
    existing drivers couldn't properly express their channel page and channel
    range (notably SUN FSK and HRP UWB drivers). The capabilities were replaced
    by the standard conforming new driver attribute
    IEEE802154_ATTR_PHY_SUPPORTED_CHANNEL_PAGES that fits all in-tree drivers.
  * The method get_subg_channel_count() was removed from ieee802154_radio_api.
    This method could not properly express the channel range of existing drivers
    (notably SUN FSK drivers that implement channel pages > 0 and may not have
    zero-based channel ranges or UWB drivers that could not be represented at
    all). The method was replaced by the new driver attribute
    IEEE802154_ATTR_PHY_SUPPORTED_CHANNEL_RANGES that fits all in-tree drivers.

* Interrupt Controller

  * GIC: Architecture version selection is now based on the device tree

* Input

  * New drivers: :dtcompatible:`gpio-qdec`, :dtcompatible:`st,stmpe811`.

  * Drivers converted from Kscan to Input: :dtcompatible:`goodix,gt911`
    :dtcompatible:`xptek,xpt2046` :dtcompatible:`hynitron,cst816s`
    :dtcompatible:`microchip,cap1203`.

  * Added a Kconfig option for dumping all events to the console
    :kconfig:option:`CONFIG_INPUT_EVENT_DUMP` and new shell commands
    :kconfig:option:`CONFIG_INPUT_SHELL`.

  * Merged ``zephyr,gpio-keys`` into :dtcompatible:`gpio-keys` and added
    ``zephyr,code`` codes to all in-tree board ``gpio-keys`` nodes.

  * Renamed the callback definition macro from ``INPUT_LISTENER_CB_DEFINE`` to
    :c:macro:`INPUT_CALLBACK_DEFINE`.

* IPM

* KSCAN

* LED

* MBOX

* MEMC

* PCIE

  * Added support in shell to display PCIe capabilities.

  * Added virtual channel support.

  * Added kconfig :kconfig:option:`CONFIG_PCIE_INIT_PRIORITY` to specify
    initialization priority for host controller.

  * Added support to get IRQ from ACPI PCI Routing Table (PRT).

* PECI

* Pin control

  * Added support for Nuvoton NuMaker M46x

* PWM

* Power domain

* Regulators

  * Added support for GPIO-controlled voltage regulator

  * Added support for AXP192 PMIC

  * Added support for NXP VREF regulator

  * Fixed regulators can now specify their operating voltage

  * PFM mode is now support for nPM1300

  * Added new API to configure "ship" mode

  * Regulator shell allows to configure DVS modes

* Reset

  * Added support for Nuvoton NuMaker M46x

* Retained memory

  * Added support for allowing mutex support to be forcibly disabled with
    :kconfig:option:`CONFIG_RETAINED_MEM_MUTEX_FORCE_DISABLE`.

  * Fixed issue with user mode support not working.

* SDHC

* Sensor

  * Reworked the :dtcompatible:`ti,bq274xx` to add ``BQ27427`` support, fixed
    units for capacity and power channels.

* Serial

  * Added support for Nuvoton NuMaker M46x

  * NS16550: Reworked how device initialization macros.

    * ``CONFIG_UART_NS16550_ACCESS_IOPORT`` and ``CONFIG_UART_NS16550_SIMULT_ACCESS``
      are removed. For UART using IO port access, add ``io-mapped`` property to
      device tree node.

  * Added async support for ESP32S3.

  * Added support for serial TTY under ``native_posix``.

  * Added support for UART on Efinix Sapphire SoCs.

  * Added Intel SEDI UART driver.

  * Added support for UART on BCM2711.

  * ``uart_stm32``:

    * Added RS485 support.

    * Added wide data support.

  * ``uart_pl011``: added support for Ambiq SoCs.

  * ``serial_test``: added support for interrupt and async APIs.

  * ``uart_emul``: added support for interrupt API.

* SPI

  * Remove npcx spi driver implemented by Flash Interface Unit (FIU) module.

* Timer

  * The TI CC13xx/26xx system clock timer compatible was changed from
    :dtcompatible:`ti,cc13xx-cc26xx-rtc` to :dtcompatible:`ti,cc13xx-cc26xx-rtc-timer`
    and the corresponding Kconfig option from :kconfig:option:`CC13X2_CC26X2_RTC_TIMER`
    to :kconfig:option:`CC13XX_CC26XX_RTC_TIMER` for improved consistency and
    extensibility. No action is required unless the internal timer was modified.

* USB

* W1

* Watchdog

* WiFi

  * Increased esp32 default network (TCP workq, RX and mgmt event) stack sizes to 2048 bytes.
  * Reduced the RAM usage for esp32s2_saola in Wi-Fi samples.
  * Fixed undefined declarations in winc1500.
  * Fixed SPI buffer length in eswifi.
  * Fixed esp32 data sending and channel selection in AP mode.
  * Fixed esp_at driver init and network interface dormant state setting.

Networking
**********

* Time and timestamps in the network subsystem, PTP and IEEE 802.15.4
  were more precisely specified and all in-tree call sites updated accordingly.
  Fields for timed TX and TX/RX timestamps have been consolidated. See
  :c:type:`net_time_t`, :c:struct:`net_ptp_time`, :c:struct:`ieee802154_config`,
  :c:struct:`ieee802154_radio_api` and :c:struct:`net_pkt` for extensive
  documentation. As this is largely an internal API, existing applications will
  most probably continue to work unchanged.

* CoAP:

  * Use 64 bit timer values for calculating transmission timeouts. This fixes potential problems for
    devices that stay on for more than 49 days when the 32 bit uptime counter might roll over and
    cause CoAP packets to not timeout at all on this event.

* Ethernet:

  * Fixed ARP queueing so that the queued network packet is sent immediately
    instead of queued 2nd time in the core network stack.

* gPTP:

  * Added support for detecting gPTP packets that use the default multicast destination address.
  * Fixed Announce and Follow Up message handling.

* LwM2M:

  * Added support for tickless mode. This removes the 500 ms timeout from the socket loop
    so the engine does not constantly wake up the CPU. This can be enabled by
    :kconfig:option:`CONFIG_LWM2M_TICKLESS`.
  * Added new :c:macro:`LWM2M_RD_CLIENT_EVENT_DEREGISTER` event.

* Wi-Fi:

  * Added Passive scan support.
  * The Wi-Fi scan API updated with Wi-Fi scan parameter to allow scan mode selection.
  * Updated TWT handling.
  * Added support for generic network manager API.
  * Added support for Wi-Fi mode setting and selection.
  * Added user input validation for SSID and PSK in Wi-Fi shell.
  * Added scan extension for specifying channels, limiting scan results, filtering SSIDs,
    setting active and passive channel dwell times and frequency bands.

USB
***

* USB device HID
  * Kconfig option USB_HID_PROTOCOL_CODE, deprecated in v2.6, is finally removed.

Devicetree
**********

Libraries / Subsystems
**********************

* Management

  * Introduced MCUmgr client support with handlers for img_mgmt and os_mgmt.

  * Added response checking to MCUmgr's :c:enumerator:`MGMT_EVT_OP_CMD_RECV`
    notification callback to allow applications to reject MCUmgr commands.

  * MCUmgr SMP version 2 error translation (to legacy MCUmgr error code) is now
    supported in function handlers by setting ``mg_translate_error`` of
    :c:struct:`mgmt_group` when registering a group. See
    :c:type:`smp_translate_error_fn` for function details.

  * Fixed an issue with MCUmgr img_mgmt group whereby the size of the upload in
    the initial packet was not checked.

  * Fixed an issue with MCUmgr fs_mgmt group whereby some status codes were not
    checked properly, this meant that the error returned might not be the
    correct error, but would only occur in situations where an error was
    already present.

  * Fixed an issue whereby the SMP response function did not check to see if
    the initial zcbor map was created successfully.

  * Fixes an issue with MCUmgr shell_mgmt group whereby the length of a
    received command was not properly checked.

  * Added optional mutex locking support to MCUmgr img_mgmt group, which can
    be enabled with :kconfig:option:`CONFIG_MCUMGR_GRP_IMG_MUTEX`.

  * Added MCUmgr settings management group, which allows for manipulation of
    zephyr settings from a remote device, see :ref:`mcumgr_smp_group_3` for
    details.

  * Added :kconfig:option:`CONFIG_MCUMGR_GRP_IMG_ALLOW_CONFIRM_NON_ACTIVE_IMAGE_SECONDARY`
    and :kconfig:option:`CONFIG_MCUMGR_GRP_IMG_ALLOW_CONFIRM_NON_ACTIVE_IMAGE_ANY`
    that allow to control whether MCUmgr client will be allowed to confirm
    non-active images.

  * Added :kconfig:option:`CONFIG_MCUMGR_GRP_IMG_ALLOW_ERASE_PENDING` that allows
    to erase slots pending for next boot, that are not revert slots.

  * Added ``user_data`` as an optional field to :c:struct:`mgmt_handler` when
    :kconfig:option:`CONFIG_MCUMGR_MGMT_HANDLER_USER_DATA` is enabled.

  * Added optional ``force`` parameter to os mgmt reset command, this can be checked in the
    :c:enum:`MGMT_EVT_OP_OS_MGMT_RESET` notification callback whose data structure is
    :c:struct:`os_mgmt_reset_data`.

  * Added configurable number of SMP encoding levels via
    :kconfig:option:`CONFIG_MCUMGR_SMP_CBOR_MIN_ENCODING_LEVELS`, which automatically increments
    minimum encoding levels for in-tree groups if :kconfig:option:`CONFIG_ZCBOR_CANONICAL` is
    enabled.

* File systems

  * Added support for ext2 file system.
  * Added support of mounting littlefs on the block device from the shell/fs.
  * Added alignment parameter to FS_LITTLEFS_DECLARE_CUSTOM_CONFIG macro, it can speed up read/write
    operation for SDMMC devices in case when we align buffers on CONFIG_SDHC_BUFFER_ALIGNMENT,
    because we can avoid extra copy of data from card bffer to read/prog buffer.

* Random

  * ``CONFIG_XOROSHIRO_RANDOM_GENERATOR``, deprecated a long time ago, is finally removed.

* Retention

  * Added the :ref:`blinfo_api` subsystem.

  * Added support for allowing mutex support to be forcibly disabled with
    :kconfig:option:`CONFIG_RETENTION_MUTEX_FORCE_DISABLE`.

* Binary descriptors

  * Added the :ref:`binary_descriptors` (``bindesc``) subsystem.

* POSIX API

  * Added dynamic thread stack support for :c:func:`pthread_create`
  * Fixed :c:func:`stat` so that it returns file stats instead of filesystem stats
  * Implemented :c:func:`pthread_barrierattr_destroy`, :c:func:`pthread_barrierattr_getpshared`,
    :c:func:`pthread_barrierattr_init`, :c:func:`pthread_barrierattr_setpshared`,
    :c:func:`pthread_condattr_destroy`, :c:func:`pthread_condattr_init`,
    :c:func:`pthread_mutexattr_destroy`, :c:func:`pthread_mutexattr_init`, :c:func:`uname`,
    :c:func:`sigaddset`, :c:func:`sigdelset`, :c:func:`sigemptyset`, :c:func:`sigfillset`,
    :c:func:`sigismember`, :c:func:`strsignal`, :c:func:`pthread_spin_destroy`,
    :c:func:`pthread_spin_init`, :c:func:`pthread_spin_lock`, :c:func:`pthread_spin_trylock`,
    :c:func:`pthread_spin_unlock`, :c:func:`timer_getoverrun`, :c:func:`pthread_condattr_getclock`,
    :c:func:`pthread_condattr_setclock`, :c:func:`clock_nanosleep`
  * Added support for querying the number of bytes available to read via the
    :c:macro:`FIONREAD` request to :c:func:`ioctl`
  * Added :kconfig:option:`CONFIG_FDTABLE` to conditionally compile file descriptor table
  * Added logging to POSIX threads, mutexes, and condition variables
  * Fixed :c:func:`poll` issue with event file descriptors

HALs
****

* Nuvoton

  * Added Nuvoton NuMaker M46x

MCUboot
*******

  * Added :kconfig:option:`CONFIG_MCUBOOT_BOOTLOADER_NO_DOWNGRADE`
    that allows to inform application that the on-board MCUboot has been configured
    with downgrade  prevention enabled. This option is automatically selected for
    DirectXIP mode and is available for both swap modes.

  * Added :kconfig:option:`CONFIG_MCUBOOT_BOOTLOADER_MODE_OVERWRITE_ONLY`
    that allows to inform application that the on-board MCUboot will overwrite
    the primary slot with secondary slot contents, without saving the original
    image in primary slot.

  * Fixed issue with serial recovery not showing image details for decrypted images.

  * Fixed issue with serial recovery in single slot mode wrongly iterating over 2 image slots.

  * Fixed an issue with boot_serial repeats not being processed when output was sent, this would
    lead to a divergence of commands whereby later commands being sent would have the previous
    command output sent instead.

  * Fixed an issue with the boot_serial zcbor setup encoder function wrongly including the buffer
    address in the size which caused serial recovery to fail on some platforms.

  * Fixed wrongly building in optimize for debug mode by default, this saves a significant amount
    of flash space.

  * Fixed issue with serial recovery use of MBEDTLS having undefined operations which led to usage
    faults when the secondary slot image was encrypted.

  * Fixed issue with bootutil asserting on maximum alignment in non-swap modes.

  * Added error output when flash device fails to open and asserts are disabled, which will now
    panic the bootloader.

  * Added currently running slot ID and maximum application size to shared data function
    definition.

  * Added P384 and SHA384 support to imgtool.

  * Added optional serial recovery image state and image set state commands.

  * Added ``dumpinfo`` command for signed image parsing in imgtool.

  * Added ``getpubhash`` command to dump the sha256 hash of the public key in imgtool.

  * Added support for ``getpub`` to print the output to a file in imgtool.

  * Added support for dumping the raw versions of the public keys in imgtool.

  * Added support for sharing boot information with application via retention subsystem.

  * Added support for serial recovery to read and handle encrypted seondary slot partitions.

  * Removed ECDSA P224 support.

  * Removed custom image list boot serial extension support.

  * Reworked boot serial extensions so that they can be used by modules or from user repositories
    by switching to iterable sections.

  * Reworked image encryption support for Zephyr, static dummy key files are no longer in the code,
    a pem file must be supplied to extract the private and public keys. The Kconfig menu has
    changed to only show a single option for enabling encryption and selecting the key file.

  * Reworked the ECDSA256 TLV curve agnostic and renamed it to ``ECDSA_SIG``.

  * CDDL auto-generated function code has been replaced with zcbor function calls, this now allows
    the parameters to be supplied in any order.

  * The MCUboot version in this release is version ``2.0.0+0-rc1``.

Nanopb
******

  * Changed project status to maintained.

  * Added a separate nanopb.cmake file to be included by applications.

  * Added helper cmake function ``zephyr_nanopb_sources`` to simplify ``.proto`` file inclusion.

Storage
*******

Trusted Firmware-M
******************

Trusted Firmware-A
******************

* Updated to TF-A 2.9.0.

zcbor
*****

Documentation
*************

* Upgraded Sphinx to 6.2

Tests and Samples
*****************

* Created common sample for file systems (`fs_sample`). It originates from sample for FAT
  (`fat_fs`) and supports both FAT and ext2 file systems.

Known Issues
************
