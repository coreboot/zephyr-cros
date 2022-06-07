.. _toolchain_zephyr_sdk:

Zephyr SDK
##########

The Zephyr Software Development Kit (SDK) contains toolchains for each of
Zephyr's supported architectures. It also includes additional host tools, such
as custom QEMU and OpenOCD.

Use of the Zephyr SDK is highly recommended and may even be required under
certain conditions (for example, running tests in QEMU for some architectures).

The Zephyr SDK supports the following target architectures:

* ARC (32-bit and 64-bit; ARCv1, ARCv2, ARCv3)
* ARM (32-bit and 64-bit; ARMv6, ARMv7, ARMv8; A/R/M Profiles)
* MIPS (32-bit and 64-bit)
* Nios II
* RISC-V (32-bit and 64-bit; RV32I, RV32E, RV64I)
* x86 (32-bit and 64-bit)
* Xtensa

.. _toolchain_zephyr_sdk_install_linux:

Install Zephyr SDK on Linux
***************************

#. Download and verify the `latest Zephyr SDK bundle`_:

   .. code-block:: bash

      wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.14.2/zephyr-sdk-0.14.2_linux-x86_64.tar.gz
      wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.14.2/sha256.sum | shasum --check --ignore-missing

   You can change ``0.14.2`` to another version if needed; the `Zephyr SDK
   Releases`_ page contains all available SDK releases.

   If your host architecture is 64-bit ARM (for example, Raspberry Pi), replace
   ``x86_64`` with ``aarch64`` in order to download the 64-bit ARM Linux SDK.

#. Extract the Zephyr SDK bundle archive:

   .. code-block:: bash

      cd <sdk download directory>
      tar xvf zephyr-sdk-0.14.2_linux-x86_64.tar.gz

#. Run the Zephyr SDK bundle setup script:

   .. code-block:: bash

      cd zephyr-sdk-0.14.2
      ./setup.sh

   If this fails, make sure Zephyr's dependencies were installed as described
   in :ref:`Install Requirements and Dependencies <linux_requirements>`.

If you want to uninstall the SDK, remove the directory where you installed it.
If you relocate the SDK directory, you need to re-run the setup script.

.. note::
   It is recommended to extract the Zephyr SDK bundle at one of the following locations:

   * ``$HOME``
   * ``$HOME/.local``
   * ``$HOME/.local/opt``
   * ``$HOME/bin``
   * ``/opt``
   * ``/usr/local``

   The Zephyr SDK bundle archive contains the ``zephyr-sdk-0.14.2`` directory and, when
   extracted under ``$HOME``, the resulting installation path will be
   ``$HOME/zephyr-sdk-0.14.2``.

   If you install the Zephyr SDK outside any of these locations, you must
   register the Zephyr SDK in the CMake package registry by running the setup
   script, or set :envvar:`ZEPHYR_SDK_INSTALL_DIR` to point to the Zephyr SDK
   installation directory.

   You can also use :envvar:`ZEPHYR_SDK_INSTALL_DIR` for pointing to a
   directory containing multiple Zephyr SDKs, allowing for automatic toolchain
   selection. For example, ``ZEPHYR_SDK_INSTALL_DIR=/company/tools``, where
   the ``company/tools`` folder contains the following subfolders:

   * ``/company/tools/zephyr-sdk-0.13.2``
   * ``/company/tools/zephyr-sdk-a.b.c``
   * ``/company/tools/zephyr-sdk-x.y.z``

   This allows the Zephyr build system to choose the correct version of the
   SDK, while allowing multiple Zephyr SDKs to be grouped together at a
   specific path.

.. _toolchain_zephyr_sdk_install_macos:

Install Zephyr SDK on macOS
***************************

#. Download and verify the `latest Zephyr SDK bundle`_:

   .. code-block:: bash

      cd ~
      wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.14.2/zephyr-sdk-0.14.2_macos-x86_64.tar.gz
      wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.14.2/sha256.sum | shasum --check --ignore-missing

   If your host architecture is 64-bit ARM (Apple Silicon, also known as M1), replace
   ``x86_64`` with ``aarch64`` in order to download the 64-bit ARM macOS SDK.

#. Extract the Zephyr SDK bundle archive:

   .. code-block:: bash

      tar xvf zephyr-sdk-0.14.2_macos-x86_64.tar.gz

   .. note::
      It is recommended to extract the Zephyr SDK bundle at one of the following locations:

      * ``$HOME``
      * ``$HOME/.local``
      * ``$HOME/.local/opt``
      * ``$HOME/bin``
      * ``/opt``
      * ``/usr/local``

      The Zephyr SDK bundle archive contains the ``zephyr-sdk-0.14.2`` directory and, when
      extracted under ``$HOME``, the resulting installation path will be
      ``$HOME/zephyr-sdk-0.14.2``.

#. Run the Zephyr SDK bundle setup script:

   .. code-block:: bash

      cd zephyr-sdk-0.14.2
      ./setup.sh

   .. note::
      You only need to run the setup script once after extracting the Zephyr SDK bundle.

      You must rerun the setup script if you relocate the Zephyr SDK bundle directory after
      the initial setup.

.. _toolchain_zephyr_sdk_install_windows:

Install Zephyr SDK on Windows
*****************************

#. Open a ``cmd.exe`` window by pressing the Windows key typing "cmd.exe".

#. Download the `latest Zephyr SDK bundle`_:

   .. code-block:: console

      cd %HOMEPATH%
      wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.14.2/zephyr-sdk-0.14.2_windows-x86_64.zip

#. Extract the Zephyr SDK bundle archive:

   .. code-block:: console

      unzip zephyr-sdk-0.14.2_windows-x86_64.zip

   .. note::
      It is recommended to extract the Zephyr SDK bundle at one of the following locations:

      * ``%HOMEPATH%``
      * ``%PROGRAMFILES%``

      The Zephyr SDK bundle archive contains the ``zephyr-sdk-0.14.2`` directory and, when
      extracted under ``%HOMEPATH%``, the resulting installation path will be
      ``%HOMEPATH%\zephyr-sdk-0.14.2``.

#. Run the Zephyr SDK bundle setup script:

   .. code-block:: console

      cd zephyr-sdk-0.14.2
      setup.cmd

   .. note::
      You only need to run the setup script once after extracting the Zephyr SDK bundle.

      You must rerun the setup script if you relocate the Zephyr SDK bundle directory after
      the initial setup.

.. _latest Zephyr SDK bundle: https://github.com/zephyrproject-rtos/sdk-ng/releases
.. _Zephyr SDK Releases: https://github.com/zephyrproject-rtos/sdk-ng/releases
