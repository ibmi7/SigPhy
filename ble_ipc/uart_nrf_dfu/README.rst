.. nrf_dfu_ipc:

uart nrf_dfu with ipc example(external secondary slot)
######################################################

.. contents::
   :local:
   :depth: 2

Overview
********

This sample works with ``netcore_ipc`` together. It will load, build and run ``netcore_ipc`` automatically. 

Use DFU module from nRF5 SDK v17.0.2 to do DFU in nRF Connect SDK. This DFU module is called nrf_dfu in this document. The DFU procedure is exactly the
same as that of nRF5 SDK. See https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.0.2%2Fble_sdk_app_dfu_bootloader.html
for a detailed description of nRF5 SDK DFU steps if you don't have too much knowledge of it.

In this sample, the secondary slot is on the external QSPI Flash. That is, the new image will be transferred by UART and stored in the secondary slot. 
After that, MCUBoot would perform the swap operation to finish the whole DFU process. 

**note: In this sample, MCUBoot uses the default signing key, which must be replaced with your own key before production.** Do it like below:

.. code-block:: console

	CONFIG_BOOT_SIGNATURE_KEY_FILE="my_mcuboot_private.pem"	

Build & Programming
*******************

Make sure ``netcore_ipc`` is placed in the following folder structure.

::

    NCS root folder
    ├── nrf
    ├── zephyr
    ├── ncs_samples          
    │   ├── ble_ipc
	│      ├── netcore_ipc 


The following NCS tags are tested for this sample. By default, NCS ``v2.2.0`` is used.

+------------------------------------------------------------------+
|NCS tags                                                          +
+==================================================================+
|v1.9.x/v2.0.x/v2.2.x                                              |
+------------------------------------------------------------------+

Use ``git tag`` to see supported tags. For ncs versions later than v1.9.0, for example ncs v2.0.0, 
use ``git checkout v2.0`` to switch to the specified NCS tag. Use ``git checkout v1.5_v1.9`` to switch to 
ncs versions earlier than v1.9.0. After the checkout operation, open this README.rst again and follow 
the instructions. 
	
This example **may** modify the original NCS source code. Refer to ``sdk_change`` for the detailed changes. 
For example, to work with NCS v1.9.1, copy folder ``sdk_change/ncs_v1.9.x`` and overwrite the same files 
in the correspondent NCS ``v1.9.1`` folders.

The following development kits are tested for this sample.

+------------------------------------------------------------------+
|Build target                                                      +
+==================================================================+
|nrf5340dk_nrf5340_cpuapp                                          |
+------------------------------------------------------------------+

For example, enter the following command to build ``nrf5340dk_nrf5340_cpuapp``.

.. code-block:: console

   west build -b nrf5340dk_nrf5340_cpuapp -d build_nrf5340dk_nrf5340_cpuapp -p

To flash the images to the board, just double click ``program.bat``, or use the following command:

.. code-block:: console

   west flash -d build_nrf5340dk_nrf5340_cpuapp     

Testing
*******

After programming the sample to your development kit, test it by performing the following steps:

1. Connect the kit to the computer using a USB cable. The kit is assigned a COM port (Windows) or ttyACM device (Linux), which is visible in the Device Manager.
#. Reset the kit. It shall advertise ``nrf_dfu_ipc``.
#. Copy app_signed.hex in folder ``build*/zephyr`` (and net_core_app_signed.hex if you want to upgrade network core of nRF5340) to folder ``update``.
#. Double click ``upload_app.bat`` (change the COM port if needed) in folder ``update``. (If you want to update nRF5340 network core, edit ``upload_app.bat`` to uncomment the last 3 lines)
#. Waiting until the transfer is done. 
#. DFU is completed. 
