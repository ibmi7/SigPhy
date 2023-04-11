add_custom_target(hci_rpmsg_shared_property_target)
add_custom_target(net_core_shared_property_target)
add_custom_target(CPUNET_shared_property_target)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY KERNEL_HEX_NAME;zephyr.hex)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY ZEPHYR_BINARY_DIR;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY KERNEL_ELF_NAME;zephyr.elf)
set_property(TARGET hci_rpmsg_shared_property_target  PROPERTY BUILD_BYPRODUCTS;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr/zephyr.hex;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr/zephyr.elf)
set_property(TARGET net_core_shared_property_target  PROPERTY SOC;nRF5340_CPUNET_QKAA)
set_property(TARGET net_core_shared_property_target  PROPERTY VERSION;1)
include(C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/b0n/shared_vars.cmake)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_PARTITIONS;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/partitions_CPUNET.yml)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_REGIONS;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/regions_CPUNET.yml)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_HEADER_FILES;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/b0n/zephyr/include/generated/pm_config.h;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr/include/generated/pm_config.h)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOMAIN_IMAGES;CPUNET:b0n;CPUNET:hci_rpmsg)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_HEX_FILE;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr/merged_CPUNET.hex)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_DOTCONF_FILES;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/pm_CPUNET.config)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_APP_HEX;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr/app.hex)
set_property(TARGET hci_rpmsg_shared_property_target APPEND PROPERTY BUILD_BYPRODUCTS;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr/merged_CPUNET.hex)
set_property(TARGET CPUNET_shared_property_target  PROPERTY PM_SIGNED_APP_HEX;C:/Users/cours/Documents/Cours/EISE4/SigPhy/Test/ble_comprehensive/build/hci_rpmsg/zephyr/signed_by_b0_app.hex)
