################################################
# Auto-generated as part of running getlibs 
################################################

# List of local libraries 
SEARCH_TARGET_CY8CKIT-062-BLE=./libs/TARGET_CY8CKIT-062-BLE

# List of shared libraries 
SEARCH_capsense=../mtb_shared/capsense/release-v3.0.0
SEARCH_core-lib=../mtb_shared/core-lib/release-v1.3.0
SEARCH_core-make=../mtb_shared/core-make/release-v1.9.1
SEARCH_mtb-hal-cat1=../mtb_shared/mtb-hal-cat1/release-v2.1.0
SEARCH_mtb-pdl-cat1=../mtb_shared/mtb-pdl-cat1/release-v2.4.0
SEARCH_psoc6cm0p=../mtb_shared/psoc6cm0p/release-v2.2.0
SEARCH_recipe-make-cat1a=../mtb_shared/recipe-make-cat1a/release-v1.9.0

# Shared libraries added to build 
SEARCH+=$(SEARCH_capsense)
SEARCH+=$(SEARCH_core-lib)
SEARCH+=$(SEARCH_core-make)
SEARCH+=$(SEARCH_mtb-hal-cat1)
SEARCH+=$(SEARCH_mtb-pdl-cat1)
SEARCH+=$(SEARCH_psoc6cm0p)
SEARCH+=$(SEARCH_recipe-make-cat1a)
-include $(CY_INTERNAL_APP_PATH)/importedbsp.mk
