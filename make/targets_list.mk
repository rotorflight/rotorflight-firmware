ALT_TARGET_PATHS  = $(filter-out %/target,$(basename $(wildcard $(ROOT)/src/main/target/*/*.mk)))
ALT_TARGET_NAMES  = $(notdir $(ALT_TARGET_PATHS))
BASE_TARGET_NAMES = $(notdir $(patsubst %/,%,$(dir $(ALT_TARGET_PATHS))))
BASE_ALT_PAIRS    = $(join $(BASE_TARGET_NAMES:=/),$(ALT_TARGET_NAMES))

ALT_TARGETS       = $(sort $(notdir $(BASE_ALT_PAIRS)))
BASE_TARGETS      = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(ROOT)/src/main/target/*/target.mk)))))
NOBUILD_TARGETS   = $(sort $(filter-out target,$(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.nomk)))))
VALID_TARGETS     = $(sort $(filter-out $(NOBUILD_TARGETS),$(BASE_TARGETS) $(ALT_TARGETS)))

# For alt targets, returns their base target name.
# For base targets, returns the (same) target name.
# param $1 = target name
find_target_pair  = $(filter %/$(1),$(BASE_ALT_PAIRS))
get_base_target   = $(if $(call find_target_pair,$(1)),$(patsubst %/,%,$(dir $(call find_target_pair,$(1)))),$(1))

UNIFIED_TARGETS := \
	STM32F405  \
	STM32F411  \
	STM32F7X2  \
	STM32F745  \
	STM32G47X  \
	STM32H743  \

# Legacy targets are targets that have been replaced by Unified Target configurations
LEGACY_TARGETS := \
    DEVEBOXH743 \
    MATEKF405 \
    MATEKF411 \
    MATEKF722 \
    MATEKH743 \
    NUCLEOF722 \
    NUCLEOH743 \

CI_TARGETS := $(UNIFIED_TARGETS) \
    MATEKF405 \
    MATEKF411 \
    MATEKF722 \
    MATEKH743 \

TARGETS_TOTAL := $(words $(CI_TARGETS))
TARGET_GROUPS := 3
TARGETS_PER_GROUP := $(shell expr $(TARGETS_TOTAL) / $(TARGET_GROUPS) )

ST := 1
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_1_TARGETS := $(wordlist  $(ST), $(ET), $(CI_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_2_TARGETS := $(wordlist $(ST), $(ET), $(CI_TARGETS))

GROUP_OTHER_TARGETS := $(filter-out $(GROUP_1_TARGETS) $(GROUP_2_TARGETS), $(CI_TARGETS))
