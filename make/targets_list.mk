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

OSD_TARGETS:= \
	STM32F405_OSD  \
	STM32F411_OSD  \
	STM32F7X2_OSD  \
	STM32F745_OSD  \
	STM32G47X_OSD  \
	STM32H743_OSD  \

# Legacy targets are targets that have been replaced by Unified Target configurations
LEGACY_TARGETS := \
    AG3XF4 \
    AG3XF7 \
    AIKONF4 \
    AIRBOTF4 \
    AIRBOTF4SD \
    AIRBOTF7 \
    AIRF7 \
    ANYFCF7 \
    ANYFCM7 \
    BEEROTORF4 \
    BETAFLIGHTF4 \
    BETAFPVF4SX1280 \
    BLUEJAYF4 \
    CLRACINGF4 \
    CLRACINGF7 \
    CRAZYBEEF4DX \
    CRAZYBEEF4FR \
    CRAZYBEEF4FS \
    CRAZYBEEF4SX1280 \
    DALRCF405 \
    DALRCF722DUAL \
    DEVEBOXH743 \
    DYSF4PRO \
    ELINF405 \
    ELINF722 \
    ELLE0 \
    EMAXF4SX1280 \
    EXF722DUAL \
    EXUAVF4PRO \
    F4BY \
    FF_FORTINIF4 \
    FF_FORTINIF4_REV03 \
    FF_PIKOF4 \
    FF_PIKOF4OSD \
    FF_RACEPIT \
    FISHDRONEF4 \
    FLYWOOF405 \
    FLYWOOF411 \
    FLYWOOF7DUAL \
    FOXEERF405 \
    FOXEERF722DUAL \
    FPVM_BETAFLIGHTF7 \
    FRSKYF4 \
    FURYF4 \
    FURYF4OSD \
    HAKRCF405 \
    HAKRCF411 \
    HAKRCF722 \
    IFLIGHT_H743_AIO \
    IFLIGHT_H743_AIO_V2 \
    JHEF7DUAL \
    KAKUTEF4 \
    KAKUTEF4V2 \
    KAKUTEF7 \
    KAKUTEF7MINI \
    KAKUTEF7V2 \
    KISSFCV2F7 \
    KIWIF4 \
    KIWIF4V2 \
    KROOZX \
    LUXF4OSD \
    LUXMINIF7 \
    MAMBAF411 \
    MAMBAF722 \
    MATEKF405 \
    MATEKF411 \
    MATEKF411RX \
    MATEKF722 \
    MATEKF722SE \
    MATEKH743 \
    MERAKRCF405 \
    MERAKRCF722 \
    MLTEMPF4 \
    MLTYPHF4 \
    NERO \
    NEUTRONRCF411SX1280 \
    NOX \
    OMNIBUSF4 \
    OMNIBUSF4FW \
    OMNIBUSF4NANOV7 \
    OMNIBUSF4SD \
    OMNIBUSF4V6 \
    OMNIBUSF7 \
    OMNIBUSF7NANOV7 \
    OMNIBUSF7V2 \
    OMNINXT4 \
    OMNINXT7 \
    PIRXF4 \
    PLUMF4 \
    PODIUMF4 \
    PYRODRONEF4 \
    REVO \
    REVOLT \
    REVOLTOSD \
    REVONANO \
    RUSHCORE7 \
    SKYZONEF405 \
    SOULF4 \
    SPARKY2 \
    SPEEDYBEEF4 \
    SPEEDYBEEF7 \
    SPEKTRUMF400 \
    SPRACINGF4EVO \
    SPRACINGF4NEO \
    SPRACINGF7DUAL \
    STACKX \
    SYNERGYF4 \
    TMOTORF4 \
    TMOTORF7 \
    TRANSTECF411 \
    TRANSTECF7 \
    UAVPNG030MINI \
    VGOODRCF4 \
    VRRACE \
    WORMFC \
    XILOF4 \
    XRACERF4 \
    YUPIF4 \
    YUPIF7 \

CI_TARGETS := $(filter-out $(LEGACY_TARGETS), $(VALID_TARGETS))

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
