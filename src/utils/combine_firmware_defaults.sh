#!/bin/bash
#
# This script embeds the Custom Defaults / Board Configuration into the firmware HEX file.
# With the combined HEX file, the FC can be flashed with any flash tool.
#
# This is especially handy in a production line, where the Configurator can't be used.
#

HEXFILE=$1
CONFIG=$2

TEMPFILE=$(mktemp)

# CUSTOM_DEFAULTS is always at this address
ADDRESS1=0x08002800
ADDRESS2=0x08002804

cat << EOM >> ${TEMPFILE}
## Rotorflight Custom Defaults
# config: ${CONFIG}
# board: ${CONFIG%%.config}
# make: RTFL
# hash: 00000000
# date: $(date -u  +%Y-%m-%dT%H:%M:%SZ)
##
EOM

grep -E '^# Rotorflight' ${CONFIG} | head -n 1 >> ${TEMPFILE}

sed -e 's/\r//g'        \
    -e 's/#.*$//g'      \
    -e 's/\s+$//g'      \
    -e 's/\s+/ /g'      \
    -e '/^\s*$/d'       \
    ${CONFIG} >> ${TEMPFILE}

echo -n -e '\n\0' >> ${TEMPFILE}

#cat ${TEMPFILE}

# Extract the CUSTOM_DEFAULTS start address
START=$(srec_cat ${HEXFILE} -Intel -crop ${ADDRESS1} ${ADDRESS2} -offset -${ADDRESS1} -output - -binary | hexdump -e '"0x%08x"')

srec_cat \
    ${HEXFILE} -Intel \
    ${TEMPFILE} -binary -offset ${START} \
    -output ${HEXFILE} -Intel -line-length=44

srec_info ${HEXFILE} -Intel

rm -f ${TEMPFILE}
