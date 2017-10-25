# Flash-Resident Bootloader for KL27Z
## Design And Implementation Notes
> Robert Poor <rdpoor@gmail.com> October 2017

## Overview
Our goal is to create a flash-resident bootloader for the KL27Z family of 
processors in order to load programs into flash memory.  It is designed to be
compatible with the Kinetis KBOOT 2.0 protocol. 

As a code base, we started with the bootloader for the KL25Z processor and used 
the following guidelines while porting the code:

1. Use the Project Wizard to create a blank Kinetis 2.2 project, specifically 
for the FRDM-KL27Z board using "all drivers".
2. Use library routines (i.e. `drivers/fsl_xxx`) from the Kinetis SDK 2.x 
whenever practical. 
3. After giving priority to 1. and 2. above, maintain file structure of the 
KL25Z bootloader sources as much as possible.

We intentionally departed from the KL25Z bootloader on a few points:

* LPUART (0 or 1) is the only supported peripheral
* Autobaud detection is disabled (or at least untested).  The baud rate is fixed,
  specified in `board/bootloader_config.h` by `BL_FEATURE_BAUD_RATE`.
* Most conditional code blocks, e.g. `#ifdef BL_FEATURE_QSPI_MODULE` have been
  left in place, _but are untested_.
* One exception: we frequently deleted code blocks under the `BOOTLOADER_HOST`
  conditional as that appears to be deprecated.
* Another exception: We didn't find any references to active code under
  `BL_FEATURE_POWERDOWN`, so we deleted the smc/ directory.

## Relevant Documents And Links
The following documents were useful during the porting process:

* [KL27 Sub-Family Reference Manual](https://www.nxp.com/docs/en/reference-manual/KL27P64M48SF2RM.pdf)
* [KL25 Sub-Family Reference Manual](https://www.nxp.com/docs/en/reference-manual/KL25P80M48SF0RM.pdf)
* [Kinetis Bootloader v2.0.0 Reference Manual](https://www.nxp.com/docs/en/reference-manual/KBTLDR200RM.pdf)
* [Kinetis Bootloader v2.0.0 Release Notes](https://www.nxp.com/docs/en/release-note/KBTLDR200RN.pdf)
* [Getting Started with Kinetis SDK (KSDK) v.2.0](https://www.nxp.com/docs/en/user-guide/KSDK20GSUG.pdf)
* [Kinetis SDK v.2.0 API Reference Manual](https://www.nxp.com/docs/en/reference-manual/KSDK20APIRM.pdf)
* [Kinetis SDK 2.0 Transition Guide](https://www.nxp.com/docs/en/user-guide/KSDK20TUG.pdf)
* [Kinetis blhost User's Guide](https://www.nxp.com/docs/en/user-guide/KBLHOSTUG.pdf)

An executable version of `blhost` is distributed in:

    NXP_Kinetis_Bootloader_2_0_0/bin/Tools/blhost/mac/blhost
    
## Other Notes

* It turned out to be crucial to add `-Xlinker --defsym=__ram_vector_table__=1` to
the KL27Z project flags.





# Size of bootloader

In general, we'd like to keep the bootloader's footprint as small as possible,
since that determines the page boundary at which the user code can start.
Currently, we're using 0x8000, but that can almost certainly be lowered.

The value of _etext is the highest address used by the bootloader; rounding up
to the next page boundary indicates where the user code can start.  Here are the
values of _etext for various compiler optimization levels and flags.

_Note: I found it was necessary to delete the Debug/ and or Release/ directories
prior to rebuilding in order to reliably get an updated xxx.map file._

                             +-----------------------------------------------------+
                             |         _etext for given optimization level         |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    | Configuration | -flto ||  -O0   |  -O1   |  -O2   |  -O3   |  -Os   |  -Og   |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Debug      |  no   || 0x82d4 | 0x5574 | 0x5510 | 0x6144 | 0x4f28 | 0x576c |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Debug      |  yes  || 0x884c | 0x4230 | 0x4104 | 0x57d8 | 0x3c30 | 0x54f4 |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Release    |  no   || 0x7194 | 0x4668 | 0x463c | 0x5294 | 0x40d8 | 0x4854 |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Release    |  yes  || 0x7198 | 0x4160 | 0x4080 | note 1 | 0x3c48 | 0x47d8 |
    +---------------+-------++--------+--------+--------+--------+--------+--------|

Note that _etext for the Release configuration with -flto (link time optimizer)
and -Os (optimize for size) is under 0x4000 -- this means we can locate our user
application at 0x4000 rather than at 0x8000 which gives us substantially more
flash memory for our code.

Note 1:

    ../source/crc/src/crc32.c: In function 'crc32_finalize':
    ../source/crc/src/crc32.c:89:6: internal compiler error: Segmentation fault: 11
     void crc32_finalize(crc32_data_t *crc32Config, uint32_t *hash)
          ^
    libbacktrace could not find executable to open
    Please submit a full bug report,
    with preprocessed source if appropriate.
    See <http://gcc.gnu.org/bugs.html> for instructions.
    lto-wrapper: arm-none-eabi-g++ returned 1 exit status
    /Applications/KDS_v3.app/Contents/toolchain/bin/../lib/gcc/arm-none-eabi/4.8.4/../../../../arm-none-eabi/bin/ld: lto-wrapper failed

