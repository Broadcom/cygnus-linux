/*
 * Portions Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef SPI_BCM_MSPI_H
#define SPI_BCM_MSPI_H

#define BSPI_REVISION_ID                0x000
#define BSPI_SCRATCH                    0x004
#define BSPI_MAST_N_BOOT_CTRL           0x008
#define BSPI_BUSY_STATUS                0x00c
#define BSPI_INTR_STATUS                0x010
#define BSPI_B0_STATUS                  0x014
#define BSPI_B0_CTRL                    0x018
#define BSPI_B1_STATUS                  0x01c
#define BSPI_B1_CTRL                    0x020
#define BSPI_STRAP_OVERRIDE_CTRL        0x024
#define BSPI_FLEX_MODE_ENABLE           0x028
#define BSPI_BITS_PER_CYCLE             0x02c
#define BSPI_BITS_PER_PHASE             0x030
#define BSPI_CMD_AND_MODE_BYTE          0x034
#define BSPI_BSPI_FLASH_UPPER_ADDR_BYTE 0x038
#define BSPI_BSPI_XOR_VALUE             0x03c
#define BSPI_BSPI_XOR_ENABLE            0x040
#define BSPI_BSPI_PIO_MODE_ENABLE       0x044
#define BSPI_BSPI_PIO_IODIR             0x048
#define BSPI_BSPI_PIO_DATA              0x04c

/* RAF */
#define RAF_START_ADDR                  0x100
#define RAF_NUM_WORDS                   0x104
#define RAF_CTRL                        0x108
#define RAF_FULLNESS                    0x10c
#define RAF_WATERMARK                   0x110
#define RAF_STATUS                      0x114
#define RAF_READ_DATA                   0x118
#define RAF_WORD_CNT                    0x11c
#define RAF_CURR_ADDR                   0x120

/* MSPI */
#define MSPI_SPCR0_LSB                  0x200
#define MSPI_SPCR0_MSB                  0x204
#define MSPI_SPCR1_LSB                  0x208
#define MSPI_SPCR1_MSB                  0x20c
#define MSPI_NEWQP                      0x210
#define MSPI_ENDQP                      0x214
#define MSPI_SPCR2                      0x218
#define MSPI_SPCR2_SPE                  0x00000040
#define MSPI_SPCR2_CONT_AFTER_CMD       0x00000080
#define MSPI_STATUS                     0x220
#define MSPI_STATUS_SPIF                0x00000001
#define MSPI_CPTQP                      0x224
#define MSPI_TXRAM                      0x240 /* 32 registers, up to 0x2b8 */
#define MSPI_RXRAM                      0x2c0 /* 32 registers, up to 0x33c */
#define MSPI_CDRAM                      0x340 /* 16 registers, up to 0x37c */
#define MSPI_CDRAM_PCS_PCS0             0x00000001
#define MSPI_CDRAM_PCS_PCS1             0x00000002
#define MSPI_CDRAM_PCS_PCS2             0x00000004
#define MSPI_CDRAM_PCS_PCS3             0x00000008
#define MSPI_CDRAM_PCS_DISABLE_ALL      0x0000000f
#define MSPI_CDRAM_PCS_DSCK             0x00000010
#define MSPI_CDRAM_BITSE                0x00000040
#define MSPI_CDRAM_CONT                 0x00000080
#define MSPI_WRITE_LOCK                 0x380
#define MSPI_DISABLE_FLUSH_GEN          0x384

/* Interrupt */
#define INTR_RAF_LR_FULLNESS_REACHED    0x3a0
#define INTR_RAF_LR_TRUNCATED           0x3a4
#define INTR_RAF_LR_IMPATIENT           0x3a8
#define INTR_RAF_LR_SESSION_DONE        0x3ac
#define INTR_RAF_LR_OVERREAD            0x3b0
#define INTR_MSPI_DONE                  0x3b4
#define INTR_MSPI_HALT_SET_TRANSACTION_DONE  0x3b8

#endif
