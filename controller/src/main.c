/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/drivers/spi.h>

#define SPI_FAST_DEV	DT_COMPAT_GET_ANY_STATUS_OKAY(spi_bus)

#define SPI_OP SPI_OP_MODE_MASTER | SPI_MODE_CPOL | 0 | \
	       SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE


struct spi_dt_spec spi_fast = SPI_DT_SPEC_GET(SPI_FAST_DEV, SPI_OP, 0);

/* to run this test, connect MOSI pin to the MISO of the SPI */