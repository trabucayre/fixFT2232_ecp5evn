/* ftdi_cbus_config.c
 * tool used to change FT232/FT230X CBUS pins configuration
 * 
 * (C) 2015-2019 by Gwenhael Goavec-Merou <gwen@trabucayre.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <stdio.h>
#include <ftdi.h>
#include <libusb.h>
#include <stdlib.h>
#include "myftdi.h"

int main(int argc, char **argv)
{
	int ret, i;
	struct ftdi_context *ftdi;
	int vendor_id = 0x403, product_id = 0x6010;
	int dont_write = 0;

	if (argc < 3) {
		printf("%s -v vid -p pid [-n]\n", argv[0]);
		printf("   -v default: 0x403\n");
		printf("   -p default: 0x6010\n");
		printf("   -n to not write into FTDI EEPROM\n");
		return EXIT_FAILURE;
	}

	for (i = 1; i < argc; i+=2) {
		if (argv[i][1] == 'v')
			sscanf(argv[i+1], "%x", &vendor_id);
		if (argv[i][1] == 'p')
			sscanf(argv[i+1], "%x", &product_id);
		if (argv[i][1] == 'n')
			dont_write = 1;
	}

	printf("vendor %x product %x\n", vendor_id, product_id);

	int decode_verbose = 1;

	if ((ftdi = ftdi_new()) == NULL) {
		printf("FTDI context allocation failed\n");
		return EXIT_FAILURE;
	}

	if (ftdi_usb_open(ftdi, vendor_id, product_id) != 0) {
		printf("FTDI open failed: %s\n", ftdi_get_error_string(ftdi));
		goto cleanup;
	}

	/* fetch EEPROM from device */
	ret = ftdi_read_eeprom(ftdi);
	if (ret != 0) {
		printf("FTDI read EEPROM failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}

	/* decode original EEPROM and display details */
	printf("\n\nDefault configuration\n\n");

	ret = ftdi_eeprom_decode(ftdi, decode_verbose);
	if (ret != 0) {
		printf("FTDI decode EEPROM failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}

	/* set GROUP2 (BL/BDBUSx drive) */
    ret = ftdi_set_eeprom_value(ftdi, GROUP2_DRIVE, DRIVE_4MA);
	if (ret != 0) {
		printf("FTDI set GROUP2 drive 4mA failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}
	/* set GROUP2 (BL/BDBUSx slew rate) */
    ret = ftdi_set_eeprom_value(ftdi, GROUP2_SLEW, SLOW_SLEW);
	if (ret != 0) {
		printf("FTDI set GROUP2 slow slew failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}
	/* set GROUP3 (BH/BCBUSx drive) */
    ret = ftdi_set_eeprom_value(ftdi, GROUP3_DRIVE, DRIVE_4MA);
	if (ret != 0) {
		printf("FTDI set GROUP3 drive 4mA failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}
	/* set GROUP3 (BH/BCBUSx slew rate) */
    ret = ftdi_set_eeprom_value(ftdi, GROUP3_SLEW, SLOW_SLEW);
	if (ret != 0) {
		printf("FTDI set GROUP3 slow slew failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}
	/* set INTERFACE_B as UART VCP */
	ret = ftdi_set_eeprom_value(ftdi, CHANNEL_B_TYPE, CHANNEL_IS_UART);
	if (ret != 0) {
		printf("FTDI set InterfaceB as UART failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}
	ret = ftdi_set_eeprom_value(ftdi, CHANNEL_B_DRIVER, DRIVER_VCP);
	if (ret != 0) {
		printf("FTDI set InterfaceB VCP failed: %s\n",
		       ftdi_get_error_string(ftdi));
		goto cleanup;
	}

	/* generate new EEPROM */
	/* my_ftdi_eeprom_build is a modified version */
	ret = my_ftdi_eeprom_build(ftdi);
	if (ret < 0) {
		printf("FTDI EEPROM_build failed: %d %s\n", ret,
		       ftdi_get_error_string(ftdi));
		//goto cleanup;
	}

	if (decode_verbose) {
		/* decode original EEPROM and display details */
		printf("\n\nNew configuration\n\n");
		ftdi_eeprom_decode(ftdi, 1);
	}

	if (dont_write == 0) {
		/* flash the new EEPROM into SPI flash */
		ret = ftdi_write_eeprom(ftdi);
		if (ret != 0) {
			printf("FTDI write EEPROM failed: %d %s\n", ret,
			       ftdi_get_error_string(ftdi));
			goto cleanup;
		}
	}

	libusb_reset_device(ftdi->usb_dev);
	printf("EEPROM updated\n");
cleanup:
	printf("FTDI close: %d\n", ftdi_usb_close(ftdi));

	ftdi_deinit(ftdi);
	ftdi_free(ftdi);

	return EXIT_SUCCESS;
}
