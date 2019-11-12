#include <libusb.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include "ftdi_i.h"
#include <ftdi.h>
#include "myftdi.h"

#define ftdi_error_return(code, str) do {  \
        if ( ftdi )                        \
            ftdi->error_str = str;         \
        else                               \
            fprintf(stderr, str);          \
        return code;                       \
   } while(0);



/* Based on official libftdi 1.4 implementation */

/*FTD2XX doesn't check for values not fitting in the ACBUS Signal options*/
void set_ft232h_cbus(struct ftdi_eeprom *eeprom, unsigned char * output)
{
    int i;
    for(i=0; i<5; i++)
    {
        int mode_low, mode_high;
        if (eeprom->cbus_function[2*i]> CBUSH_CLK7_5)
            mode_low = CBUSH_TRISTATE;
        else
            mode_low = eeprom->cbus_function[2*i];
        if (eeprom->cbus_function[2*i+1]> CBUSH_CLK7_5)
            mode_high = CBUSH_TRISTATE;
        else
            mode_high = eeprom->cbus_function[2*i+1];

        output[0x18+i] = (mode_high <<4) | mode_low;
    }
}
/* Return the bits for the encoded EEPROM Structure of a requested Mode
 *
 */
static unsigned char type2bit(unsigned char type, enum ftdi_chip_type chip)
{
    switch (chip)
    {
        case TYPE_2232H:
        case TYPE_2232C:
        {
            switch (type)
            {
                case CHANNEL_IS_UART: return 0;
                case CHANNEL_IS_FIFO: return 0x01;
                case CHANNEL_IS_OPTO: return 0x02;
                case CHANNEL_IS_CPU : return 0x04;
                default: return 0;
            }
        }
        case TYPE_232H:
        {
            switch (type)
            {
                case CHANNEL_IS_UART   : return 0;
                case CHANNEL_IS_FIFO   : return 0x01;
                case CHANNEL_IS_OPTO   : return 0x02;
                case CHANNEL_IS_CPU    : return 0x04;
                case CHANNEL_IS_FT1284 : return 0x08;
                default: return 0;
            }
        }
        case TYPE_R:
        {
            switch (type)
            {
                case CHANNEL_IS_UART   : return 0;
                case CHANNEL_IS_FIFO   : return 0x01;
                default: return 0;
            }
        }
        case TYPE_230X: /* FT230X is only UART */
        default: return 0;
    }
    return 0;
}

/**
    Build binary buffer from ftdi_eeprom structure.
    Output is suitable for ftdi_write_eeprom().

    \param ftdi pointer to ftdi_context

    \retval >=0: size of eeprom user area in bytes
    \retval -1: eeprom size (128 bytes) exceeded by custom strings
    \retval -2: Invalid eeprom or ftdi pointer
    \retval -3: Invalid cbus function setting     (FIXME: Not in the code?)
    \retval -4: Chip doesn't support invert       (FIXME: Not in the code?)
    \retval -5: Chip doesn't support high current drive         (FIXME: Not in the code?)
    \retval -6: No connected EEPROM or EEPROM Type unknown
*/
int my_ftdi_eeprom_build(struct ftdi_context *ftdi)
{
    unsigned char i, j, eeprom_size_mask;
    unsigned short checksum, value;
    unsigned char manufacturer_size = 0, product_size = 0, serial_size = 0;
    int user_area_size, free_start, free_end;
    struct ftdi_eeprom *eeprom;
    unsigned char * output;

    if (ftdi == NULL)
        ftdi_error_return(-2,"No context");
    if (ftdi->eeprom == NULL)
        ftdi_error_return(-2,"No eeprom structure");

    eeprom= ftdi->eeprom;
    output = eeprom->buf;

    if (eeprom->chip == -1)
        ftdi_error_return(-6,"No connected EEPROM or EEPROM type unknown");

    if (eeprom->size == -1)
    {
        if ((eeprom->chip == 0x56) || (eeprom->chip == 0x66))
            eeprom->size = 0x100;
        else
            eeprom->size = 0x80;
    }

    if (eeprom->manufacturer != NULL)
        manufacturer_size = strlen(eeprom->manufacturer);
    if (eeprom->product != NULL)
        product_size = strlen(eeprom->product);
    if (eeprom->serial != NULL)
        serial_size = strlen(eeprom->serial);

    // eeprom size check
    switch (ftdi->type)
    {
        case TYPE_AM:
        case TYPE_BM:
        case TYPE_R:
            user_area_size = 96;    // base size for strings (total of 48 characters)
            break;
        case TYPE_2232C:
            user_area_size = 90;     // two extra config bytes and 4 bytes PnP stuff
            break;
        case TYPE_230X:
            user_area_size = 88;     // four extra config bytes + 4 bytes PnP stuff
            break;
        case TYPE_2232H:            // six extra config bytes + 4 bytes PnP stuff
        case TYPE_4232H:
            user_area_size = 86;
			user_area_size += 128;
            break;
        case TYPE_232H:
            user_area_size = 80;
            break;
        default:
            user_area_size = 0;
            break;
    }
    user_area_size  -= (manufacturer_size + product_size + serial_size) * 2;

    if (user_area_size < 0)
        ftdi_error_return(-1,"eeprom size exceeded");

    // empty eeprom
    if (ftdi->type == TYPE_230X)
    {
        /* FT230X have a reserved section in the middle of the MTP,
           which cannot be written to, but must be included in the checksum */
        memset(ftdi->eeprom->buf, 0, 0x80);
        memset((ftdi->eeprom->buf + 0xa0), 0, (FTDI_MAX_EEPROM_SIZE - 0xa0));
    }
    else
    {
        memset(ftdi->eeprom->buf, 0, FTDI_MAX_EEPROM_SIZE);
    }

    // Bytes and Bits set for all Types

    // Addr 02: Vendor ID
    output[0x02] = eeprom->vendor_id;
    output[0x03] = eeprom->vendor_id >> 8;

    // Addr 04: Product ID
    output[0x04] = eeprom->product_id;
    output[0x05] = eeprom->product_id >> 8;

    // Addr 06: Device release number (0400h for BM features)
    output[0x06] = eeprom->release_number;
    output[0x07] = eeprom->release_number >> 8;

    // Addr 08: Config descriptor
    // Bit 7: always 1
    // Bit 6: 1 if this device is self powered, 0 if bus powered
    // Bit 5: 1 if this device uses remote wakeup
    // Bit 4-0: reserved - 0
    j = 0x80;
    if (eeprom->self_powered)
        j |= 0x40;
    if (eeprom->remote_wakeup)
        j |= 0x20;
    output[0x08] = j;

    // Addr 09: Max power consumption: max power = value * 2 mA
    output[0x09] = eeprom->max_power / MAX_POWER_MILLIAMP_PER_UNIT;

    if ((ftdi->type != TYPE_AM) && (ftdi->type != TYPE_230X))
    {
        // Addr 0A: Chip configuration
        // Bit 7: 0 - reserved
        // Bit 6: 0 - reserved
        // Bit 5: 0 - reserved
        // Bit 4: 1 - Change USB version
        // Bit 3: 1 - Use the serial number string
        // Bit 2: 1 - Enable suspend pull downs for lower power
        // Bit 1: 1 - Out EndPoint is Isochronous
        // Bit 0: 1 - In EndPoint is Isochronous
        //
        j = 0;
        if (eeprom->in_is_isochronous)
            j = j | 1;
        if (eeprom->out_is_isochronous)
            j = j | 2;
        output[0x0A] = j;
    }

    // Dynamic content
    // Strings start at 0x94 (TYPE_AM, TYPE_BM)
    // 0x96 (TYPE_2232C), 0x98 (TYPE_R) and 0x9a (TYPE_x232H)
    // 0xa0 (TYPE_232H)
    i = 0;
    switch (ftdi->type)
    {
        case TYPE_2232H:
        case TYPE_4232H:
            i += 2;
        case TYPE_R:
            i += 2;
        case TYPE_2232C:
            i += 2;
        case TYPE_AM:
        case TYPE_BM:
            i += 0x94;
            break;
        case TYPE_232H:
        case TYPE_230X:
            i = 0xa0;
            break;
    }
    /* Wrap around 0x80 for 128 byte EEPROMS (Internale and 93x46) */
    eeprom_size_mask = eeprom->size -1;
    free_end = i & eeprom_size_mask;

    // Addr 0E: Offset of the manufacturer string + 0x80, calculated later
    // Addr 0F: Length of manufacturer string
    // Output manufacturer
    output[0x0E] = i;  // calculate offset
    output[i & eeprom_size_mask] = manufacturer_size*2 + 2, i++;
    output[i & eeprom_size_mask] = 0x03, i++; // type: string
    for (j = 0; j < manufacturer_size; j++)
    {
        output[i & eeprom_size_mask] = eeprom->manufacturer[j], i++;
        output[i & eeprom_size_mask] = 0x00, i++;
    }
    output[0x0F] = manufacturer_size*2 + 2;

    // Addr 10: Offset of the product string + 0x80, calculated later
    // Addr 11: Length of product string
    output[0x10] = i | 0x80;  // calculate offset
    output[i & eeprom_size_mask] = product_size*2 + 2, i++;
    output[i & eeprom_size_mask] = 0x03, i++;
    for (j = 0; j < product_size; j++)
    {
        output[i & eeprom_size_mask] = eeprom->product[j], i++;
        output[i & eeprom_size_mask] = 0x00, i++;
    }
    output[0x11] = product_size*2 + 2;

    // Addr 12: Offset of the serial string + 0x80, calculated later
    // Addr 13: Length of serial string
    output[0x12] = i | 0x80; // calculate offset
    output[i & eeprom_size_mask] = serial_size*2 + 2, i++;
    output[i & eeprom_size_mask] = 0x03, i++;
    for (j = 0; j < serial_size; j++)
    {
        output[i & eeprom_size_mask] = eeprom->serial[j], i++;
        output[i & eeprom_size_mask] = 0x00, i++;
    }

    // Legacy port name and PnP fields for FT2232 and newer chips
    if (ftdi->type > TYPE_BM)
    {
        output[i & eeprom_size_mask] = 0x02; /* as seen when written with FTD2XX */
        i++;
        output[i & eeprom_size_mask] = 0x03; /* as seen when written with FTD2XX */
        i++;
        output[i & eeprom_size_mask] = eeprom->is_not_pnp; /* as seen when written with FTD2XX */
        i++;
    }

    output[0x13] = serial_size*2 + 2;

    if (ftdi->type > TYPE_AM) /* use_serial not used in AM devices */
    {
        if (eeprom->use_serial)
            output[0x0A] |= USE_SERIAL_NUM;
        else
            output[0x0A] &= ~USE_SERIAL_NUM;
    }

    /* Bytes and Bits specific to (some) types
       Write linear, as this allows easier fixing*/
    switch (ftdi->type)
    {
        case TYPE_AM:
            break;
        case TYPE_BM:
            output[0x0C] = eeprom->usb_version & 0xff;
            output[0x0D] = (eeprom->usb_version>>8) & 0xff;
            if (eeprom->use_usb_version)
                output[0x0A] |= USE_USB_VERSION_BIT;
            else
                output[0x0A] &= ~USE_USB_VERSION_BIT;

            break;
        case TYPE_2232C:

            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_2232C);
            if ( eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCP;
            else
                output[0x00] &= ~DRIVER_VCP;

            if ( eeprom->high_current_a == HIGH_CURRENT_DRIVE)
                output[0x00] |= HIGH_CURRENT_DRIVE;
            else
                output[0x00] &= ~HIGH_CURRENT_DRIVE;

            output[0x01] = type2bit(eeprom->channel_b_type, TYPE_2232C);
            if ( eeprom->channel_b_driver == DRIVER_VCP)
                output[0x01] |= DRIVER_VCP;
            else
                output[0x01] &= ~DRIVER_VCP;

            if ( eeprom->high_current_b == HIGH_CURRENT_DRIVE)
                output[0x01] |= HIGH_CURRENT_DRIVE;
            else
                output[0x01] &= ~HIGH_CURRENT_DRIVE;

            if (eeprom->in_is_isochronous)
                output[0x0A] |= 0x1;
            else
                output[0x0A] &= ~0x1;
            if (eeprom->out_is_isochronous)
                output[0x0A] |= 0x2;
            else
                output[0x0A] &= ~0x2;
            if (eeprom->suspend_pull_downs)
                output[0x0A] |= 0x4;
            else
                output[0x0A] &= ~0x4;
            if (eeprom->use_usb_version)
                output[0x0A] |= USE_USB_VERSION_BIT;
            else
                output[0x0A] &= ~USE_USB_VERSION_BIT;

            output[0x0C] = eeprom->usb_version & 0xff;
            output[0x0D] = (eeprom->usb_version>>8) & 0xff;
            output[0x14] = eeprom->chip;
            break;
        case TYPE_R:
            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_R);
            if (eeprom->high_current == HIGH_CURRENT_DRIVE_R)
                output[0x00] |= HIGH_CURRENT_DRIVE_R;
            if (eeprom->external_oscillator)
                output[0x00] |= 0x02;
            output[0x01] = 0x40; /* Hard coded Endpoint Size*/

            if (eeprom->suspend_pull_downs)
                output[0x0A] |= 0x4;
            else
                output[0x0A] &= ~0x4;
            output[0x0B] = eeprom->invert;
            output[0x0C] = eeprom->usb_version & 0xff;
            output[0x0D] = (eeprom->usb_version>>8) & 0xff;

            if (eeprom->cbus_function[0] > CBUS_BB_RD)
                output[0x14] = CBUS_TXLED;
            else
                output[0x14] = eeprom->cbus_function[0];

            if (eeprom->cbus_function[1] > CBUS_BB_RD)
                output[0x14] |= CBUS_RXLED<<4;
            else
                output[0x14] |= eeprom->cbus_function[1]<<4;

            if (eeprom->cbus_function[2] > CBUS_BB_RD)
                output[0x15] = CBUS_TXDEN;
            else
                output[0x15] = eeprom->cbus_function[2];

            if (eeprom->cbus_function[3] > CBUS_BB_RD)
                output[0x15] |= CBUS_PWREN<<4;
            else
                output[0x15] |= eeprom->cbus_function[3]<<4;

            if (eeprom->cbus_function[4] > CBUS_CLK6)
                output[0x16] = CBUS_SLEEP;
            else
                output[0x16] = eeprom->cbus_function[4];
            break;
        case TYPE_2232H:
            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_2232H);
            if ( eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCP;
            else
                output[0x00] &= ~DRIVER_VCP;

            output[0x01] = type2bit(eeprom->channel_b_type, TYPE_2232H);
            if ( eeprom->channel_b_driver == DRIVER_VCP)
                output[0x01] |= DRIVER_VCP;
            else
                output[0x01] &= ~DRIVER_VCP;
            if (eeprom->suspend_dbus7 == SUSPEND_DBUS7_BIT)
                output[0x01] |= SUSPEND_DBUS7_BIT;
            else
                output[0x01] &= ~SUSPEND_DBUS7_BIT;

            if (eeprom->suspend_pull_downs)
                output[0x0A] |= 0x4;
            else
                output[0x0A] &= ~0x4;

            if (eeprom->group0_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA;
            else
                output[0x0c] |= eeprom->group0_drive;
            if (eeprom->group0_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT;
            if (eeprom->group0_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW;

            if (eeprom->group1_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA<<4;
            else
                output[0x0c] |= eeprom->group1_drive<<4;
            if (eeprom->group1_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT<<4;
            if (eeprom->group1_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW<<4;

            if (eeprom->group2_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA;
            else
                output[0x0d] |= eeprom->group2_drive;
            if (eeprom->group2_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT;
            if (eeprom->group2_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW;

            if (eeprom->group3_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA<<4;
            else
                output[0x0d] |= eeprom->group3_drive<<4;
            if (eeprom->group3_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT<<4;
            if (eeprom->group3_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW<<4;

            output[0x18] = eeprom->chip;

            break;
        case TYPE_4232H:
            if (eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCP;
            else
                output[0x00] &= ~DRIVER_VCP;
            if (eeprom->channel_b_driver == DRIVER_VCP)
                output[0x01] |= DRIVER_VCP;
            else
                output[0x01] &= ~DRIVER_VCP;
            if (eeprom->channel_c_driver == DRIVER_VCP)
                output[0x00] |= (DRIVER_VCP << 4);
            else
                output[0x00] &= ~(DRIVER_VCP << 4);
            if (eeprom->channel_d_driver == DRIVER_VCP)
                output[0x01] |= (DRIVER_VCP << 4);
            else
                output[0x01] &= ~(DRIVER_VCP << 4);

            if (eeprom->suspend_pull_downs)
                output[0x0a] |= 0x4;
            else
                output[0x0a] &= ~0x4;

            if (eeprom->channel_a_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 0;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 0);
            if (eeprom->channel_b_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 1;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 1);
            if (eeprom->channel_c_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 2;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 2);
            if (eeprom->channel_d_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 3;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 3);

            if (eeprom->group0_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA;
            else
                output[0x0c] |= eeprom->group0_drive;
            if (eeprom->group0_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT;
            if (eeprom->group0_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW;

            if (eeprom->group1_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA<<4;
            else
                output[0x0c] |= eeprom->group1_drive<<4;
            if (eeprom->group1_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT<<4;
            if (eeprom->group1_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW<<4;

            if (eeprom->group2_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA;
            else
                output[0x0d] |= eeprom->group2_drive;
            if (eeprom->group2_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT;
            if (eeprom->group2_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW;

            if (eeprom->group3_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA<<4;
            else
                output[0x0d] |= eeprom->group3_drive<<4;
            if (eeprom->group3_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT<<4;
            if (eeprom->group3_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW<<4;

            output[0x18] = eeprom->chip;

            break;
        case TYPE_232H:
            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_232H);
            if ( eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCPH;
            else
                output[0x00] &= ~DRIVER_VCPH;
            if (eeprom->powersave)
                output[0x01] |= POWER_SAVE_DISABLE_H;
            else
                output[0x01] &= ~POWER_SAVE_DISABLE_H;

            if (eeprom->suspend_pull_downs)
                output[0x0a] |= 0x4;
            else
                output[0x0a] &= ~0x4;

            if (eeprom->clock_polarity)
                output[0x01] |= FT1284_CLK_IDLE_STATE;
            else
                output[0x01] &= ~FT1284_CLK_IDLE_STATE;
            if (eeprom->data_order)
                output[0x01] |= FT1284_DATA_LSB;
            else
                output[0x01] &= ~FT1284_DATA_LSB;
            if (eeprom->flow_control)
                output[0x01] |= FT1284_FLOW_CONTROL;
            else
                output[0x01] &= ~FT1284_FLOW_CONTROL;
            if (eeprom->group0_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA;
            else
                output[0x0c] |= eeprom->group0_drive;
            if (eeprom->group0_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT;
            if (eeprom->group0_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW;

            if (eeprom->group1_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA;
            else
                output[0x0d] |= eeprom->group1_drive;
            if (eeprom->group1_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT;
            if (eeprom->group1_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW;

            set_ft232h_cbus(eeprom, output);

            output[0x1e] = eeprom->chip;
            fprintf(stderr,"FIXME: Build FT232H specific EEPROM settings\n");
            break;
        case TYPE_230X:
            output[0x00] = 0x80; /* Actually, leave the default value */
            /*FIXME: Make DBUS & CBUS Control configurable*/
            output[0x0c] = 0;    /* DBUS drive 4mA, CBUS drive 4 mA like factory default */
            for (j = 0; j <= 6; j++)
            {
                output[0x1a + j] = eeprom->cbus_function[j];
            }
            output[0x0b] = eeprom->invert;
            break;
    }

    /* First address without use */
    free_start = 0;
    switch (ftdi->type)
    {
        case TYPE_230X:
            free_start += 2;
        case TYPE_232H:
            free_start += 6;
        case TYPE_2232H:
        case TYPE_4232H:
            free_start += 2;
        case TYPE_R:
            free_start += 2;
        case TYPE_2232C:
            free_start++;
        case TYPE_AM:
        case TYPE_BM:
            free_start += 0x14;
    }

    /* Arbitrary user data */
    if (eeprom->user_data && eeprom->user_data_size >= 0)
    {
        if (eeprom->user_data_addr < free_start)
            fprintf(stderr,"Warning, user data starts inside the generated data!\n");
        if (eeprom->user_data_addr + eeprom->user_data_size >= free_end)
            fprintf(stderr,"Warning, user data overlaps the strings area!\n");
        if (eeprom->user_data_addr + eeprom->user_data_size > eeprom->size)
            ftdi_error_return(-1,"eeprom size exceeded");
        memcpy(output + eeprom->user_data_addr, eeprom->user_data, eeprom->user_data_size);
    }

    // calculate checksum
    checksum = 0xAAAA;

    for (i = 0; i < eeprom->size/2-1; i++)
    {
        if ((ftdi->type == TYPE_230X) && (i == 0x12))
        {
            /* FT230X has a user section in the MTP which is not part of the checksum */
            i = 0x40;
        }
        if ((ftdi->type == TYPE_230X) && (i >=  0x40) && (i < 0x50)) {
            uint16_t data;
            if (ftdi_read_eeprom_location(ftdi, i, &data)) {
                fprintf(stderr, "Reading Factory Configuration Data failed\n");
                i = 0x50;
            }
            value = data;
        }
        else {
            value = output[i*2];
            value += output[(i*2)+1] << 8;
        }
        checksum = value^checksum;
        checksum = (checksum << 1) | (checksum >> 15);
    }

    output[eeprom->size-2] = checksum;
    output[eeprom->size-1] = checksum >> 8;

    eeprom->initialized_for_connected_device = 1;
    return user_area_size;
}
