# fixFT2232_ecp5evn
Tool to fix FT2232's uart interface configuration for
[ecp5evn (LFE5UM5G-85F-EVN)](https://www.latticesemi.com/products/developmentboardsandkits/ecp5evaluationboard) and
[CrossLink-NX EVN
(LIFCL-40-EVN)](https://www.latticesemi.com/en/Products/DevelopmentBoardsAndKits/CrossLink-NXEvaluationBoard)
boards.

An SPI flash is used by FT2232 to store device configuration. By default, with the ecp5 evn board, FT2232 interface B used as UART is configured in FIFO mode. With this behavior it's not possible to communicate with the FPGA.

This tool as for goal to update EEPROM content, with correct mode, slew rate and drive.

**Note:** libftdi1 has a limited ftdi_eeprom_build function, the user area space not take into account eeprom size, so this repository provides an adapted implementation dedicated to 256 Bytes EEPROM.

The new FT2232 behavior has been tested with
[soc_ecp5_evn](https://github.com/SymbiFlow/prjtrellis/tree/master/examples/soc_ecp5_evn)
and fix issue [RISCV example not working](https://github.com/SymbiFlow/prjtrellis/issues/83)

## compile

This application uses **libftdi1**, so this library must be installed (and, depending of the distribution, headers too)
```bash
apt-get install libftdi1-2 libftdi1-dev
```
and if not already done, install **pkg-config**, **make** and **gcc**.

To build the app:
```bash
$ make
```

## Usage
```bash
./fixFT2232_ecp5evn -v vid -p pid [-n]
   -v default: 0x403
   -p default: 0x6010
   -n to not write into FTDI EEPROM
```
